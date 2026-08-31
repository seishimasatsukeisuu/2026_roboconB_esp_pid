#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include "SpeedPID.h"
#include <math.h>

#include <esp_now.h>
#include <WiFi.h>

// 受信側のMACアドレスを入れる
uint8_t receiverMac[] = {0x48, 0xE7, 0x29, 0xA3, 0xBF, 0xCC};
bool esp_now_send_available = true;

// PS4入力
int lx;
int ly;
int up_straight;
int down_straight;
int r_straight;
int l_straight;
int l2;
int r2;
// int crossState;

int circleState = false; // ベル直
int lastcircleState = false;

int triangleState = false; // ちょっとだけ前進
int lasttriangleState = false;
int crossState = false; // ちょっと後退
int lastcrossState = false;

// CAN送信用
int16_t motor[4] = {0};

// CANデータ計算用
float vx;
float vy;
float rot;

// 前回のエンコーダー値保持
int16_t prev_count_1;
int16_t prev_count_2;
int16_t prev_count_3;

// 現在位置
float x = 0.0f;     // count_1(前後方向)
float y = 0.0f;     // count_2(左右方向)
float theta = 0.0f; // count_3(回転)

// エンコーダカウント → mm変換
float scale_x = 0.05f; // 1count あたり何mm動くか
float scale_y = 0.05f;

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
const int16_t PWM_LIMIT = 2999; // pwmの最大値
SpeedPID pid_x(1.0, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT);
SpeedPID pid_y(1.0, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT);
SpeedPID pid_theta(1.0, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT); // 5
const int16_t AUTO_PWM_LIMIT = 1200;

// 自動制御の速度制限
float auto_vx = 0.0f;
float auto_vy = 0.0f;

// 加速度制限(mm/s^2)
const float AUTO_ACCEL = 300.0f;

// 目標座標
int target_x = 0;
int target_y = 0;
float target_theta = 0.0f; // rad

const float ERROR = 25.0f;

// ロボット中心からE1,E3までの距離
const float L = 355.0f; // mm

// モード切り替え
bool auto_mode = 0;

// エンコーダ1回転あたりのカウント数
const double ENC_COUNTS_PER_REV = 4096.0 * 2.;

const float wheel_radius = 30.0f; // 計測輪半径(mm) 30mm
const float mm_per_count = 2.0f * PI * wheel_radius / ENC_COUNTS_PER_REV;

// ギア比(補正係数)実際の回転数に変換するため
const double GEAR_RATIO = 1.;

// 制御周期：20000μs = 20ms
const long CONTROL_CYCLE = 20000;
const float dt = CONTROL_CYCLE * 1.0e-6f;

// espnowのやつ
typedef struct
{
  int target_x;
  int target_y;
  float target_theta;
} TargetData;

void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  esp_now_send_available = true;
}
TargetData recvTarget;

void OnDataRecv(const uint8_t *mac,
                const uint8_t *incomingData,
                int len)
{
  if (len == sizeof(TargetData))
  {
    memcpy(&recvTarget, incomingData, sizeof(TargetData));

    // 相対位置に変更
    float dx = (float)recvTarget.target_x;
    float dy = (float)recvTarget.target_y;

    target_x = (int)(x + dx * cosf(theta) - dy * sinf(theta));
    target_y = (int)(y + dx * sinf(theta) + dy * cosf(theta) * (-1));
    target_theta = theta + recvTarget.target_theta;

    auto_mode = 1;
    pid_x.reset();
    pid_y.reset();
    pid_theta.reset();

    auto_vx = 0.0f;
    auto_vy = 0.0f;

    Serial.printf("Target : %d %d %.2f\n",
                  target_x,
                  target_y,
                  target_theta);
  }
}

void setup()
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("My MAC = ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP初期化失敗");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ピア追加失敗");
    return;
  }

  Serial.println("文字を入力してください");

  esp_now_register_send_cb(OnDataSend);
  esp_now_register_recv_cb(OnDataRecv); // 受信コールバック登録

  while (!Serial)
    ;
  PS4.begin("00:02:5b:00:a5:ac");

  CAN.setPins(4, 5);      // 16,17ピンはつかえない(4,5ピンは使えた)
  if (!CAN.begin(1000E3)) // 1000kbpsで開始
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;

  Serial.println("Ready");
}

void loop()
{

  if (auto_mode == 0) // 手動入力(PS4)
  {
    if (PS4.isConnected())
    {
      lx = -PS4.LStickX();
      ly = PS4.LStickY();
      up_straight = PS4.Up();
      down_straight = PS4.Down();
      r_straight = PS4.Right();
      l_straight = PS4.Left();
      l2 = -PS4.L2Value();
      r2 = -PS4.R2Value();
      // crossState = PS4.Cross();

      circleState = PS4.data.button.circle;

      triangleState = PS4.data.button.triangle;
      crossState = PS4.data.button.cross;

      if (circleState && !lastcircleState)
      {
        printf("osita\r\n");
        CAN.beginPacket(0x102);

        CAN.write(1);

        CAN.endPacket();
      }
      lastcircleState = circleState;

      if (triangleState && !lasttriangleState)
      {
        printf("osita\r\n");
        CAN.beginPacket(0x105);

        CAN.write(2);

        CAN.endPacket();
      }
      lasttriangleState = triangleState;

      if (crossState && !lastcrossState)
      {
        printf("osita\r\n");
        CAN.beginPacket(0x105);

        CAN.write(3);

        CAN.endPacket();
      }
      lastcrossState = crossState;

      vx = ly;
      vy = lx;

      if (abs(vx) < 10)
        vx = 0;
      if (abs(vy) < 10)
        vy = 0;
      if (abs(l2) < 10)
        l2 = 0;
      if (abs(r2) < 10)
        r2 = 0;
      if (up_straight)
      {
        vx = 100;
        vy = 0;
      }
      if (down_straight)
      {
        vx = -100;
        vy = 0;
      }
      if (r_straight)
      {
        vx = 0;
        vy = -100;
      }
      if (l_straight)
      {
        vx = 0;
        vy = 100;
      }

      constexpr float INV_SQRT2 = 0.70710678f;
      rot = (r2 - l2) * 0.25f;
      float gain = 20.0f;

      float v1 = ((-vx + vy) * INV_SQRT2 + rot) * gain;
      float v2 = ((vx + vy) * INV_SQRT2 + rot) * gain;
      float v3 = ((-vx - vy) * INV_SQRT2 + rot) * gain;
      float v4 = ((vx - vy) * INV_SQRT2 + rot) * gain;

      float v[4] = {v1, v2, v3, v4};

      for (int i = 0; i < 4; i++)
      {
        motor[i] = (int16_t)constrain(v[i], -2999, 2999);
      }

      // static bool prev_cross = false;

      // if (crossState && !prev_cross)
      // {
      //   auto_mode = !auto_mode;

      //   pid_x.reset();
      //   pid_y.reset();
      //   pid_theta.reset();
      // }

      // prev_cross = crossState;
    }

    if (!PS4.isConnected())
    {
      for (int i = 0; i < 4; i++)
        motor[i] = 0;
    }
  }

  // CAN受信
  int packetSize = CAN.parsePacket();
  static uint8_t rx[8] = {0};
  if (packetSize == 8 && CAN.packetId() == 0x101)
  {
    for (int i = 0; i < 8; i++)
    {
      rx[i] = CAN.read();
      // Serial.println(rx[i]);
    }
  }
  // else
  // {
  //   Serial.println("CAN dekinu");
  // }

  static uint32_t last_control = 0;
  static uint32_t last_can_tx = 0;
  if (last_control == 0)
  {
    last_control = micros();
    last_can_tx = micros();
    return;
  }

  // サンプリング制御(一定周期でだけ実行)
  unsigned long now_us = micros();
  if (now_us - last_control >= CONTROL_CYCLE)
  {
    last_control += CONTROL_CYCLE;

    // エンコーダー値取得
    int16_t count_1 =
        (int16_t)(((uint16_t)rx[0] << 8) | rx[1]) * (-1);

    int16_t count_2 =
        (int16_t)(((uint16_t)rx[2] << 8) | rx[3]);

    int16_t count_3 =
        (int16_t)(((uint16_t)rx[4] << 8) | rx[5]);

    // prev_countの初期化用
    static bool first = true;
    if (first)
    {
      prev_count_1 = count_1;
      prev_count_2 = count_2;
      prev_count_3 = count_3;
      first = false;
      return;
    }

    // 増分(速度に対応)
    int16_t dc1 = count_1 - prev_count_1;
    int16_t dc2 = count_2 - prev_count_2;
    int16_t dc3 = count_3 - prev_count_3;

    // 前回のカウント取得
    prev_count_1 = count_1;
    prev_count_2 = count_2;
    prev_count_3 = count_3;

    // printf("count1 = %d, count2 = %d, count3 = %d\n", count_1, count_2, count_3);

    float s1 = dc1 * mm_per_count;
    float s2 = dc2 * mm_per_count;
    float s3 = dc3 * mm_per_count;

    // Serial.printf(
    //     "dc1=%d dc2=%d dc3=%d | s1=%.2f s2=%.2f s3=%.2f\n",
    //     dc1, dc2, dc3,
    //     s1, s2, s3);

    // 自己位置更新
    x += (s1 + s3) / 2;
    y += s2;
    theta += (s3 - s1) / (2.0f * L); // ←ここでradになる

    const float PI_F = 3.14159265f;

    while (theta > PI_F)
      theta -= 2 * PI_F;

    while (theta < -PI_F)
      theta += 2 * PI_F;

    if (auto_mode == 1) // 自動入力(PID)
    {
      // 目標位置までの速度を計算
      float vx_global = pid_x.update(target_x, x, dt);
      float vy_global = pid_y.update(target_y, y, dt);

      // 速度制限
      float max_delta_v = AUTO_ACCEL * dt;
      float dvx = vx_global - auto_vx;
      float dvy = vy_global - auto_vy;

      // X方向の速度変化を制限
      if (dvx > max_delta_v)
        dvx = max_delta_v;

      if (dvx < -max_delta_v)
        dvx = -max_delta_v;

      // Y方向の速度変化を制限
      if (dvy > max_delta_v)
        dvy = max_delta_v;

      if (dvy < -max_delta_v)
        dvy = -max_delta_v;

      auto_vx += dvx;
      auto_vy += dvy;

      // 回転計算
      vx = auto_vx * cosf(theta) + auto_vy * sinf(theta);
      vy = -auto_vx * sinf(theta) + auto_vy * cosf(theta);

      // ±180° に収める
      float err_theta = target_theta - theta;
      while (err_theta > PI_F)
        err_theta -= 2 * PI_F;
      while (err_theta < -PI_F)
        err_theta += 2 * PI_F;

      // 誤差0を目標にPID
      rot = pid_theta.update(0, -err_theta, dt);

      constexpr float INV_SQRT2 = 0.70710678f;

      float gain = 8.0f;

      float v1 = ((-vx + vy) * INV_SQRT2 + rot) * gain;
      float v2 = ((vx + vy) * INV_SQRT2 + rot) * gain;
      float v3 = ((-vx - vy) * INV_SQRT2 + rot) * gain;
      float v4 = ((vx - vy) * INV_SQRT2 + rot) * gain;

      float v[4] = {v1, v2, v3, v4};

      for (int i = 0; i < 4; i++)
      {
        motor[i] = (int16_t)constrain(v[i], -AUTO_PWM_LIMIT, AUTO_PWM_LIMIT);
      }

      Serial.printf(
          "x=%.1f y=%.1f theta=%.3f | vx=%.2f vy=%.2f rot=%.2f | motor=%d %d %d %d\n",
          x, y, theta,
          vx, vy, rot,
          motor[0], motor[1], motor[2], motor[3]);

      // 到達判定(目標位置に到達したことを判定して自動制御を終了する)
      if (fabsf(target_x - x) < ERROR &&
          fabsf(target_y - y) < ERROR &&
          fabsf(err_theta) < 5.0f * PI_F / 180.0f) // 5度をラジアンに変換
      {
        for (int i = 0; i < 4; i++)
          motor[i] = 0;

        auto_vx = 0.0f;
        auto_vy = 0.0f;

        pid_x.reset();
        pid_y.reset();
        pid_theta.reset();

        // auto_mode = 0;
      }
    }
  }

  // CAN送信

  if (micros() - last_can_tx >= 20000)
  {
    last_can_tx = micros();

    CAN.beginPacket(0x103);

    for (int i = 0; i < 4; i++)
    {
      CAN.write((uint8_t)(motor[i] >> 8));
      CAN.write((uint8_t)(motor[i] & 0xFF));
    }

    CAN.endPacket();
  }
}