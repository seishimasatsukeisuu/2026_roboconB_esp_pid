#include <Arduino.h>
#include <CAN.h>
#include "Pid.h"
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

// 受信側のMACアドレスを入れる
// uint8_t receiverMac[] = {0x48, 0xE7, 0x29, 0xA3, 0xBF, 0xCC};//パソコン
uint8_t receiverMac[] = {0x08, 0xB6, 0x1F, 0xED, 0x5E, 0x34}; // タブレット
bool esp_now_send_available = true;

// CAN pwm送信用
int16_t motor[4] = {0};
// ベル直データ送信用
uint8_t data[8] = {0};
uint8_t shoot_flag = 0;
// 足回りエンコーダーデータ受信用
static uint8_t rx[8] = {0};
const int CAN_ID_WHEEL_ENC = 0x101;
int last_wheel_can_rx = 0;
// 射出エンコーダーデータ受信用
static uint8_t rx_syasyutu[8] = {0};
const int CAN_ID_SHOOT_ENC = 0x104;
int last_shoot_can_rx = 0;

int packetSize = 0;

// CAN受信タイムアウト
const uint32_t CAN_RX_TIMEOUT_MS = 100;

// CANデータ計算用
float vx;
float vy;
float rot;

// 前回のエンコーダー値保持
int16_t prev_count_1;
int16_t prev_count_2;
int16_t prev_count_3;
int16_t prev_count_4; // ベル直のエンコーダー

// 現在位置
float x = 0.0f;     // count_1(前後方向)
float y = 0.0f;     // count_2(左右方向)
float theta = 0.0f; // count_3(回転)

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
const int16_t PWM_LIMIT = 2999; // pwmの最大値
PositionPID pid_x(0.4, 0.1, 0.05, -PWM_LIMIT, PWM_LIMIT, -100, 100);
PositionPID pid_y(0.4, 0.1, 0.05, -PWM_LIMIT, PWM_LIMIT, -100, 100);
PositionPID pid_theta(30.0, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT, -100, 100);
const int16_t AUTO_PWM_LIMIT = 2999;

// 自動制御の速度制限
float auto_vx = 0.0f;
float auto_vy = 0.0f;

float auto_ax = 0.0f;
float auto_ay = 0.0f;

// 加速度制限(mm/s^2)
const float AUTO_MAX_V = 400.0f;
const float AUTO_ACCEL = 1400.0f;
const float AUTO_DECEL = 1400.0f; // 減速
const float AUTO_JERK = 15000.0f; // mm/s^3

// 目標座標
int target_x = 0;
int target_y = 0;
float target_theta = 0.0f; // rad
const float ERROR = 10.0f; // 目標位置±10mmで停止
float err_theta = 0.0f;    // 目標との差を計算(±1°で停止)

// ロボット中心からE1,E3までの距離
const float L = 355.0f; // mm

//  入力モード切り替え
bool auto_mode = 0;

// エンコーダ1回転あたりのカウント数
const double ENC_COUNTS_PER_REV = 4096.0 * 2.;

// 計測輪半径(mm) 30mm
const float wheel_radius = 30.0f;
const float mm_per_count = 2.0f * PI * wheel_radius / ENC_COUNTS_PER_REV;

// ギア比(補正係数)実際の回転数に変換するため
const double GEAR_RATIO = 1.;

// ロボットが動き出す最低PWM値
constexpr float FRICTION_OFFSET = 50.0f;

// 制御周期：20000μs = 20ms
const long CONTROL_CYCLE = 20000;
const float dt = CONTROL_CYCLE * 1.0e-6f;

// ESP-NOW送信周期：100ms
const unsigned long ESP_NOW_TX_CYCLE = 100000;
unsigned long last_esp_now_tx = 0;

// マニュアル移動モード
bool manual_mode = false;
float manual_vx_dir = 0.0f;
float manual_vy_dir = 0.0f;
float manual_rot_dir = 0.0f;
uint32_t last_manual_cmd_time = 0;

// マニュアル操作速度
const float MANUAL_SPEED = 500.0f;
const float MANUAL_ROT_SPEED = 1.0f;

// マニュアルモード移動のタイムアウト時間(ms)
const uint32_t MANUAL_TIMEOUT_MS = 300;

// espnow
typedef struct __attribute__((packed))
{
  uint8_t command_type;
  int32_t param1;
  int32_t param2;
  float param3;
} EspNowMessage;

EspNowMessage recvMsg;

bool esp_now_connected = false;

void OnDataSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  esp_now_send_available = true;
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    esp_now_connected = true;
  }
  else
  {
    esp_now_connected = false;
    Serial.println("ESP-NOW DISCONNECTED");
  }
}
EspNowMessage recvTarget;

void OnDataRecv(const uint8_t *mac,
                const uint8_t *incomingData,
                int len)
{
  if (len != sizeof(EspNowMessage))
    return;

  memcpy(&recvMsg, incomingData, sizeof(EspNowMessage));

  switch (recvMsg.command_type)
  {
  case 0x0A: // 緊急停止
    Serial.println("Emergency Stop");
    auto_mode = 0;
    manual_mode = false; // マニュアルモードも強制解除
    for (int i = 0; i < 4; i++)
      motor[i] = 0;
    auto_vx = 0.0f;
    auto_vy = 0.0f;
    auto_ax = 0.0f;
    auto_ay = 0.0f;
    break;

  case 0x10: // 座標指示
  {
    float dx = (float)recvMsg.param1;

    // 座標系を合わせる必要があるなら反転
    float dy = (float)recvMsg.param2 * (-1.0f);

    // ロボット座標 → グローバル座標
    target_x = (int)(x + dx * cosf(theta) - dy * sinf(theta));

    target_y = (int)(y + dx * sinf(theta) + dy * cosf(theta));

    // すでにradへ変換されている
    target_theta = theta + recvMsg.param3;

    auto_mode = 1;

    // pid_x.reset(x);
    // pid_y.reset(y);
    // pid_theta.reset(theta);

    // auto_vx = 0.0f;
    // auto_vy = 0.0f;

    // Serial.printf(
    //     "Target : %d %d %.3f rad\n",
    //     target_x,
    //     target_y,
    //     target_theta);

    break;
  }

  case 0x11:
  {
    // 絶対座標指定: 変換せずそのまま目標値に
    target_x = recvMsg.param1;
    target_y = recvMsg.param2;
    target_theta = recvMsg.param3; // ラジアンの絶対角度

    auto_mode = 1;
    // pid_x.reset(x);
    // pid_y.reset(y);
    // pid_theta.reset(theta);
    // auto_vx = 0.0f;
    // auto_vy = 0.0f;
    //  Serial.printf("Target Absolute : %d %d %.3f rad\n", target_x, target_y, target_theta);
    break;
  }

  case 0x40: // set_shoot
  {
    // Serial.printf(
    //     "Shoot setting: PWM=%ld duration=%.3f\n",
    //     (long)recvMsg.param1,
    //     recvMsg.param3);

    int32_t pwm = recvMsg.param1;
    float duration = recvMsg.param3;

    // param1: int32_t → 4byte
    memcpy(&data[0], &pwm, sizeof(int32_t));
    // param3: float → 4byte
    memcpy(&data[4], &duration, sizeof(float));

    if (!esp_now_connected)
    {
      for (int i = 0; i < 8; i++)
      {
        data[i] = 0;
      }
    }
    shoot_flag = 1;

    break;
  }

  case 0x50: // マニュアル移動
  {
    manual_vx_dir = (float)recvMsg.param1;
    manual_vy_dir = (float)recvMsg.param2;
    manual_rot_dir = recvMsg.param3;

    if (manual_vx_dir == 0.0f && manual_vy_dir == 0.0f && manual_rot_dir == 0.0f)
    {
      manual_mode = false;
      for (int i = 0; i < 4; i++)
        motor[i] = 0;
    }
    else
    {
      manual_mode = true;
      auto_mode = 0;
      last_manual_cmd_time = millis();
    }
    break;
  }

  default:
    // Serial.printf(
    //     "Unknown command: 0x%02X\n",
    //     recvMsg.command_type);
    break;
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

  esp_now_register_send_cb(OnDataSend);
  esp_now_register_recv_cb(OnDataRecv); // 受信コールバック登録

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
  bool can_wheel_ok =
      (last_wheel_can_rx != 0) &&
      (millis() - last_wheel_can_rx < CAN_RX_TIMEOUT_MS);

  if (!can_wheel_ok)
  {
    auto_mode = 0;
    manual_mode = false;

    auto_vx = 0.0f;
    auto_vy = 0.0f;
    auto_ax = 0.0f;
    auto_ay = 0.0f;

    for (int i = 0; i < 4; i++)
    {
      motor[i] = 0;
    }
  }

  // CAN受信
  while ((packetSize = CAN.parsePacket()) > 0)
  {
    int id = CAN.packetId();

    if (packetSize != 8)
    {
      while (CAN.available())
        CAN.read();

      continue;
    }

    if (id == CAN_ID_WHEEL_ENC)
    {
      for (int i = 0; i < 8; i++)
      {
        rx[i] = CAN.read();
      }

      last_wheel_can_rx = millis();
    }
    else if (id == CAN_ID_SHOOT_ENC)
    {
      for (int i = 0; i < 8; i++)
      {
        rx_syasyutu[i] = CAN.read();
      }

      last_shoot_can_rx = millis();
    }
    else
    {
      while (CAN.available())
        CAN.read();
    }
  }

  if (shoot_flag == 1)
  {
    CAN.beginPacket(0x102);
    for (int i = 0; i < 8; i++)
    {
      CAN.write(data[i]);
    }
    CAN.endPacket();
    shoot_flag = 0;
  }

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

    int16_t count_4 =
        (int16_t)(((uint16_t)rx_syasyutu[0] << 8) | rx_syasyutu[1]);

    // prev_countの初期化用
    static bool first = true;
    if (first)
    {
      prev_count_1 = count_1;
      prev_count_2 = count_2;
      prev_count_3 = count_3;
      prev_count_4 = count_4;
      first = false;
      return;
    }

    // 増分
    int16_t dc1 =
        (int16_t)((uint16_t)count_1 - (uint16_t)prev_count_1);

    int16_t dc2 =
        (int16_t)((uint16_t)count_2 - (uint16_t)prev_count_2);

    int16_t dc3 =
        (int16_t)((uint16_t)count_3 - (uint16_t)prev_count_3);

    // 前回のカウント取得
    prev_count_1 = count_1;
    prev_count_2 = count_2;
    prev_count_3 = count_3;
    prev_count_4 = count_4;

    printf("count1 = %d, count2 = %d, count3 = %d, count4 = %d\n", count_1, count_2, count_3, count_4);

    float s1 = dc1 * mm_per_count;
    float s2 = dc2 * mm_per_count;
    float s3 = dc3 * mm_per_count;

    // 自己位置更新
    float dx_local = (s1 + s3) * (-1) * 0.5f;
    float dy_local = s2;
    float dtheta = (s3 - s1) / (2.0f * L);

    x += dx_local * cosf(theta) - dy_local * sinf(theta);
    y += dx_local * sinf(theta) + dy_local * cosf(theta);
    theta += dtheta;

    const float PI_F = 3.14159265f;

    while (theta > PI_F)
      theta -= 2 * PI_F;

    while (theta < -PI_F)
      theta += 2 * PI_F;

    // 座標をESP-NOWで100msごとに送信
    if (now_us - last_esp_now_tx >= ESP_NOW_TX_CYCLE)
    {
      last_esp_now_tx = now_us;

      EspNowMessage encMsg = {0};

      encMsg.command_type = 0x20;
      encMsg.param1 = (int32_t)x;
      encMsg.param2 = (int32_t)y;
      encMsg.param3 = (float)(theta * 180.0f / 3.14159265f);

      if (esp_now_send_available)
      {
        esp_now_send_available = false;

        esp_err_t result = esp_now_send(
            receiverMac,
            (uint8_t *)&encMsg,
            sizeof(encMsg));

        if (result == ESP_OK)
        {
          Serial.println("SUCCSES_SEND");
        }
        else
        {
          esp_now_send_available = true;
          Serial.printf("ESP-NOW send error: %d\n", result);
        }
      }
    }

    if (auto_mode == 1) // 自動入力(PID)
    {
      float error_x = target_x - x;
      float error_y = target_y - y;
      float distance = sqrtf(error_x * error_x + error_y * error_y);
      float max_v = AUTO_MAX_V;

      // if (distance < 100.0f)
      // {
      //   max_v = 100.0f;
      // }
      // else if (distance < 200.0f)
      // {
      //   max_v = 250.0f;
      // }

      // 位置PID
      float vx_global = pid_x.update(target_x, x, dt);
      float vy_global = pid_y.update(target_y, y, dt);

      vx_global = constrain(vx_global, -max_v, max_v);
      vy_global = constrain(vy_global, -max_v, max_v);

      // ジャーク制限付き速度制御

      // 目標速度に追従するために必要な加速度
      float desired_ax = (vx_global - auto_vx) / dt;
      float desired_ay = (vy_global - auto_vy) / dt;

      // 方向転換時は減速側を優先
      if (auto_vx * vx_global < 0.0f)
      {
        desired_ax = constrain(desired_ax, -AUTO_DECEL, AUTO_DECEL);
      }
      else
      {
        desired_ax = constrain(desired_ax, -AUTO_ACCEL, AUTO_ACCEL);
      }

      if (auto_vy * vy_global < 0.0f)
      {
        desired_ay = constrain(desired_ay, -AUTO_DECEL, AUTO_DECEL);
      }
      else
      {
        desired_ay = constrain(desired_ay, -AUTO_ACCEL, AUTO_ACCEL);
      }

      // ジャーク制限
      float max_da = AUTO_JERK * dt;

      // X
      float da_x = desired_ax - auto_ax;
      da_x = constrain(da_x, -max_da, max_da);
      auto_ax += da_x;

      // Y
      float da_y = desired_ay - auto_ay;
      da_y = constrain(da_y, -max_da, max_da);
      auto_ay += da_y;

      // 加速度から速度を更新

      auto_vx += auto_ax * dt;
      auto_vy += auto_ay * dt;

      // 最大速度制限
      auto_vx = constrain(auto_vx, -AUTO_MAX_V, AUTO_MAX_V);
      auto_vy = constrain(auto_vy, -AUTO_MAX_V, AUTO_MAX_V);
      // グローバル座標 → ロボット座標
      vx = auto_vx * cosf(theta) + auto_vy * sinf(theta);
      vy = -auto_vx * sinf(theta) + auto_vy * cosf(theta);

      // 角度誤差計算
      err_theta = target_theta - theta;

      while (err_theta > PI_F)
        err_theta -= 2 * PI_F;
      while (err_theta < -PI_F)
        err_theta += 2 * PI_F;

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
        // // 微小出力をカット
        // if (v[i] > 1.0f)
        // {
        //   v[i] += FRICTION_OFFSET;
        // }
        // else if (v[i] < -1.0f)
        // {
        //   v[i] -= FRICTION_OFFSET;
        // }
        // else
        // {
        //   v[i] = 0.0f;
        // }

        motor[i] = (int16_t)constrain(v[i], -AUTO_PWM_LIMIT, AUTO_PWM_LIMIT);
      }

      // 到達判定
      if (fabsf(target_x - x) < ERROR &&
          fabsf(target_y - y) < ERROR &&
          fabsf(err_theta) < 1.0f * PI_F / 180.0f &&
          fabsf(auto_vx) < 20.0f &&
          fabsf(auto_vy) < 20.0f) // 1度をラジアンに変換
      {
        auto_vx = 0.0f;
        auto_vy = 0.0f;
        auto_ax = 0.0f;
        auto_ay = 0.0f;

        for (int i = 0; i < 4; i++)
          motor[i] = 0;

        auto_mode = 0;
      }
    }

    else if (manual_mode) // マニュアル入力
    {
      // タイムアウトで停止
      if (millis() - last_manual_cmd_time > MANUAL_TIMEOUT_MS)
      {
        manual_mode = false;
        for (int i = 0; i < 4; i++)
        {
          motor[i] = 0;
        }
      }
      else
      {
        float vx_m = manual_vx_dir * MANUAL_SPEED;
        float vy_m = manual_vy_dir * MANUAL_SPEED;
        float rot_m = manual_rot_dir * MANUAL_ROT_SPEED;

        constexpr float INV_SQRT2 = 0.70710678f;
        float gain = 8.0f;

        float v1 = ((-vx_m + vy_m) * INV_SQRT2 + rot_m) * gain;
        float v2 = ((vx_m + vy_m) * INV_SQRT2 + rot_m) * gain;
        float v3 = ((-vx_m - vy_m) * INV_SQRT2 + rot_m) * gain;
        float v4 = ((vx_m - vy_m) * INV_SQRT2 + rot_m) * gain;

        float v[4] = {v1, v2, v3, v4};

        for (int i = 0; i < 4; i++)
        {
          motor[i] = (int16_t)constrain(v[i], -AUTO_PWM_LIMIT, AUTO_PWM_LIMIT);
        }
      }
    }

    if (!esp_now_connected)
    {
      for (int i = 0; i < 4; i++)
      {
        motor[i] = 0;
      }
    }

    // CAN送信
    CAN.beginPacket(0x103);

    for (int i = 0; i < 4; i++)
    {
      CAN.write((uint8_t)(motor[i] >> 8));
      CAN.write((uint8_t)(motor[i] & 0xFF));
    }

    CAN.endPacket();
  }
}