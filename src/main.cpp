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

// エンコーダカウント → mm変換
float scale_x = 0.05f; // 1count あたり何mm動くか
float scale_y = 0.05f;

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
const int16_t PWM_LIMIT = 2999; // pwmの最大値
PositionPID pid_x(0.6, 0.1, 0.001, -PWM_LIMIT, PWM_LIMIT, -100, 100);
PositionPID pid_y(0.6, 0.1, 0.001, -PWM_LIMIT, PWM_LIMIT, -100, 100);
PositionPID pid_theta(35.0, 1.5, 0.001, -PWM_LIMIT, PWM_LIMIT, -100, 100);
const int16_t AUTO_PWM_LIMIT = 1200;

// 自動制御の速度制限
float auto_vx = 0.0f;
float auto_vy = 0.0f;

// 加速度制限(mm/s^2)
const float AUTO_ACCEL = 300.0f;
const float AUTO_MAX_V = 100.0f;

// 目標座標
int target_x = 0;
int target_y = 0;
float target_theta = 0.0f; // rad
const float ERROR = 25.0f; // 目標位置±25mmで停止

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

    for (int i = 0; i < 4; i++)
      motor[i] = 0;

    auto_vx = 0.0f;
    auto_vy = 0.0f;

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

    pid_x.reset(x);
    pid_y.reset(y);
    pid_theta.reset(theta);

    auto_vx = 0.0f;
    auto_vy = 0.0f;

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
    pid_x.reset(x);
    pid_y.reset(y);
    pid_theta.reset(theta);
    auto_vx = 0.0f;
    auto_vy = 0.0f;
    Serial.printf("Target Absolute : %d %d %.3f rad\n", target_x, target_y, target_theta);
    break;
  }

  case 0x40: // set_shoot
  {
    Serial.printf(
        "Shoot setting: PWM=%ld duration=%.3f\n",
        (long)recvMsg.param1,
        recvMsg.param3);

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

    CAN.beginPacket(0x102);
    for (int i = 0; i < 8; i++)
    {
      CAN.write(data[i]);
    }
    CAN.endPacket();
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
  // CAN受信
  int packetSize = CAN.parsePacket();
  static uint8_t rx[8] = {0};          // 足回りエンコーダーデータ
  static uint8_t rx_syasyutu[8] = {0}; // 射出エンコーダーデータ
  if (packetSize == 8 && CAN.packetId() == 0x101)
  {
    for (int i = 0; i < 8; i++)
    {
      rx[i] = CAN.read();
    }
  }

  if (packetSize == 8 && CAN.packetId() == 0x104)
  {
    for (int i = 0; i < 8; i++)
    {
      rx_syasyutu[i] = CAN.read();
    }
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
    int16_t dc1 = count_1 - prev_count_1;
    int16_t dc2 = count_2 - prev_count_2;
    int16_t dc3 = count_3 - prev_count_3;

    // 前回のカウント取得
    prev_count_1 = count_1;
    prev_count_2 = count_2;
    prev_count_3 = count_3;
    prev_count_4 = count_4;

    // printf("count1 = %d, count2 = %d, count3 = %d, count4 = %d\n", count_1, count_2, count_3, count_4);

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

      if (distance < 100.0f)
      {
        max_v = 50.0f;
      }
      else if (distance < 200.0f)
      {
        max_v = 100.0f;
      }

      // 目標位置までの速度を計算
      float vx_global = pid_x.update(target_x, x, dt);
      float vy_global = pid_y.update(target_y, y, dt);

      vx_global = constrain(vx_global, -max_v, max_v);
      vy_global = constrain(vy_global, -max_v, max_v);

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
        // 微小出力をカット
        if (v[i] > 1.0f)
        {
          v[i] += FRICTION_OFFSET;
        }
        else if (v[i] < -1.0f)
        {
          v[i] -= FRICTION_OFFSET;
        }
        else
        {
          v[i] = 0.0f;
        }

        motor[i] = (int16_t)constrain(v[i], -AUTO_PWM_LIMIT, AUTO_PWM_LIMIT);
      }

      if (!esp_now_connected)
      {
        for (int i = 0; i < 4; i++)
        {
          motor[i] = 0;
        }
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