#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include "SpeedPID.h"

// PS4入力
int lx;
int ly;
int up_straight;
int down_straight;
int r_straight;
int l_straight;
int l2;
int r2;
int crossState;

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
float x;     // count_1(前後方向)
float y;     // count_2(左右方向)
float theta; // count_3(回転)

// エンコーダカウント → mm変換
float scale_x = 0.05f; // 1count あたり何mm動くか
float scale_y = 0.05f;
float scale_theta = 0.1f; // 1countあたり何deg(rad)か

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
const int16_t PWM_LIMIT = 2999; // pwmの最大値
SpeedPID pid_x(0.05, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT);
SpeedPID pid_y(0.05, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT);
SpeedPID pid_theta(0.05, 0.0, 0.001, -PWM_LIMIT, PWM_LIMIT);

// 目標座標
int target_x = 200;
int target_y = 0;
int target_theta = 0;

// モード切り替え
bool auto_mode = 0;

// エンコーダ1回転あたりのカウント数
const double ENC_COUNTS_PER_REV = 4096.0 * 2.;

// ギア比(補正係数)実際の回転数に変換するため
const double GEAR_RATIO = 1.;

// 制御周期：20000μs = 20ms
const long CONTROL_CYCLE = 20000;
const float dt = CONTROL_CYCLE * 1.0e-6f;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  PS4.begin("E4:65:B8:7E:05:4A");

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
      lx = PS4.LStickX();
      ly = PS4.LStickY();
      up_straight = PS4.Up();
      down_straight = PS4.Down();
      r_straight = PS4.Right();
      l_straight = PS4.Left();
      l2 = PS4.L2Value();
      r2 = PS4.R2Value();
      crossState = PS4.Cross();

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
        vy = 100;
      }
      if (l_straight)
      {
        vx = 0;
        vy = -100;
      }

      constexpr float INV_SQRT2 = 0.70710678f;
      rot = (l2 - r2);
      float gain = 20.0f;

      float v1 = ((-vx + vy) * INV_SQRT2 + rot) * gain;
      float v2 = ((vx + vy) * INV_SQRT2 + rot) * gain;
      float v3 = ((vx - vy) * INV_SQRT2 + rot) * gain;
      float v4 = ((-vx - vy) * INV_SQRT2 + rot) * gain;

      float v[4] = {v1, v2, v3, v4};

      for (int i = 0; i < 4; i++)
      {
        motor[i] = (int16_t)constrain(v[i], -2999, 2999);
      }

      static bool prev_cross = false;

      if (crossState && !prev_cross)
      {
        auto_mode = !auto_mode;

        pid_x.reset();
        pid_y.reset();
        pid_theta.reset();
      }

      prev_cross = crossState;
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
  if (packetSize == 8 && CAN.packetId() == 0x100)
  {
    for (int i = 0; i < 8; i++)
    {
      rx[i] = CAN.read();
    }
  }

  static uint32_t last_control = 0;

  if (last_control == 0)
  {
    last_control = micros();
    return;
  }

  // サンプリング制御(一定周期でだけ実行)
  unsigned long now_us = micros();
  if (now_us - last_control >= CONTROL_CYCLE)
  {
    last_control += CONTROL_CYCLE;

    // エンコーダー値取得
    int16_t count_1 =
        (int16_t)(((uint16_t)rx[0] << 8) | rx[1]);

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

    // 自己位置更新
    x += dc1 * scale_x;
    y += dc2 * scale_y;
    theta += dc3 * scale_theta;

    while (theta >= 360)
      theta -= 360;
    while (theta < 0)
      theta += 360;
    while (target_theta >= 360)
      target_theta -= 360;
    while (target_theta < 0)
      target_theta += 360;

    if (auto_mode == 1) // 自動入力(PID)
    {
      vx = pid_x.update(target_x, x, dt);
      vy = pid_y.update(target_y, y, dt);

      // ±180° に収める
      float err_theta = target_theta - theta;
      while (err_theta > 180)
        err_theta -= 360;
      while (err_theta < -180)
        err_theta += 360;

      // 誤差0を目標にPID
      rot = pid_theta.update(0, -err_theta, dt);

      constexpr float INV_SQRT2 = 0.70710678f;

      float gain = 1;

      float v1 = ((-vx + vy) * INV_SQRT2 + rot) * gain;
      float v2 = ((vx + vy) * INV_SQRT2 + rot) * gain;
      float v3 = ((vx - vy) * INV_SQRT2 + rot) * gain;
      float v4 = ((-vx - vy) * INV_SQRT2 + rot) * gain;

      float v[4] = {v1, v2, v3, v4};

      for (int i = 0; i < 4; i++)
      {
        motor[i] = (int16_t)constrain(v[i], -2999, 2999);
      }

      // 到達判定(目標位置に到達したことを判定して自動制御を終了する)
      if (abs(target_x - x) < 100 &&
          abs(target_y - y) < 100 &&
          abs(err_theta) < 30)
      {
        for (int i = 0; i < 4; i++)
          motor[i] = 0;

        pid_x.reset();
        pid_y.reset();
        pid_theta.reset();

        auto_mode = 0;
      }
    }
  }

  // CAN送信
  CAN.beginPacket(0x101);
  for (int i = 0; i < 4; i++)
  {
    CAN.write((uint8_t)(motor[i] >> 8));
    CAN.write((uint8_t)(motor[i] & 0xFF));
  }
  CAN.endPacket();

  /*
  //デバック用
  // 回転数計算
  double rpm = dc1 * GEAR_RATIO / ENC_COUNTS_PER_REV / dt * 60;

  static uint32_t last_print = 0;

  // エンコーダー値確認用
  if (millis() - last_print > 100)
  {
      last_print = millis();

      printf("%ld %ld %ld\n", count_1, count_2, count_3);
  }

  // エンコーダーから速度取得
  int vel1 = dc1 / dt;
  int vel2 = dc2 / dt;

  printf(
        "x=%.1f y=%.1f th=%.1f  tx=%d ty=%d tth=%d\n",
        x,y,theta,
        target_x,target_y,target_theta
    );
         */
}