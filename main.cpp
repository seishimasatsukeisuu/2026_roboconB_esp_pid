#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include "AnglePID.h"
#include "SpeedPID.h"

int lx;
int ly;
int up_straight;
int down_straight;
int r_straight;
int l_straight;
int l2;
int r2;
int crossState;

int target_lx;
int target_ly;

const int8_t Sign_Correction = 1; // 符号補正、モーターの取り付け方向によって+-を変える

const long CONTROL_CYCLE = 20000; // 制御周期：20000μs = 20ms
long prev_count_1 = 0;            // 前回のエンコーダー値

double angle_filtered = 0;
double rpm_filtered = 0;

const double target_rpm = 60;   // 目標rpm値
const double target_angle = 90; // 目標角度(度)
const int16_t PWM_LIMIT = 2999; // pwmの最大値

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
AnglePID angle_pid_1(2., 0.3, 0.05, -PWM_LIMIT, PWM_LIMIT, 360., 100., 0.);

// PID制御器(Kp(比例), Ki(積分), Kd(微分), pwm出力制限)
SpeedPID speed_pid_1(1.5, 3., 0., -PWM_LIMIT, PWM_LIMIT);

const double ENC_COUNTS_PER_REV = 4096.0 * 2.; // エンコーダ1回転あたりのカウント数
const double GEAR_RATIO = 1.;                  // ギア比(補正係数)実際の回転数に変換するため

bool is_dir = HIGH;
void Motor(int sign, int pwm_signed)
{
  int duty = sign * pwm_signed; // 符号補正
  // 回転方向決定
  if (duty > 0)
  {
    is_dir = HIGH;
  }
  else
  {
    is_dir = LOW;
  }
}

unsigned long last = micros(); // 前回の時刻を記録

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

  last = micros(); // 時刻リセット
}

const long CONTROL_CYCLE = 20000; // 制御周期：20000μs = 20ms
long prev_count_1 = 0;            // 前回のエンコーダー値

void loop()
{
  int16_t motor[4] = {0};

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

    if (abs(lx) < 10)
      lx = 0;
    if (abs(ly) < 10)
      ly = 0;
    if (abs(l2) < 10)
      l2 = 0;
    if (abs(r2) < 10)
      r2 = 0;

    if (crossState == HIGH)
    {
      memset(motor, 0, sizeof(motor));
    }
    else
    {
      constexpr float INV_SQRT2 = 0.70710678f;
      int rot = (l2 - r2);
      float gain = 20.0f;

      if (up_straight)
      {
        lx = 0;
        ly = 255;
      }
      if (down_straight)
      {
        lx = 0;
        ly = -255;
      }
      if (r_straight)
      {
        lx = 255;
        ly = 0;
      }
      if (l_straight)
      {
        lx = -255;
        ly = 0;
      }

      float v1 = ((-lx + ly) * INV_SQRT2 + rot) * gain * is_dir;
      float v2 = ((lx + ly) * INV_SQRT2 + rot) * gain * is_dir;
      float v3 = ((lx - ly) * INV_SQRT2 + rot) * gain * is_dir;
      float v4 = ((-lx - ly) * INV_SQRT2 + rot) * gain * is_dir;

      float v[4] = {v1, v2, v3, v4};

      for (int i = 0; i < 4; i++)
      {
        motor[i] = (int16_t)constrain(v[i], -2999, 2999);
      }
    }
  }

  CAN.beginPacket(0x101);

  for (int i = 0; i < 4; i++)
  {
    CAN.write((uint8_t)(motor[i] >> 8));
    CAN.write((uint8_t)(motor[i] & 0xFF));
  }

  CAN.endPacket();

  int packetSize = CAN.parsePacket();
  uint8_t rx[8] = {0};
  if (packetSize == 8 && CAN.packetId() == 0x100)
  {
    for (int i = 0; i < 8; i++)
    {
      rx[i] = CAN.read();
    }
  }

  int32_t count_1 =
      ((int32_t)rx[0] << 24) |
      ((int32_t)rx[1] << 16) |
      ((int32_t)rx[2] << 8) |
      ((int32_t)rx[3]);

  int32_t count_2 =
      ((int32_t)rx[4] << 24) |
      ((int32_t)rx[5] << 16) |
      ((int32_t)rx[6] << 8) |
      ((int32_t)rx[7]);

  static uint32_t last = 0;

  if (millis() - last > 100)
  {
    last = millis();

    printf("%ld %ld\n", count_1, count_2);
  }

  // サンプリング制御(一定周期でだけ実行)
  unsigned long now = micros();
  if (now - last < CONTROL_CYCLE)
    return;

  // dt = 0.02秒
  last += CONTROL_CYCLE;
  double dt = CONTROL_CYCLE * 1e-6;

  // 現在のカウント取得
  long c1 = count_1;
  long c2 = count_2;

  // カウント → 角度変換
  double angle = fmod((double)c1 / ENC_COUNTS_PER_REV * 360.0, 360.0);
  if (angle < 0)
    angle += 360.0;

  // 誤差計算
  double error = target_angle - angle;
  // wrap
  if (error > 180)
    error -= 360;
  if (error < -180)
    error += 360;
  // デッドバンド
  if (fabs(error) < 2.0)
  {
    Motor(Sign_Correction, 0);
    // angle_pid_1.reset();//resetでI項を0にする → pwmが小さくなる → 摩擦を突破できずに止まってしまう
    return;
  }
  // PID制御
  double output_1 = angle_pid_1.update(target_angle, angle, dt);
  // 出力
  Motor(Sign_Correction, (int)(lround(output_1)));

  // デバック出力
  Serial.printf("angle:%f filtered:%f target:%f c1:%d pwm:%f\n", angle, angle_filtered, target_angle, c1, output_1);
  // 増分(速度に対応)
  const long dc1 = c1 - prev_count_1;
  // 更新
  prev_count_1 = c1;

  // 回転数計算
  double rpm = dc1 * GEAR_RATIO / ENC_COUNTS_PER_REV / dt * 60;

  // フィルタ
  double alpha = 0.2; // 0〜1（小さいほどなめらか）
  rpm_filtered = alpha * rpm + (1 - alpha) * rpm_filtered;

  // PID制御
  double output_1 = speed_pid_1.update(target_rpm, rpm_filtered, dt);

  Motor(Sign_Correction, (int)(lround(output_1)));

  // デバック出力
  Serial.printf("rpm:%f filtered:%f target:%f c1:%d pwm:%f\n", rpm, rpm_filtered, target_rpm, c1, output_1);

  double x = 0;

  double meter_per_count =
      (0.05 * M_PI) / ENC_COUNTS_PER_REV; // 半径5cmのホイールの場合

  // 移動量計算(今どこにいるかがわかる)
  long dc = c1 - prev_count_1;
  x += dc * meter_per_count;

  // 目標位置
  double target_x = 1.0; // 1m先

  // P制御だけ使う
  float Kp = 200.0;
  double error_x = target_x - x;
  double vx = Kp * error_x;

  // pwm最大値制限
  vx = constrain(vx, -255, 255);

  // 前進のみの場合
  lx = 0; // 1m先まで前進
  ly = vx;

  // 自己位置更新
  long dc1 = c1 - prev_count_1;
  prev_count_1 = c1;

  x += dc1 * meter_per_count;

  // 位置制御
  double error_x = target_x - x;

  double auto_v = 200.0 * error_x;

  auto_v = constrain(auto_v, -255, 255);

  // 自動入力
  lx = 0;
  ly = auto_v;
}

// PS4入力
uint16_t lx;
uint16_t ly;
u_int16_t rot;

// 各モーターのpwm計算
constexpr float INV_SQRT2 = 0.70710678f;
float wheel_target = (-lx + ly) * INV_SQRT2 + rot;
float wheel_target = (lx + ly) * INV_SQRT2 + rot;
float wheel_target = (lx - ly) * INV_SQRT2 + rot;
float wheel_target = (-lx - ly) * INV_SQRT2 + rot;

// SpeedPID
pwm = speedPID(target_rpm, actual_rpm);

// CAN送信
motor[i] = pwm;
target_rpm[i] = v[i];

// 自動移動
lx = Kp * (target_x - x);
ly = Kp * (target_y - y);
// 手動時
lx = PS4.LStickX();
ly = PS4.LStickY();