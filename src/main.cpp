#include <Arduino.h>

/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
#define DXL_SERIAL Serial
#define DEBUG_SERIAL soft_serial
const int DXL_DIR_PIN = 2;     // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
#define DXL_SERIAL Serial
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL SerialUSB
const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
#define DXL_SERIAL Serial3       // OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22; // OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
// For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
// Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB) // When using OpenRB-150
// OpenRB does not require the DIR control pin.
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = -1;
#endif

#define TOUCH1 0

const uint8_t DXL_ID = 21;
const float DXL_PROTOCOL_VERSION = 2.0;

// 2つの角度間の最小差分を計算する関数のプロトタイプ宣言
float calculateAngleDifference(float angle1, float angle2);
// 角度を0〜360度の範囲に正規化する関数のプロトタイプ宣言
float normalizeAngle(float angle);

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

// タッチセンサーの状態を管理する変数
bool touchState = false;
bool prevTouchState = false;
bool rotationDirection = true; // true: 反時計回り, false: 時計回り

// ポジションモードでの設定
const float TARGET_ANGLE = 180.0;           // 目標回転角度
const float POSITION_THRESHOLD = 1.0;       // 位置判定のしきい値（度）
const float APPROACH_THRESHOLD = 10.0;      // 接近判定のしきい値（度）
const int32_t GOAL_CURRENT_LIMIT = 1000;    // 電流制限値（mA）
const int32_t APPROACH_CURRENT_LIMIT = 300; // 接近時の電流制限値（mA）
const int32_t CURRENT_THRESHOLD = 500;      // 電流検出しきい値（mA）
const float VELOCITY = 10.0;                // 回転速度（RPM）

// サーボの状態
enum ServoState
{
  IDLE,     // 待機中
  ROTATING, // 回転中
};

ServoState currentState = IDLE;

// 開始位置を記録
float startPosition = 0.0;
float targetPosition = 0.0;
float initialPosition = 0.0; // 初期位置を保存

unsigned long lastActionTime = 0;
unsigned long lastCheckTime = 0; // 最後にポジションをチェックした時間

void setup()
{
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.begin(1000000);

  // トルクをオフにする（操作モード変更のため）
  dxl.torqueOff(DXL_ID);
  delay(100);

  // Set operating mode to current mode
  dxl.setOperatingMode(DXL_ID, OP_CURRENT);
  delay(100);

  // タッチセンサーピンを入力に設定
  pinMode(TOUCH1, INPUT_PULLDOWN);

  // 初期位置を取得
  dxl.torqueOff(DXL_ID);
  delay(100);
  initialPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);

  // 初期設定完了メッセージ
  DEBUG_SERIAL.println("セットアップを開始します");
  DEBUG_SERIAL.print("初期位置(度): ");
  DEBUG_SERIAL.println(initialPosition);
  DEBUG_SERIAL.println("サーボの初期化完了。トルクはオフ状態です。");
  DEBUG_SERIAL.println("タッチセンサーを押すと反時計回りに180度回転します。");
}

void loop()
{
  // タッチセンサーの状態を読み取る
  touchState = digitalRead(TOUCH1) == HIGH;

  // 現在の位置を取得
  float currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);

  // 現在の時間を取得
  unsigned long currentTime = millis();

  // サーボの状態に応じた処理
  switch (currentState)
  {
  case IDLE:
    // if (currentTime - lastCheckTime >= 100)
    // {
    //   // チェック時間を更新
    //   lastCheckTime = currentTime;
    //   DEBUG_SERIAL.print("現在位置(度): ");
    //   DEBUG_SERIAL.println(currentPosition);
    // }

    // タッチセンサーの状態が変化した場合
    if (touchState && !prevTouchState)
    {
      DEBUG_SERIAL.println("タッチセンサーが押されました");

      // トルクをオンにする
      dxl.torqueOn(DXL_ID);
      DEBUG_SERIAL.println("トルクをオンにしました");

      // トルクがオンになるまで少し待機
      delay(100);

      // 初期位置を基準にして目標位置を計算（固定位置に移動）
      if (rotationDirection)
      {
        // 反時計回り位置（初期位置+180度）
        DEBUG_SERIAL.println("反時計回り回転を開始します");
        targetPosition = normalizeAngle(initialPosition + TARGET_ANGLE);
        // 反時計回りに回転するための電流値を設定（正の値）
        dxl.setGoalCurrent(DXL_ID, GOAL_CURRENT_LIMIT);
        DEBUG_SERIAL.print("設定電流値(mA): ");
        DEBUG_SERIAL.println(GOAL_CURRENT_LIMIT);
        DEBUG_SERIAL.print("初期位置(度): ");
        DEBUG_SERIAL.println(initialPosition);
        DEBUG_SERIAL.print("目標位置(度): ");
        DEBUG_SERIAL.println(targetPosition);
      }
      else
      {
        // 時計回り位置（初期位置-180度）
        DEBUG_SERIAL.println("時計回り回転を開始します");
        targetPosition = normalizeAngle(initialPosition);
        // 時計回りに回転するための電流値を設定（負の値）
        dxl.setGoalCurrent(DXL_ID, -GOAL_CURRENT_LIMIT);
        DEBUG_SERIAL.print("設定電流値(mA): ");
        DEBUG_SERIAL.println(-GOAL_CURRENT_LIMIT);
        DEBUG_SERIAL.print("初期位置(度): ");
        DEBUG_SERIAL.println(initialPosition);
        DEBUG_SERIAL.print("目標位置(度): ");
        DEBUG_SERIAL.println(targetPosition);
      }

      // 状態を回転中に変更
      currentState = ROTATING;

      // 最終アクション時間とチェック時間を更新
      lastActionTime = currentTime;
      lastCheckTime = currentTime;
    }
    break;

  case ROTATING:
    // 100msごとに位置を確認（前回のチェックから100ms以上経過したら）
    if (currentTime - lastCheckTime >= 10)
    {
      // チェック時間を更新
      lastCheckTime = currentTime;

      // 現在の位置と電流値を取得
      float currentPosition = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
      int16_t currentCurrent = dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);

      DEBUG_SERIAL.print("現在位置(度): ");
      DEBUG_SERIAL.println(currentPosition);
      DEBUG_SERIAL.print("目標位置(度): ");
      DEBUG_SERIAL.println(targetPosition);
      DEBUG_SERIAL.print("現在電流(mA): ");
      DEBUG_SERIAL.println(currentCurrent);

      // 目標位置までの角度差を計算
      float angleDiff = calculateAngleDifference(currentPosition, targetPosition);

      // デバッグ情報を出力
      DEBUG_SERIAL.print("目標までの差(度): ");
      DEBUG_SERIAL.println(angleDiff);

      // 目標位置に近づいたら電流値を下げてスピードを調整
      if (angleDiff < APPROACH_THRESHOLD && angleDiff >= POSITION_THRESHOLD)
      {
        // 接近モードに入ったことを通知
        if (abs(currentCurrent) > APPROACH_CURRENT_LIMIT)
        {
          if (rotationDirection)
          {
            // 反時計回り（正の電流値）を維持しながら電流値を下げる
            dxl.setGoalCurrent(DXL_ID, APPROACH_CURRENT_LIMIT);
            DEBUG_SERIAL.println("目標に接近: 電流値を下げました（反時計回り）");
          }
          else
          {
            // 時計回り（負の電流値）を維持しながら電流値を下げる
            dxl.setGoalCurrent(DXL_ID, -APPROACH_CURRENT_LIMIT);
            DEBUG_SERIAL.println("目標に接近: 電流値を下げました（時計回り）");
          }
          DEBUG_SERIAL.print("調整後電流値(mA): ");
          DEBUG_SERIAL.println(APPROACH_CURRENT_LIMIT);
        }
      }

      // 目標位置に到達した場合、または電流値が閾値を超えた場合に停止
      if (angleDiff < POSITION_THRESHOLD || abs(currentCurrent) > CURRENT_THRESHOLD)
      {
        // 停止理由を表示
        if (angleDiff < POSITION_THRESHOLD)
        {
          DEBUG_SERIAL.println("目標角度に到達しました");
        }
        else
        {
          DEBUG_SERIAL.println("電流値が閾値を超えました - 障害物検出");
        }

        // 電流を0に設定して停止
        dxl.setGoalCurrent(DXL_ID, 0);
        delay(300);

        // トルクをオフにする
        dxl.torqueOff(DXL_ID);

        DEBUG_SERIAL.println("回転完了。トルクをオフにしました");

        // 次回は逆方向に回転するよう設定
        rotationDirection = !rotationDirection;

        if (rotationDirection)
        {
          DEBUG_SERIAL.println("次回タッチで反時計回り回転します");
        }
        else
        {
          DEBUG_SERIAL.println("次回タッチで時計回り回転します");
        }

        // 状態を待機中に戻す
        currentState = IDLE;
      }
      // タイムアウト処理（安全対策）- 10秒経過しても到達しない場合
      else if (currentTime - lastActionTime > 10000)
      {
        // トルクをオフにする
        dxl.torqueOff(DXL_ID);

        DEBUG_SERIAL.println("タイムアウト: 目標角度に到達できませんでした");
        DEBUG_SERIAL.println("回転を停止します。トルクをオフにしました");

        // 次回は逆方向に回転するよう設定
        rotationDirection = !rotationDirection;

        // 状態を待機中に戻す
        currentState = IDLE;
      }
    }
    break;
  }

  // タッチセンサーの前回の状態を更新
  prevTouchState = touchState;

  // タッチセンサーが押された時に時間を記録
  if (touchState && !prevTouchState)
  {
    lastActionTime = currentTime;
  }

  // 少し待機
  // delay(10);
}

// 2つの角度間の最小差分を計算する関数（0〜360度の範囲で）
float calculateAngleDifference(float angle1, float angle2)
{
  // 両方の角度を0〜360度の範囲に正規化
  angle1 = normalizeAngle(angle1);
  angle2 = normalizeAngle(angle2);

  float diff = fabs(angle1 - angle2);
  if (diff > 180.0)
  {
    diff = 360.0 - diff;
  }
  return diff;
}

// 角度を0〜360度の範囲に正規化する関数
float normalizeAngle(float angle)
{
  // 負の角度を正の角度に変換
  while (angle < 0.0)
  {
    angle += 360.0;
  }

  // 360度以上の角度を0〜360度の範囲に収める
  return fmod(angle, 360.0);
}