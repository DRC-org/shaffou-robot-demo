// CWMC (中央制御基板) の設定。実機調整はこのファイルだけで済むようにしている。
#pragma once

#include <stdint.h>

namespace cfg {

// ---- PS5 コントローラ ----
// コントローラ本体の Bluetooth MAC アドレス (README の手順で調べて書き換える)
constexpr const char* kPs5Mac = "14:3A:9A:2E:AC:11";

// ---- ピン ----
constexpr int kCanRxPin = 4;
constexpr int kCanTxPin = 16;
// LCD (HD44780 互換 16x2, 4bit 接続): RS, EN, D4, D5, D6, D7
constexpr int kLcdRs = 27, kLcdEn = 14, kLcdD4 = 15, kLcdD5 = 13, kLcdD6 = 5, kLcdD7 = 17;

// ---- DC モータ (r, θ, z の順) ----
// 回転方向が逆なら -1 にする
constexpr int8_t kAxisDir[3] = {-1, +1, +1};
// 速度モードごとの最大 duty [%]
constexpr float kDutyLow = 30.0f;
constexpr float kDutyHigh = 50.0f;
// duty の変化率 [%/s] (急発進・急停止を防ぐ)
constexpr float kDutyRampPerSec = 150.0f;
// スティックのデッドゾーン (0..127)
constexpr int kStickDeadzone = 12;

// ---- サーボ ----
constexpr float kRotateMinDeg = 0.0f;
constexpr float kRotateMaxDeg = 180.0f;
constexpr float kRotateInitDeg = 90.0f;
constexpr float kRotateSpeedDegPerSec = 90.0f;  // 右スティックいっぱいで回る速さ (低速モードはこの半分)

constexpr float kGripOpenDeg = 30.0f;    // 開いたときの角度 (要調整)
constexpr float kGripCloseDeg = 120.0f;  // 閉じたときの角度 (要調整)
constexpr float kGripSpeedDegPerSec = 120.0f;

// ---- LCD ----
constexpr uint32_t kLcdPeriodMs = 200;

}  // namespace cfg
