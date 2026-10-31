// 全基板共通の CAN プロトコル定義
//
// フレーム: 標準 ID (11bit)、データ長 8 バイト、1 Mbps
//   byte0      : command
//   byte1..4   : value (int32, ビッグエンディアン)
//   byte5..7   : 予備 (0)
//
// ID 割り当て
//   0x100 + axis : CWMC -> DCMD  指令 (axis = DIP スイッチ値, 0:r 1:θ 2:z)
//   0x180 + axis : DCMD -> CWMC  状態 (100 ms 周期)
//   0x300        : CWMC -> SVMD  サーボ角度指令
#pragma once

#include <stdint.h>

namespace proto {

constexpr uint8_t kNumAxes = 3;
enum Axis : uint8_t { kAxisR = 0, kAxisTheta = 1, kAxisZ = 2 };

constexpr uint32_t kDcmdCmdBaseId = 0x100;
constexpr uint32_t kDcmdStatusBaseId = 0x180;
constexpr uint32_t kSvmdCmdId = 0x300;

// ---- DCMD 向けコマンド ----
// kDcmdSetDuty : value = duty[%] x 100  (例: 30.00% -> 3000, 範囲 -10000..10000)
// kDcmdStop    : value は無視。即停止
enum DcmdCommand : uint8_t {
    kDcmdSetDuty = 0x00,
    kDcmdStop = 0x01,
};

// ---- DCMD からの状態通知 ----
// command = kDcmdStatus, value = StatusFlag のビット和
constexpr uint8_t kDcmdStatus = 0x80;
enum StatusFlag : uint32_t {
    kFlagEstop = 1u << 0,       // 非常停止 (REF) が作動中
    kFlagCmdTimeout = 1u << 1,  // 指令が途絶えて停止中
    kFlagWaitRearm = 1u << 2,   // 非常停止解除後、duty 0 の指令待ち
};

// ---- SVMD 向けコマンド ----
// command = チャンネル番号, value = 角度[deg] x 10  (例: 90.0° -> 900)
enum SvmdChannel : uint8_t {
    kSvRotate = 0,  // エンドエフェクタ回転 (SV0)
    kSvGrip = 1,    // 把持 (SV1)
};

// 受信側 (DCMD) はこの時間指令が来なければモータを止める
constexpr uint32_t kCmdTimeoutMs = 200;
// 送信側 (CWMC) の指令周期
constexpr uint32_t kCmdPeriodMs = 20;
// DCMD の状態通知周期
constexpr uint32_t kStatusPeriodMs = 100;

struct Frame {
    uint8_t command;
    int32_t value;
};

inline void encode(uint8_t command, int32_t value, uint8_t out[8]) {
    const uint32_t v = static_cast<uint32_t>(value);
    out[0] = command;
    out[1] = static_cast<uint8_t>(v >> 24);
    out[2] = static_cast<uint8_t>(v >> 16);
    out[3] = static_cast<uint8_t>(v >> 8);
    out[4] = static_cast<uint8_t>(v);
    out[5] = 0;
    out[6] = 0;
    out[7] = 0;
}

inline Frame decode(const uint8_t in[8]) {
    const uint32_t v = (static_cast<uint32_t>(in[1]) << 24) | (static_cast<uint32_t>(in[2]) << 16) |
                       (static_cast<uint32_t>(in[3]) << 8) | static_cast<uint32_t>(in[4]);
    return Frame{in[0], static_cast<int32_t>(v)};
}

}  // namespace proto
