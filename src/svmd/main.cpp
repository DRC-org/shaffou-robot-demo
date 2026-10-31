// SVMD (サーボドライバ基板, UNO R4 Minima ベース)
//   CWMC からの角度指令でサーボ 2 個を動かす。
//     ch0 (SV0, D3) : エンドエフェクタ回転
//     ch1 (SV1, D6) : 把持
//   - 電源投入直後は信号を出さず、最初の指令を受けたチャンネルから動かし始める (急な動きを防ぐ)
//   - 指令が途絶えてもその角度を保持する (物を落とさないため)
#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Servo.h>

#include "protocol.h"

namespace {

constexpr uint8_t kNumServos = 2;
constexpr uint8_t kServoPins[kNumServos] = {3, 6};  // SV0, SV1
// 0°/180° に対応するパルス幅 [us]。サーボの仕様に合わせて調整する
constexpr int kPulseMinUs[kNumServos] = {500, 500};
constexpr int kPulseMaxUs[kNumServos] = {2400, 2400};
constexpr int32_t kMinDeci = 0;     // 0.0°
constexpr int32_t kMaxDeci = 1800;  // 180.0°
constexpr uint8_t kLedPin = 13;

Servo servos[kNumServos];
bool attached[kNumServos] = {false, false};
int32_t angleDeci[kNumServos] = {0, 0};
int32_t lastWrittenUs[kNumServos] = {-1, -1};
uint32_t lastCmdMs = 0;

int32_t toMicros(uint8_t ch, int32_t deci) {
    return kPulseMinUs[ch] + (static_cast<int32_t>(kPulseMaxUs[ch] - kPulseMinUs[ch]) * deci) / kMaxDeci;
}

void apply(uint8_t ch) {
    if (!attached[ch]) {
        servos[ch].attach(kServoPins[ch], kPulseMinUs[ch], kPulseMaxUs[ch]);
        attached[ch] = true;
    }
    const int32_t us = toMicros(ch, angleDeci[ch]);
    if (us != lastWrittenUs[ch]) {
        servos[ch].writeMicroseconds(us);
        lastWrittenUs[ch] = us;
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    pinMode(kLedPin, OUTPUT);

    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("CAN.begin failed");
        while (true) {
            digitalWrite(kLedPin, !digitalRead(kLedPin));
            delay(100);
        }
    }
    Serial.println("SVMD ready");
}

void loop() {
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();

    while (CAN.available()) {
        const CanMsg msg = CAN.read();
        if (!msg.isStandardId() || msg.getStandardId() != proto::kSvmdCmdId || msg.data_length != 8) continue;
        const proto::Frame f = proto::decode(msg.data);
        if (f.command >= kNumServos) continue;

        angleDeci[f.command] = constrain(f.value, kMinDeci, kMaxDeci);
        apply(f.command);
        lastCmdMs = now;
    }

    // 指令が来ている間は LED 点灯
    digitalWrite(kLedPin, (lastCmdMs != 0 && now - lastCmdMs < proto::kCmdTimeoutMs) ? HIGH : LOW);

    if (now - lastLogMs >= 500) {
        lastLogMs = now;
        Serial.print("rot=");
        Serial.print(angleDeci[0] / 10.0f, 1);
        Serial.print(" grip=");
        Serial.println(angleDeci[1] / 10.0f, 1);
    }
}
