// DCMD (DC モータドライバ基板, UNO R4 Minima ベース)
//   CWMC からの duty 指令でモータを回す。3 枚とも同じファームで、
//   DIP スイッチの値で軸を決める (0:r 1:θ 2:z -> CAN ID 0x100 + n)。
//
// 安全機能
//   - 指令が kCmdTimeoutMs 途絶えたら停止
//   - REF (非常停止) 作動中は停止。解除後は duty 0 の指令を一度受けるまで動かさない
//   - duty の上限 kMaxDutyHard
//
// RGB LED: 緑=運転可  青=指令待ち  赤=非常停止  黄=非常停止解除後の待機  赤点滅=CAN 初期化失敗
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Arduino_CAN.h>
#include <pwm.h>

#include "protocol.h"

namespace {

// ---- ピン ----
constexpr uint8_t kPwmN = 11;  // 正転側
constexpr uint8_t kPwmL = 10;  // 逆転側
constexpr uint8_t kLedPin = 13;
constexpr uint8_t kRgbPin = 6;
constexpr uint8_t kRefPin = 12;
constexpr uint8_t kDipPins[4] = {8, 9, 1, 0};  // SW0..SW3 (ON で LOW)

// ---- 設定 ----
constexpr float kPwmFreqHz = 30000.0f;
constexpr float kMaxDutyHard = 80.0f;  // CWMC 側の設定に関係なくこれ以上は出さない [%]
// REF ピンがこのレベルのとき「非常停止中」とみなす
// ※要確認: 非常停止ボタンを押してシリアルの REF= の値で確かめ、逆なら HIGH にする
constexpr int kRefEstopLevel = LOW;

PwmOut pwmN(kPwmN);
PwmOut pwmL(kPwmL);
Adafruit_NeoPixel rgb(1, kRgbPin, NEO_GRB + NEO_KHZ800);

uint8_t axis = 0;
uint32_t cmdId = 0;
uint32_t statusId = 0;

float commandDuty = 0.0f;   // 最後に受けた指令 [%]
float appliedDuty = 0.0f;   // 実際に出している duty [%]
uint32_t lastCmdMs = 0;
bool everReceived = false;
bool waitRearm = false;     // 非常停止解除後、duty 0 指令待ち
uint32_t lastRgb = 0xFFFFFFFF;

void motorDrive(float d) {
    static float last = NAN;
    d = constrain(d, -kMaxDutyHard, kMaxDutyHard);
    if (d == last) return;  // 変化したときだけ PWM を設定し直す
    last = d;
    if (d > 0) {
        pwmL.pulse_perc(0);
        pwmN.pulse_perc(d);
    } else if (d < 0) {
        pwmN.pulse_perc(0);
        pwmL.pulse_perc(-d);
    } else {
        pwmN.pulse_perc(0);
        pwmL.pulse_perc(0);
    }
}

void setRgb(uint8_t r, uint8_t g, uint8_t b) {
    const uint32_t c = rgb.Color(r, g, b);
    if (c == lastRgb) return;  // 毎ループ show() すると割り込みが止まるので変化時だけ
    lastRgb = c;
    rgb.setPixelColor(0, c);
    rgb.show();
}

uint8_t readDip() {
    uint8_t n = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (digitalRead(kDipPins[i]) == LOW) n |= (1 << i);
    }
    return n;
}

bool estopActive() { return digitalRead(kRefPin) == kRefEstopLevel; }

void receiveCan(uint32_t now) {
    while (CAN.available()) {
        const CanMsg msg = CAN.read();
        if (!msg.isStandardId() || msg.getStandardId() != cmdId || msg.data_length != 8) continue;
        const proto::Frame f = proto::decode(msg.data);
        switch (f.command) {
            case proto::kDcmdSetDuty:
                commandDuty = f.value / 100.0f;
                break;
            case proto::kDcmdStop:
                commandDuty = 0.0f;
                break;
            default:
                continue;
        }
        lastCmdMs = now;
        everReceived = true;
        digitalWrite(kLedPin, !digitalRead(kLedPin));
    }
}

uint32_t statusFlags(bool estop, bool timeout) {
    uint32_t f = 0;
    if (estop) f |= proto::kFlagEstop;
    if (timeout) f |= proto::kFlagCmdTimeout;
    if (waitRearm) f |= proto::kFlagWaitRearm;
    return f;
}

void sendStatus(uint32_t flags) {
    uint8_t buf[8];
    proto::encode(proto::kDcmdStatus, static_cast<int32_t>(flags), buf);
    CAN.write(CanMsg(CanStandardId(statusId), 8, buf));
}

}  // namespace

void setup() {
    Serial.begin(115200);

    // 何よりも先にモータ出力を止める
    pinMode(kPwmN, OUTPUT);
    pinMode(kPwmL, OUTPUT);
    digitalWrite(kPwmN, LOW);
    digitalWrite(kPwmL, LOW);
    pwmN.begin(kPwmFreqHz, 0.0f);
    pwmL.begin(kPwmFreqHz, 0.0f);

    pinMode(kLedPin, OUTPUT);
    pinMode(kRefPin, INPUT);
    for (uint8_t p : kDipPins) pinMode(p, INPUT_PULLUP);

    rgb.begin();
    rgb.setBrightness(30);

    delay(10);
    axis = readDip();
    cmdId = proto::kDcmdCmdBaseId + axis;
    statusId = proto::kDcmdStatusBaseId + axis;

    if (!CAN.begin(CanBitRate::BR_1000k)) {
        Serial.println("CAN.begin failed");
        while (true) {
            setRgb(255, 0, 0);
            delay(100);
            setRgb(0, 0, 0);
            delay(100);
        }
    }

    // 起動直後に REF が作動していれば、解除後は再アーム待ちから始める
    waitRearm = estopActive();
    Serial.print("DCMD axis=");
    Serial.print(axis);
    Serial.print(" cmdId=0x");
    Serial.println(cmdId, HEX);
}

void loop() {
    static uint32_t lastStatusMs = 0;
    static uint32_t lastLogMs = 0;
    const uint32_t now = millis();

    receiveCan(now);

    const bool estop = estopActive();
    const bool timeout = !everReceived || (now - lastCmdMs > proto::kCmdTimeoutMs);

    if (estop) {
        waitRearm = true;
    } else if (waitRearm && !timeout && commandDuty == 0.0f) {
        waitRearm = false;
    }

    if (estop || timeout || waitRearm) {
        appliedDuty = 0.0f;
    } else {
        appliedDuty = commandDuty;
    }
    motorDrive(appliedDuty);

    if (estop) {
        setRgb(255, 0, 0);
    } else if (waitRearm) {
        setRgb(255, 160, 0);
    } else if (timeout) {
        setRgb(0, 0, 255);
    } else {
        setRgb(0, 255, 0);
    }

    if (now - lastStatusMs >= proto::kStatusPeriodMs) {
        lastStatusMs = now;
        sendStatus(statusFlags(estop, timeout));
    }

    if (now - lastLogMs >= 500) {
        lastLogMs = now;
        Serial.print("axis=");
        Serial.print(axis);
        Serial.print(" duty=");
        Serial.print(appliedDuty);
        Serial.print(" cmd=");
        Serial.print(commandDuty);
        Serial.print(" REF=");
        Serial.print(digitalRead(kRefPin));
        Serial.print(estop ? " ESTOP" : "");
        Serial.print(waitRearm ? " WAIT_REARM" : "");
        Serial.println(timeout ? " TIMEOUT" : "");
    }
}
