// CWMC (中央制御基板, ESP32-DevKitC)
//   PS5 コントローラの入力を読み、DCMD ×3 と SVMD に CAN で指令を送る。
//
// 操作
//   左スティック Y : r (伸縮)        左スティック X : θ (旋回)
//   右スティック Y : z (上下)        右スティック X : エンドエフェクタ回転
//   R1 / L1        : 把持 閉じる / 開く (押している間だけ動く)
//   △             : 低速 / 高速 切り替え
//   ×             : 全軸停止 (ソフト非常停止, ラッチ)   OPTIONS : 解除
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <ps5Controller.h>

#include "can_twai.h"
#include "config.h"
#include "protocol.h"

namespace {

LiquidCrystal lcd(cfg::kLcdRs, cfg::kLcdEn, cfg::kLcdD4, cfg::kLcdD5, cfg::kLcdD6, cfg::kLcdD7);

struct DcmdState {
    uint32_t lastStatusMs = 0;
    bool seen = false;
    uint32_t flags = 0;
    bool online(uint32_t now) const { return seen && now - lastStatusMs < 3 * proto::kStatusPeriodMs + 50; }
};

bool canOk = false;
bool highSpeed = false;
bool softStop = false;
bool prevTriangle = false;
float duty[proto::kNumAxes] = {0, 0, 0};  // 実際に送る duty [%] (ランプ後)
float rotateDeg = cfg::kRotateInitDeg;
float gripDeg = cfg::kGripOpenDeg;
DcmdState dcmd[proto::kNumAxes];

// スティック値 (-128..127) をデッドゾーン付きで -1..1 に正規化
float normStick(int8_t raw) {
    int v = raw;
    if (v > -cfg::kStickDeadzone && v < cfg::kStickDeadzone) return 0.0f;
    const float span = 127.0f - cfg::kStickDeadzone;
    float x = (v > 0) ? (v - cfg::kStickDeadzone) / span : (v + cfg::kStickDeadzone) / span;
    return constrain(x, -1.0f, 1.0f);
}

float approach(float cur, float target, float maxStep) {
    if (target > cur + maxStep) return cur + maxStep;
    if (target < cur - maxStep) return cur - maxStep;
    return target;
}

bool anyHwEstop(uint32_t now) {
    for (auto& d : dcmd) {
        if (d.online(now) && (d.flags & proto::kFlagEstop)) return true;
    }
    return false;
}

int onlineCount(uint32_t now) {
    int n = 0;
    for (auto& d : dcmd) n += d.online(now) ? 1 : 0;
    return n;
}

void sendFrame(uint32_t id, uint8_t command, int32_t value) {
    if (!canOk) return;
    uint8_t buf[8];
    proto::encode(command, value, buf);
    can_twai::send(id, buf);
}

void receiveStatus(uint32_t now) {
    uint32_t id;
    uint8_t buf[8];
    while (can_twai::receive(&id, buf)) {
        if (id < proto::kDcmdStatusBaseId || id >= proto::kDcmdStatusBaseId + proto::kNumAxes) continue;
        const proto::Frame f = proto::decode(buf);
        if (f.command != proto::kDcmdStatus) continue;
        DcmdState& d = dcmd[id - proto::kDcmdStatusBaseId];
        d.flags = static_cast<uint32_t>(f.value);
        d.lastStatusMs = now;
        d.seen = true;
    }
}

void controlStep(uint32_t now, float dt) {
    const bool connected = ps5.isConnected();
    float target[proto::kNumAxes] = {0, 0, 0};
    float rotateInput = 0.0f;
    int gripInput = 0;

    if (connected) {
        const bool tri = ps5.Triangle();
        if (tri && !prevTriangle) highSpeed = !highSpeed;
        prevTriangle = tri;

        if (ps5.Cross()) softStop = true;
        if (ps5.Options()) softStop = false;

        const float maxDuty = highSpeed ? cfg::kDutyHigh : cfg::kDutyLow;
        target[proto::kAxisR] = normStick(ps5.LStickY()) * maxDuty;
        target[proto::kAxisTheta] = normStick(ps5.LStickX()) * maxDuty;
        target[proto::kAxisZ] = normStick(ps5.RStickY()) * maxDuty;

        rotateInput = normStick(ps5.RStickX()) * (highSpeed ? 1.0f : 0.5f);
        gripInput = (ps5.R1() ? 1 : 0) - (ps5.L1() ? 1 : 0);
    } else {
        prevTriangle = false;
    }

    const bool stopAll = !connected || softStop || anyHwEstop(now);

    // ---- DC モータ ----
    for (uint8_t i = 0; i < proto::kNumAxes; i++) {
        if (stopAll) {
            duty[i] = 0.0f;  // 停止はランプを通さず即座に
        } else {
            duty[i] = approach(duty[i], target[i], cfg::kDutyRampPerSec * dt);
        }
        const int32_t v = lroundf(duty[i] * cfg::kAxisDir[i] * 100.0f);
        if (stopAll) {
            sendFrame(proto::kDcmdCmdBaseId + i, proto::kDcmdStop, 0);
        } else {
            sendFrame(proto::kDcmdCmdBaseId + i, proto::kDcmdSetDuty, v);
        }
    }

    // ---- サーボ (停止中はその場で保持) ----
    if (!stopAll) {
        rotateDeg += rotateInput * cfg::kRotateSpeedDegPerSec * dt;
        rotateDeg = constrain(rotateDeg, cfg::kRotateMinDeg, cfg::kRotateMaxDeg);

        const float lo = min(cfg::kGripOpenDeg, cfg::kGripCloseDeg);
        const float hi = max(cfg::kGripOpenDeg, cfg::kGripCloseDeg);
        const float dirToClose = (cfg::kGripCloseDeg >= cfg::kGripOpenDeg) ? 1.0f : -1.0f;
        gripDeg += gripInput * dirToClose * cfg::kGripSpeedDegPerSec * dt;
        gripDeg = constrain(gripDeg, lo, hi);
    }
    sendFrame(proto::kSvmdCmdId, proto::kSvRotate, lroundf(rotateDeg * 10.0f));
    sendFrame(proto::kSvmdCmdId, proto::kSvGrip, lroundf(gripDeg * 10.0f));
}

// 16 文字ちょうどにそろえて書く (clear() を使わずちらつきを防ぐ)
void lcdLine(uint8_t row, const char* text) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", text);
    lcd.setCursor(0, row);
    lcd.print(buf);
}

void updateLcd(uint32_t now) {
    char l0[24];
    char l1[24];
    const bool connected = ps5.isConnected();

    if (!canOk) {
        snprintf(l0, sizeof(l0), "CAN INIT ERROR");
    } else if (anyHwEstop(now)) {
        snprintf(l0, sizeof(l0), "!!E-STOP (REF)!!");
    } else if (connected && softStop) {
        snprintf(l0, sizeof(l0), "STOP  OPTIONS:GO");
    } else {
        snprintf(l0, sizeof(l0), "PS5:%s %s MD%d/3", connected ? "OK" : "--", highSpeed ? "HI" : "LO",
                 onlineCount(now));
    }
    // 2 行目は 2 秒ごとに「各軸 duty[%]」と「サーボ角度[deg]」を切り替える
    if ((now / 2000) % 2 == 0) {
        snprintf(l1, sizeof(l1), "r%+03d t%+03d z%+03d", static_cast<int>(lroundf(duty[0])),
                 static_cast<int>(lroundf(duty[1])), static_cast<int>(lroundf(duty[2])));
    } else {
        snprintf(l1, sizeof(l1), "ROT %03d GRIP %03d", static_cast<int>(lroundf(rotateDeg)),
                 static_cast<int>(lroundf(gripDeg)));
    }
    lcdLine(0, l0);
    lcdLine(1, l1);
}

}  // namespace

void setup() {
    Serial.begin(115200);

    lcd.begin(16, 2);
    lcd.clear();
    lcdLine(0, "Shaffou demo");
    lcdLine(1, "Starting...");

    canOk = can_twai::begin(cfg::kCanTxPin, cfg::kCanRxPin);
    Serial.println(canOk ? "CAN ready" : "CAN init failed");

    ps5.begin(cfg::kPs5Mac);
    Serial.printf("PS5 waiting: %s\n", cfg::kPs5Mac);
}

void loop() {
    static uint32_t lastCtrlMs = millis();
    static uint32_t lastLcdMs = 0;
    static bool prevConnected = false;
    const uint32_t now = millis();

    receiveStatus(now);
    if (canOk) can_twai::service();

    const bool connected = ps5.isConnected();
    if (connected != prevConnected) {
        Serial.println(connected ? "PS5 connected" : "PS5 disconnected");
        prevConnected = connected;
    }

    if (now - lastCtrlMs >= proto::kCmdPeriodMs) {
        const float dt = (now - lastCtrlMs) / 1000.0f;
        lastCtrlMs = now;
        controlStep(now, min(dt, 0.1f));
    }

    if (now - lastLcdMs >= cfg::kLcdPeriodMs) {
        lastLcdMs = now;
        updateLcd(now);
    }

    delay(1);
}
