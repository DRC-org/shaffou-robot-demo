// ESP32 内蔵 CAN コントローラ (TWAI) の薄いラッパ
#pragma once

#include <stdint.h>

namespace can_twai {

bool begin(int txPin, int rxPin);
// 送信キューが一杯のときは false (待たない)
bool send(uint32_t id, const uint8_t data[8]);
// 受信があれば true。待たない
bool receive(uint32_t* id, uint8_t data[8]);
// バスオフからの復帰などを行う。loop から定期的に呼ぶ
void service();

}  // namespace can_twai
