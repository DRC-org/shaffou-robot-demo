#include "can_twai.h"

#include <Arduino.h>
#include <driver/twai.h>
#include <string.h>

namespace can_twai {

bool begin(int txPin, int rxPin) {
    twai_general_config_t g =
        TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(txPin), static_cast<gpio_num_t>(rxPin), TWAI_MODE_NORMAL);
    g.tx_queue_len = 16;
    g.rx_queue_len = 32;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    return twai_start() == ESP_OK;
}

bool send(uint32_t id, const uint8_t data[8]) {
    twai_message_t msg = {};
    msg.identifier = id;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, 0) == ESP_OK;
}

bool receive(uint32_t* id, uint8_t data[8]) {
    twai_message_t msg;
    if (twai_receive(&msg, 0) != ESP_OK) return false;
    if (msg.extd || msg.rtr || msg.data_length_code != 8) return false;
    *id = msg.identifier;
    memcpy(data, msg.data, 8);
    return true;
}

void service() {
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK) return;
    if (s.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
    } else if (s.state == TWAI_STATE_STOPPED) {
        twai_start();  // 復帰完了後は停止状態になるので再開する
    }
}

}  // namespace can_twai
