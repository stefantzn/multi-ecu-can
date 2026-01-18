#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/twai.h"

#include "esp_log.h"
#include "esp_err.h"

// config
#define TWAI_TX_GPIO        GPIO_NUM_21
#define TWAI_RX_GPIO        GPIO_NUM_22

#define LED_GPIO            GPIO_NUM_2
#define HEARTBEAT_PERIOD_MS 100

#define CAN_ID_HEARTBEAT    0x100
#define CAN_ID_ECU_B_ALIVE  0x101
#define CAN_ID_CMD          0x200

#define ECU_A_ALIVE_TIMEOUT_MS 1500

#define ECU_A_FLAG_FAULT (1u << 0)

static const char *TAG = "ECU_A";

static uint8_t crc8_j1850(const uint8_t *data, int len) {
    uint8_t crc = 0xFF;
    const uint8_t poly = 0x1D;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
        }
    }
    return (uint8_t)~crc;
}

static const char *twai_state_str(twai_state_t s) {

    switch (s) {
    case TWAI_STATE_STOPPED:      return "STOPPED";
    case TWAI_STATE_RUNNING:      return "RUNNING";
    case TWAI_STATE_BUS_OFF:      return "BUS_OFF";
    case TWAI_STATE_RECOVERING:   return "RECOVERING";
    default:                      return "UNKNOWN";
    }
}

static void led_init(void) {

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(LED_GPIO, 0);
}

static void twai_init_start(void) {

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);

    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 16;

    // Enable alerts for monitoring + recovery
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_ABOVE_ERR_WARN |
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_RX_QUEUE_FULL;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_LOGI(TAG, "Installing TWAI driver (TX=%d RX=%d)...", (int)TWAI_TX_GPIO, (int)TWAI_RX_GPIO);
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    ESP_LOGI(TAG, "Starting TWAI...");
    ESP_ERROR_CHECK(twai_start());
}

static esp_err_t can_send_heartbeat(uint8_t ctr, uint8_t flags) {

    twai_message_t msg = {0};
    msg.identifier = CAN_ID_HEARTBEAT;
    msg.flags = TWAI_MSG_FLAG_NONE;
    msg.data_length_code = 8;

    uint32_t up_ms = (uint32_t)esp_log_timestamp(); // ms since boot

    msg.data[0] = ctr;
    msg.data[1] = flags;

    msg.data[2] = (uint8_t)(up_ms & 0xFF);
    msg.data[3] = (uint8_t)((up_ms >> 8) & 0xFF);
    msg.data[4] = (uint8_t)((up_ms >> 16) & 0xFF);
    msg.data[5] = (uint8_t)((up_ms >> 24) & 0xFF);

    msg.data[6] = 0;
    msg.data[7] = crc8_j1850(msg.data, 7);

    return twai_transmit(&msg, pdMS_TO_TICKS(20));
}

static volatile uint32_t last_alive_tick = 0;
static volatile bool ecu_b_timeout = true;

static void heartbeat_task(void *arg) {

    (void)arg;
    uint8_t hb = 0;
    int led = 0;
    uint8_t flags = 0;

    while (1) {

        esp_err_t err = can_send_heartbeat(hb++, flags);

        if (err == ESP_OK) {

            led ^= 1;
            gpio_set_level(LED_GPIO, led);
        } 
        else {

            ESP_LOGW(TAG, "Heartbeat TX failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS));
    }
}

static void rx_task(void *arg) {

    (void)arg;
    twai_message_t rx_msg;

    while (1) {

        esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(200));

        if (err == ESP_ERR_TIMEOUT) continue;

        if (err != ESP_OK) {

            ESP_LOGW(TAG, "RX error: %s", esp_err_to_name(err));
            continue;
        }

        if (rx_msg.extd) continue; // ignore extended for this demo

        // ECU_B alive handling (CRC + timeout tracking)
        if (rx_msg.identifier == CAN_ID_ECU_B_ALIVE && rx_msg.data_length_code == 8) {

            uint8_t rx_crc = rx_msg.data[7];
            uint8_t calc   = crc8_j1850(rx_msg.data, 7);

            if (rx_crc != calc) {

                ESP_LOGW(TAG, "ECU_B ALIVE CRC FAIL: rx=%02X calc=%02X (ctr=%u)", rx_crc, calc, rx_msg.data[0]);
            } 
            else {

                last_alive_tick = xTaskGetTickCount();
                ecu_b_timeout = false;
                ESP_LOGI(TAG, "ECU_B alive OK (ctr=%u flags=0x%02X state=%u)", rx_msg.data[0], rx_msg.data[1], rx_msg.data[2]);
            }
            continue;
        }

        // Command handling (LED)
        if (rx_msg.identifier == CAN_ID_CMD && rx_msg.data_length_code >= 1) {

            uint8_t cmd = rx_msg.data[0];

            if (cmd == 0x01) {

                gpio_set_level(LED_GPIO, 1);
                ESP_LOGI(TAG, "CMD: LED ON");
            } 
            else if (cmd == 0x00) {

                gpio_set_level(LED_GPIO, 0);
                ESP_LOGI(TAG, "CMD: LED OFF");
            } 
            else {

                ESP_LOGI(TAG, "CMD: unknown 0x%02X", cmd);
            }
        } 
        else {

            ESP_LOGI(TAG, "RX id=0x%03X dlc=%d", (unsigned)rx_msg.identifier, (int)rx_msg.data_length_code);
        }
    }
}

static void alert_task(void *arg) {

    (void)arg;

    while (1) {

        uint32_t alerts = 0;
        esp_err_t err = twai_read_alerts(&alerts, pdMS_TO_TICKS(1000));

        if (err == ESP_ERR_TIMEOUT) continue;

        if (err != ESP_OK) continue;

        if (alerts & TWAI_ALERT_BUS_OFF) {

            ESP_LOGE(TAG, "TWAI BUS OFF -> initiating recovery");
            (void)twai_initiate_recovery();
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {

            ESP_LOGW(TAG, "TWAI bus recovered");
        }
        if (alerts & TWAI_ALERT_ERR_PASS) {

            ESP_LOGW(TAG, "TWAI error passive");
        }
        if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {

            ESP_LOGW(TAG, "TWAI above error warning");
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {

            ESP_LOGW(TAG, "TWAI RX queue full (drops likely)");
        }
        if (alerts & TWAI_ALERT_TX_FAILED) {

            ESP_LOGW(TAG, "TWAI TX failed");
        }
    }
}

static void health_task(void *arg) {

    (void)arg;
    while (1) {
        
        // ECU_B alive timeout check
        uint32_t now = xTaskGetTickCount();
        uint32_t age_ms = (uint32_t)(now - last_alive_tick) * (uint32_t)portTICK_PERIOD_MS;

        if (!ecu_b_timeout && age_ms > ECU_A_ALIVE_TIMEOUT_MS) {

            ecu_b_timeout = true;
            ESP_LOGW(TAG, "ECU_B ALIVE TIMEOUT (%lu ms)", (unsigned long)age_ms);
        }

        // TWAI status
        twai_status_info_t st;

        if (twai_get_status_info(&st) == ESP_OK) {

            ESP_LOGI(TAG,
                     "TWAI: %s (state=%d) TEC=%d REC=%d bus_err=%lu rx_missed=%lu tx_failed=%lu",
                     twai_state_str(st.state),
                     (int)st.state,
                     (int)st.tx_error_counter,
                     (int)st.rx_error_counter,
                     (unsigned long)st.bus_error_count,
                     (unsigned long)st.rx_missed_count,
                     (unsigned long)st.tx_failed_count);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {

    ESP_LOGI(TAG, "ECU A booting...");

    led_init();
    twai_init_start();

    xTaskCreatePinnedToCore(heartbeat_task, "hb",    4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(rx_task,        "rx",    4096, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(alert_task,     "alert", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(health_task,    "hlth",  4096, NULL, 3, NULL, 1);

    ESP_LOGI(TAG, "ECU A ready. Heartbeat TX=0x%03X, CMD RX=0x%03X, ECU_B alive RX=0x%03X", CAN_ID_HEARTBEAT, CAN_ID_CMD, CAN_ID_ECU_B_ALIVE);
}
