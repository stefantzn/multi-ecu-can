#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/twai.h"

// configuration
#define TWAI_TX_GPIO GPIO_NUM_21
#define TWAI_RX_GPIO GPIO_NUM_22

                                                                                                                                                                                                                                                                                                                                                                                                                                    
#define CAN_ID_ECU_A_HEARTBEAT 0x100
#define CAN_ID_ECU_B_ALIVE     0x101

#define ECU_A_HB_TIMEOUT_MS    300

#define ECU_B_STATE_OK         0
#define ECU_B_STATE_DEGRADED   1
#define ECU_B_STATE_SAFE       2

#define ECU_B_FLAG_FAULT       (1u << 0)
#define ECU_B_FLAG_A_TIMEOUT   (1u << 1)

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static const char *TAG = "ECU_B";

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

static volatile uint32_t last_hb_tick = 0;
static volatile bool ecu_a_timeout = true;   // true until first good heartbeat
static volatile bool crc_fail_seen = false;

static void twai_startup_or_die(void) {
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_GPIO, TWAI_RX_GPIO, TWAI_MODE_NORMAL);

    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 16;

    // Alerts for bus/fault monitoring
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF |
                              TWAI_ALERT_BUS_RECOVERED |
                              TWAI_ALERT_ERR_PASS |
                              TWAI_ALERT_ABOVE_ERR_WARN |
                              TWAI_ALERT_TX_FAILED |
                              TWAI_ALERT_RX_QUEUE_FULL;

    ESP_LOGI(TAG, "Installing TWAI driver...");
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));

    ESP_LOGI(TAG, "Starting TWAI...");
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG, "TWAI started (NORMAL mode, will ACK frames).");
}

static void rx_task(void *arg) {

    (void)arg;
    twai_message_t msg;

    while (1) {

        esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(1000));

        if (err == ESP_OK) {

            if (!msg.extd) {

                ESP_LOGI(TAG,
                         "RX ID=0x%03X DLC=%d data=%02X %02X %02X %02X %02X %02X %02X %02X",
                         msg.identifier, msg.data_length_code,
                         msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                         msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

                if (msg.identifier == CAN_ID_ECU_A_HEARTBEAT && msg.data_length_code == 8) {

                    uint8_t rx_crc = msg.data[7];
                    uint8_t calc   = crc8_j1850(msg.data, 7);

                    if (rx_crc != calc) {

                        crc_fail_seen = true;
                        ESP_LOGW(TAG, "HB CRC FAIL: rx=%02X calc=%02X (ctr=%u)", rx_crc, calc, msg.data[0]);
                    } 
                    else {

                        last_hb_tick = xTaskGetTickCount();
                        ecu_a_timeout = false;

                        uint32_t up_ms = (uint32_t)msg.data[2] |
                                         ((uint32_t)msg.data[3] << 8) |
                                         ((uint32_t)msg.data[4] << 16) |
                                         ((uint32_t)msg.data[5] << 24);

                        ESP_LOGI(TAG, "ECU_A heartbeat OK (ctr=%u uptime_ms=%lu)", msg.data[0], (unsigned long)up_ms);
                    }
                }
            } 
            else {
                ESP_LOGI(TAG, "RX EXT ID=0x%08lX DLC=%d", (unsigned long)msg.identifier, msg.data_length_code);
            }
        }
        // timeout is normal, do nothing
    }
}

static void tx_task(void *arg) {

    (void)arg;
    uint8_t ctr = 0;

    while (1) {

        // timeout check
        uint32_t now = xTaskGetTickCount();
        uint32_t age_ms = (uint32_t)(now - last_hb_tick) * (uint32_t)portTICK_PERIOD_MS;

        if (!ecu_a_timeout && age_ms > ECU_A_HB_TIMEOUT_MS) {

            ecu_a_timeout = true;
            ESP_LOGW(TAG, "ECU_A heartbeat TIMEOUT (%lu ms)", (unsigned long)age_ms);
        }

        uint8_t flags = 0;
        uint8_t state = ECU_B_STATE_OK;

        if (crc_fail_seen) flags |= ECU_B_FLAG_FAULT;

        if (ecu_a_timeout) {

            flags |= ECU_B_FLAG_A_TIMEOUT;
            state = ECU_B_STATE_DEGRADED;   // change to ECU_B_STATE_SAFE if you want hard fail-safe
        }

        twai_message_t tx = {0};
        tx.identifier = CAN_ID_ECU_B_ALIVE;
        tx.extd = 0;
        tx.rtr = 0;
        tx.data_length_code = 8;

        tx.data[0] = ctr++;
        tx.data[1] = flags;
        tx.data[2] = state;
        tx.data[3] = 0;
        tx.data[4] = 0;
        tx.data[5] = 0;
        tx.data[6] = 0;
        tx.data[7] = crc8_j1850(tx.data, 7);

        esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(50));

        if (err == ESP_OK) {

            ESP_LOGI(TAG, "TX alive (ctr=%u flags=0x%02X state=%u)", tx.data[0], tx.data[1], tx.data[2]);
        } 
        else {

            ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(500));
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

static void status_task(void *arg) {

    (void)arg;
    while (1) {

        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {

            ESP_LOGI(TAG, "State=%d TEC=%d REC=%d bus_err=%lu rx_missed=%lu tx_failed=%lu",
                     st.state, st.tx_error_counter, st.rx_error_counter,
                     (unsigned long)st.bus_error_count,
                     (unsigned long)st.rx_missed_count,
                     (unsigned long)st.tx_failed_count);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void) {
    
    twai_startup_or_die();

    xTaskCreate(rx_task,     "rx_task",     4096, NULL, 10, NULL);
    xTaskCreate(tx_task,     "tx_task",     4096, NULL,  9, NULL);
    xTaskCreate(alert_task,  "alert_task",  4096, NULL,  8, NULL);
    xTaskCreate(status_task, "status_task", 4096, NULL,  5, NULL);
}
