// Fake ESP-IDF TWAI API: records transmitted frames, serves queued received frames (unit tests only).
#pragma once
#include <cstdint>
#include <cstring>
#include <deque>
#include <vector>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
typedef int gpio_num_t;
typedef enum { TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY } twai_mode_t;
typedef struct { uint32_t flags; uint32_t identifier; uint8_t data_length_code; uint8_t data[8]; } twai_message_t;
typedef struct { twai_mode_t mode; int tx_io, rx_io; uint32_t tx_queue_len, rx_queue_len; } twai_general_config_t;
typedef struct { int brp; } twai_timing_config_t;
typedef struct { uint32_t acceptance_code, acceptance_mask; bool single_filter; } twai_filter_config_t;
#define TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, mode) {mode, tx, rx, 5, 5}
#define TWAI_TIMING_CONFIG_1MBITS() {4}
#define TWAI_FILTER_CONFIG_ACCEPT_ALL() {0, 0xFFFFFFFF, true}
typedef enum { TWAI_STATE_STOPPED, TWAI_STATE_RUNNING, TWAI_STATE_BUS_OFF, TWAI_STATE_RECOVERING } twai_state_t;
typedef struct { twai_state_t state; uint32_t msgs_to_tx, msgs_to_rx, tx_error_counter, rx_error_counter, tx_failed_count,
                 rx_missed_count, rx_overrun_count, arb_lost_count, bus_error_count; } twai_status_info_t;
namespace fake {
inline std::vector<twai_message_t> tx;
inline std::deque<twai_message_t> rx;
inline bool installed = false, started = false;
inline int tx_queue_free = 1000;
inline twai_state_t state = TWAI_STATE_STOPPED;
inline int recover_polls = 0;          // status reads until a requested recovery completes
inline int recoveries = 0, starts = 0;
}
inline esp_err_t twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*, const twai_filter_config_t*) { fake::installed = true; return ESP_OK; }
inline esp_err_t twai_start() {
  if (!fake::installed || fake::state == TWAI_STATE_BUS_OFF || fake::state == TWAI_STATE_RECOVERING) return ESP_FAIL;
  fake::started = true; fake::state = TWAI_STATE_RUNNING; ++fake::starts; return ESP_OK; }
inline esp_err_t twai_initiate_recovery() {
  if (fake::state != TWAI_STATE_BUS_OFF) return ESP_FAIL;
  fake::state = TWAI_STATE_RECOVERING; fake::recover_polls = 2; ++fake::recoveries; return ESP_OK; }
inline esp_err_t twai_get_status_info(twai_status_info_t* s) {
  if (fake::state == TWAI_STATE_RECOVERING && --fake::recover_polls <= 0) fake::state = TWAI_STATE_STOPPED;
  *s = {}; s->state = fake::state; return ESP_OK; }
inline esp_err_t twai_transmit(const twai_message_t* m, uint32_t) {
  if (fake::state != TWAI_STATE_RUNNING || fake::tx_queue_free <= 0) return ESP_FAIL;
  fake::tx.push_back(*m); return ESP_OK;
}
inline esp_err_t twai_receive(twai_message_t* m, uint32_t) {
  if (fake::rx.empty()) return ESP_FAIL;
  *m = fake::rx.front(); fake::rx.pop_front(); return ESP_OK;
}
