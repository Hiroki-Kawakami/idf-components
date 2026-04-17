#pragma once
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Button event callback function type
 *
 * @param handle Button handle (reserved for future use)
 * @param user_data User data passed during callback registration
 */
typedef void (*m5atom_button_cb_t)(void *handle, void *user_data);

/**
 * @brief Button event types
 */
typedef enum {
    M5ATOM_BUTTON_SINGLE_CLICK,
    M5ATOM_BUTTON_LONG_PRESS_START,
    M5ATOM_BUTTON_LONG_PRESS_HOLD,
} m5atom_button_event_t;

/**
 * @brief Initialize M5Atom BSP (LED and Button)
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t m5atom_init(void);

/**
 * @brief Set LED color (RGB)
 *
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t m5atom_led_set_color(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Turn off LED
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t m5atom_led_clear(void);

/**
 * @brief Register button event callback
 *
 * @param event Button event type
 * @param cb Callback function
 * @param user_data User data to pass to callback
 * @return esp_err_t ESP_OK on success
 */
esp_err_t m5atom_button_register_cb(m5atom_button_event_t event, m5atom_button_cb_t cb, void *user_data);

#ifdef __cplusplus
}
#endif
