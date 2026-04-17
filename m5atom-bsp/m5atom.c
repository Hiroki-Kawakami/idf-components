#include "m5atom.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "led_strip.h"

static const char *TAG = "m5atom";

#if CONFIG_IDF_TARGET_ESP32
// ATOM Lite, ATOM U
#define BUTTON_PIN GPIO_NUM_39
#define LED_PIN    GPIO_NUM_27
#elif CONFIG_IDF_TARGET_ESP32S3
// AtomS3 Lite, AtomS3U
#define BUTTON_PIN GPIO_NUM_41
#define LED_PIN    GPIO_NUM_35
#endif

static led_strip_handle_t led_strip = NULL;
static button_handle_t gpio_btn = NULL;

esp_err_t m5atom_init(void) {
    esp_err_t ret;

    // Setup LED
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 0,
        .flags.with_dma = false,
    };
    ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear LED on init
    m5atom_led_clear();

    // Setup Button
    button_config_t btn_cfg = {
        .long_press_time = 1000,
        .short_press_time = 50,
    };
    button_gpio_config_t btn_gpio_cfg = {
        .gpio_num = BUTTON_PIN,
        .active_level = 0,
    };
    ret = iot_button_new_gpio_device(&btn_cfg, &btn_gpio_cfg, &gpio_btn);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create button: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "M5Atom initialized successfully");
    return ESP_OK;
}

esp_err_t m5atom_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    ret = led_strip_set_pixel(led_strip, 0, r, g, b);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = led_strip_refresh(led_strip);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t m5atom_led_clear(void) {
    return m5atom_led_set_color(0, 0, 0);
}

esp_err_t m5atom_button_register_cb(m5atom_button_event_t event, m5atom_button_cb_t cb, void *user_data) {
    if (gpio_btn == NULL) {
        ESP_LOGE(TAG, "Button not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    button_event_t iot_event;
    switch (event) {
        case M5ATOM_BUTTON_SINGLE_CLICK:
            iot_event = BUTTON_SINGLE_CLICK;
            break;
        case M5ATOM_BUTTON_LONG_PRESS_START:
            iot_event = BUTTON_LONG_PRESS_START;
            break;
        case M5ATOM_BUTTON_LONG_PRESS_HOLD:
            iot_event = BUTTON_LONG_PRESS_HOLD;
            break;
        default:
            ESP_LOGE(TAG, "Unknown button event: %d", event);
            return ESP_ERR_INVALID_ARG;
    }

    return iot_button_register_cb(gpio_btn, iot_event, NULL, (button_cb_t)cb, user_data);
}
