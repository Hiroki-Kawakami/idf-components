#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLEXISPOT_HEIGHT_MIN (625)
#define FLEXISPOT_HEIGHT_MAX (1280)

typedef enum __attribute__((packed)) {
    FLEXISPOT_BUTTON_NONE    = 0,
    FLEXISPOT_BUTTON_UP      = 1 << 0,
    FLEXISPOT_BUTTON_DOWN    = 1 << 1,
    FLEXISPOT_BUTTON_MEMORY  = 1 << 2,
    FLEXISPOT_BUTTON_PRESET1 = 1 << 3,
    FLEXISPOT_BUTTON_PRESET2 = 1 << 4,
    FLEXISPOT_BUTTON_PRESET3 = 1 << 5,
    FLEXISPOT_BUTTON_PRESET4 = 1 << 6,
} flexispot_button_t;

typedef struct {
    enum __attribute__((packed)) flexispot_display_type {
        FLEXISPOT_DISPLAY_NONE,
        FLEXISPOT_DISPLAY_HEIGHT,
        FLEXISPOT_DISPLAY_LOCK,
        FLEXISPOT_DISPLAY_SELECT,
        FLEXISPOT_DISPLAY_SENSITIVITY,
        FLEXISPOT_DISPLAY_RESET,
        FLEXISPOT_DISPLAY_UNKNOWN,
    } type;
    int16_t value;
} __attribute__((packed)) flexispot_display_info_t;

typedef enum __attribute__((packed)) {
    FLEXISPOT_STATE_STOP,
    FLEXISPOT_STATE_ASCENDING,
    FLEXISPOT_STATE_DESCENDING,
} flexispot_state_t;

typedef void (*flexispot_notify_callback_t)(flexispot_display_info_t info, bool changed);

void flexispot_set_button_state(flexispot_button_t buttons);
uint32_t flexispot_get_display_data(void);
flexispot_display_info_t flexispot_get_display_info(void);
uint16_t flexispot_get_height(void);
void flexispot_register_notify_callback(flexispot_notify_callback_t callback);
void flexispot_turn_on_display(void);
void flexispot_set_height_auto(int height);
void flexispot_stop_auto_adjust(void);
flexispot_state_t flexispot_get_state(void);
esp_err_t flexispot_init(void);

#ifdef __cplusplus
}
#endif
