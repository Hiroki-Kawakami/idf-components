#include "flexispot.h"
#include "flexispot_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "flexispot";

flexispot_button_t flexispot_button_state = FLEXISPOT_BUTTON_NONE;

void flexispot_set_button_state(flexispot_button_t buttons) {
    flexispot_button_state = buttons;
}

static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}
static void flexispot_send_packet(uint8_t type, const uint8_t *payload, size_t size) {
    uint8_t packet[32] = { 0x9b, size + 4, type };
    if (size) memcpy(&packet[3], payload, size);
    uint16_t checksum = crc16_modbus(&packet[1], size + 2);
    packet[3 + size] = checksum >> 8;    // CRC high byte first (big endian)
    packet[4 + size] = checksum & 0xFF;  // CRC low byte
    packet[5 + size] = 0x9d;

    // char hex_str[128] = {0};
    // for (int i = 0; i < 6 + size; i++) {
    //     sprintf(hex_str + i * 3, "%02x ", packet[i]);
    // }
    // ESP_LOGI(TAG, "[TX] %s", hex_str);

    int len = uart_write_bytes(FLEXISPOT_UART_NUM, packet, 6 + size);
    if (len != 6 + size) ESP_LOGE(TAG, "Failed to send packet");
}
static void send_button_state(flexispot_button_t buttons) {
    uint16_t raw_value = 0;
    struct { flexispot_button_t button; uint16_t raw_value; } tbl[] = {
        { FLEXISPOT_BUTTON_UP     , 0x0001 },
        { FLEXISPOT_BUTTON_DOWN   , 0x0002 },
        { FLEXISPOT_BUTTON_MEMORY , 0x0020 },
        { FLEXISPOT_BUTTON_PRESET1, 0x0004 },
        { FLEXISPOT_BUTTON_PRESET2, 0x0008 },
        { FLEXISPOT_BUTTON_PRESET3, 0x0010 },
        { FLEXISPOT_BUTTON_PRESET4, 0x0100 },
    };
    for (int i = 0; i < (sizeof(tbl)/sizeof(tbl[0])); i++) {
        if (buttons & tbl[i].button) raw_value |= tbl[i].raw_value;
    }
    // ESP_LOGI(TAG, "Sending button state [0x%04d]", raw_value);
    flexispot_send_packet(0x02, (uint8_t[]){ raw_value & 0xFF, raw_value >> 8 }, 2);
}

// MARK: Display
static struct {
    uint32_t display;
    int height;
} current_state;
static flexispot_notify_callback_t notify_callback;

static int decode_number(uint8_t segment) {
    const uint8_t nums[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };
    for (int i = 0; i < sizeof(nums); i++) {
        if (nums[i] == segment) return i;
    }
    return -1;
}
static int decode_height(uint32_t display) {
    int height = 0, multiplier = 1;
    for (int i = 0; i < 3; i++) {
        int num = decode_number((display >> (8 * i)) & 0x7f);
        if (num < 0) return 0;
        height += num * multiplier;
        multiplier *= 10;
    }
    if (!(display & 0x008000)) height *= 10;
    return height;
}
static flexispot_display_info_t decode_display_info(uint32_t display) {
    if (!display) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_NONE, 0 };
    int height = decode_height(display);
    if (height) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_HEIGHT, height };
    if (display == 0x385c39) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_LOCK, 0 };
    if ((display & 0xffff00) == 0x6d4000) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_SELECT, decode_number(display & 0xff) };
    if ((display & 0xffff00) == 0x774000) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_SENSITIVITY, decode_number(display & 0xff) };
    if (display == 0x776d31) return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_RESET, 0 };
    return (flexispot_display_info_t){ FLEXISPOT_DISPLAY_UNKNOWN, 0 };
}
static void log_display_info(flexispot_display_info_t info) {
    switch (info.type) {
    case FLEXISPOT_DISPLAY_NONE:
        ESP_LOGI(TAG, "Display: None");
        break;
    case FLEXISPOT_DISPLAY_HEIGHT:
        if (info.value < 1000) ESP_LOGI(TAG, "Display: [%d.%dcm]", info.value / 10, info.value % 10);
        else ESP_LOGI(TAG, "Display: [%dcm]", info.value / 10);
        break;
    case FLEXISPOT_DISPLAY_LOCK:
        ESP_LOGI(TAG, "Display: [Lock]");
        break;
    case FLEXISPOT_DISPLAY_SELECT:
        if (info.value >= 0) ESP_LOGI(TAG, "Display: [Select-%d]", info.value);
        else ESP_LOGI(TAG, "Display: [Select- ]");
        break;
    case FLEXISPOT_DISPLAY_SENSITIVITY:
        if (info.value >= 0) ESP_LOGI(TAG, "Display: [Sensitivity-%d]", info.value);
        else ESP_LOGI(TAG, "Display: [Sensitivity- ]");
        break;
    case FLEXISPOT_DISPLAY_RESET:
        ESP_LOGI(TAG, "Display: [Reset]");
        break;
    default:
        ESP_LOGI(TAG, "Display: Unknown");
        break;
    }
}

static void display_packet_received(uint8_t *data, int size) {
    if (size != 7) return;
    uint32_t display = 0;
    for (int i = 1; i <= 3; i++) display = (display << 8) | data[i];
    if (current_state.display == display) return;

    ESP_LOGI(TAG, "Display Raw: %06X", display);
    flexispot_display_info_t info = decode_display_info(display);
    log_display_info(info);
    current_state.display = display;
    if (info.type == FLEXISPOT_DISPLAY_HEIGHT) current_state.height = info.value;
    if (notify_callback) notify_callback(info);
}

uint32_t flexispot_get_display_data(void) {
    return current_state.display;
}
flexispot_display_info_t flexispot_get_display_info(void) {
    return decode_display_info(current_state.display);
}
uint16_t flexispot_get_height(void) {
    return current_state.height;
}

void flexispot_register_notify_callback(flexispot_notify_callback_t callback) {
    notify_callback = callback;
}

// MARK: RX Task
#define UART_QUEUE_SIZE 8
#define UART_BUFFER_SIZE 256
static QueueHandle_t uart_queue;
static int rx_offset = -1, rx_remain = 0;
static uint8_t rx_buf[UART_BUFFER_SIZE];

static void rx_consume_byte(uint8_t byte) {
    if (rx_offset < 0) {
        if (byte == 0x9b) rx_offset = 0;
        return;
    }
    if (rx_remain == 0) {
        rx_remain = byte;
        return;
    }
    rx_buf[rx_offset++] = byte;
    if (--rx_remain == 0) {
        if (rx_buf[rx_offset - 1] == 0x9d) {
            // char hex_str[64] = {0};
            // for (int i = 0; i < rx_offset && i < 20; i++) {
            //     sprintf(hex_str + i * 3, "%02x ", rx_buf[i]);
            // }
            // ESP_LOGI(TAG, "RX[%d]: %s", rx_offset, hex_str);
            if (rx_buf[0] == 0x11) send_button_state(flexispot_button_state);
            if (rx_buf[0] == 0x12) display_packet_received(rx_buf, rx_offset);
        }
        rx_offset = -1;
    }
}

static void rx_task(void *pvParameters) {
    ESP_LOGI(TAG, "UART Rx task started");
    while (1) {
        uart_event_t event;
        uint8_t rx_buf[UART_BUFFER_SIZE];
        xQueueReceive(uart_queue, &event, portMAX_DELAY);
        if (event.type == UART_PATTERN_DET) {
            int pos = uart_pattern_pop_pos(FLEXISPOT_UART_NUM);
            if (pos == -1) continue;

            int len = uart_read_bytes(FLEXISPOT_UART_NUM, rx_buf, pos + 1, 0);
            for (int i = 0; i < len; i++) rx_consume_byte(rx_buf[i]);
        } else if (event.type == UART_DATA) {
            int len = uart_read_bytes(FLEXISPOT_UART_NUM, rx_buf, sizeof(rx_buf), 0);
            for (int i = 0; i < len; i++) rx_consume_byte(rx_buf[i]);
        }
    }
}
static esp_err_t rx_task_start(void) {
    BaseType_t ret = xTaskCreate(rx_task, "flexispot", FLEXISPOT_TASK_STACK_SIZE, NULL, FLEXISPOT_TASK_PRIORITY, NULL);
    return ret ? ESP_OK : ESP_FAIL;
}

esp_err_t flexispot_init(void) {
    esp_err_t ret = ESP_OK;

    // Install UART driver
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ret = uart_driver_install(FLEXISPOT_UART_NUM, UART_BUFFER_SIZE, 0, UART_QUEUE_SIZE, &uart_queue, 0);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
    ret = uart_param_config(FLEXISPOT_UART_NUM, &uart_config);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
    ret = uart_set_pin(FLEXISPOT_UART_NUM, FLEXISPOT_TXD_PIN, FLEXISPOT_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
    ret = uart_enable_pattern_det_baud_intr(FLEXISPOT_UART_NUM, 0x9D, 1, 0, 0, 0);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to set UART pattern: %s", esp_err_to_name(ret));
    ret = uart_pattern_queue_reset(FLEXISPOT_UART_NUM, UART_QUEUE_SIZE);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to reqet UART queue: %s", esp_err_to_name(ret));

    // Configure Detect PIN
    gpio_config_t detect_pin_config = {
        .pin_bit_mask = (1ULL << FLEXISPOT_DETECT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&detect_pin_config);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to configure detect pin: %s", esp_err_to_name(ret));
    gpio_set_level(FLEXISPOT_DETECT_PIN, 1); // set detect pin always high

    // Start UART Rx Task
    ret = rx_task_start();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "Failed to start UART Rx Task: %s", esp_err_to_name(ret));

    ESP_LOGI(TAG, "Flexispot initialized successfully");
    ESP_LOGI(TAG, "  UART: UART%d, TX: GPIO%d, RX: GPIO%d", FLEXISPOT_UART_NUM, FLEXISPOT_TXD_PIN, FLEXISPOT_RXD_PIN);
    return ESP_OK;
err:
    uart_driver_delete(FLEXISPOT_UART_NUM);
    return ret;
}
