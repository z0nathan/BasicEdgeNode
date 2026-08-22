// BEN - Basic Edge Node
// minimal desktop robot platform for everyone.
// 
// 
// 
// Copyright (c) 2026 Minjae Kim
//
// See LICENSE for the full MIT License text.



#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// The laptop owns all behavior. This firmware only follows safe targets.
#define LED1_PIN   GPIO_NUM_1
#define LED2_PIN   GPIO_NUM_0
#define LED3_PIN   GPIO_NUM_3
#define SERVO1_PIN GPIO_NUM_4
#define SERVO2_PIN GPIO_NUM_5
#define SWITCH_PIN GPIO_NUM_10

#define LED_PWM_FREQUENCY 5000

#define SERVO_PWM_FREQUENCY  50
#define SERVO_PWM_BITS       14
#define SERVO_PWM_PERIOD_US  20000
#define SERVO_MIN_US         500
#define SERVO_MAX_US         2400
#define SERVO_CENTER_ANGLE   90
#define SERVO1_MIN_ANGLE     90
#define SERVO1_MAX_ANGLE     135
#define SERVO2_MIN_ANGLE     50
#define SERVO2_MAX_ANGLE     90
#define SERVO_UPDATE_MS      15

#define SWITCH_DEBOUNCE_MS       40
#define COMMAND_TIMEOUT_MS       1000
#define PACKET_SYNC_1            0xA5
#define PACKET_SYNC_2            0x5A
#define PACKET_DATA_SIZE         6
#define COMMAND_FLAG_ATTACH      0x01

typedef enum {
    PARSER_SYNC_1,
    PARSER_SYNC_2,
    PARSER_DATA,
    PARSER_CHECKSUM,
} parser_state_t;

typedef struct {
    parser_state_t state;
    uint8_t data[PACKET_DATA_SIZE];
    uint8_t index;
} packet_parser_t;

static int servo1_angle = SERVO_CENTER_ANGLE;
static int servo2_angle = SERVO_CENTER_ANGLE;
static int servo1_target = SERVO_CENTER_ANGLE;
static int servo2_target = SERVO_CENTER_ANGLE;
static bool servos_attached = false;
static bool attach_requested = false;
static uint64_t last_command_ms = 0;

static void sleep_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static uint64_t millis_now(void)
{
    return (uint64_t)esp_timer_get_time() / 1000;
}

static int clamp_angle(int angle, int minimum, int maximum)
{
    if (angle < minimum) {
        return minimum;
    }
    if (angle > maximum) {
        return maximum;
    }
    return angle;
}

static void set_led_brightness(ledc_channel_t channel, uint8_t brightness)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, brightness));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static uint32_t servo_duty_for_angle(int angle)
{
    uint32_t pulse_us = SERVO_MIN_US +
        ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180);
    return pulse_us * ((1U << SERVO_PWM_BITS) - 1) / SERVO_PWM_PERIOD_US;
}

static void set_servo_angle(ledc_channel_t channel, int angle)
{
    ESP_ERROR_CHECK(ledc_set_duty(
        LEDC_LOW_SPEED_MODE, channel, servo_duty_for_angle(angle)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void attach_servo(ledc_channel_t channel, gpio_num_t pin, int angle)
{
    ledc_channel_config_t config = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = servo_duty_for_angle(angle),
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

static void attach_servos(void)
{
    if (servos_attached) {
        return;
    }
    attach_servo(LEDC_CHANNEL_3, SERVO1_PIN, servo1_angle);
    attach_servo(LEDC_CHANNEL_4, SERVO2_PIN, servo2_angle);
    servos_attached = true;
}

static void detach_servos(void)
{
    if (!servos_attached) {
        return;
    }
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0));
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, 0));
    ESP_ERROR_CHECK(gpio_reset_pin(SERVO1_PIN));
    ESP_ERROR_CHECK(gpio_reset_pin(SERVO2_PIN));
    servos_attached = false;
}

static void apply_command(const uint8_t data[PACKET_DATA_SIZE])
{
    servo1_target = clamp_angle(
        data[0], SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
    servo2_target = clamp_angle(
        data[1], SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
    set_led_brightness(LEDC_CHANNEL_0, data[2]);
    set_led_brightness(LEDC_CHANNEL_1, data[3]);
    set_led_brightness(LEDC_CHANNEL_2, data[4]);
    attach_requested = (data[5] & COMMAND_FLAG_ATTACH) != 0;
    last_command_ms = millis_now();
}

static void parse_byte(packet_parser_t *parser, uint8_t byte)
{
    if (parser->state == PARSER_SYNC_1) {
        if (byte == PACKET_SYNC_1) {
            parser->state = PARSER_SYNC_2;
        }
    } else if (parser->state == PARSER_SYNC_2) {
        if (byte == PACKET_SYNC_2) {
            parser->index = 0;
            parser->state = PARSER_DATA;
        } else {
            parser->state = byte == PACKET_SYNC_1
                ? PARSER_SYNC_2 : PARSER_SYNC_1;
        }
    } else if (parser->state == PARSER_DATA) {
        parser->data[parser->index++] = byte;
        if (parser->index == PACKET_DATA_SIZE) {
            parser->state = PARSER_CHECKSUM;
        }
    } else {
        uint8_t checksum = 0;
        for (int i = 0; i < PACKET_DATA_SIZE; ++i) {
            checksum = (uint8_t)(checksum + parser->data[i]);
        }
        if (checksum == byte) {
            apply_command(parser->data);
        }
        parser->state = PARSER_SYNC_1;
    }
}

static void update_servos(void)
{
    if (!attach_requested) {
        detach_servos();
        return;
    }

    attach_servos();
    if (servo1_angle < servo1_target) {
        ++servo1_angle;
    } else if (servo1_angle > servo1_target) {
        --servo1_angle;
    }
    if (servo2_angle < servo2_target) {
        ++servo2_angle;
    } else if (servo2_angle > servo2_target) {
        --servo2_angle;
    }
    set_servo_angle(LEDC_CHANNEL_3, servo1_angle);
    set_servo_angle(LEDC_CHANNEL_4, servo2_angle);
}

static void send_switch_event(void)
{
    static const char event[] = "EVENT:SPACE\n";
    (void)usb_serial_jtag_write_bytes(
        event, sizeof(event) - 1, pdMS_TO_TICKS(100));
}

static void configure_hardware(void)
{
    gpio_config_t switch_config = {
        .pin_bit_mask = 1ULL << SWITCH_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&switch_config));

    ledc_timer_config_t led_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = LED_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&led_timer));

    const gpio_num_t led_pins[] = {LED1_PIN, LED2_PIN, LED3_PIN};
    for (int i = 0; i < 3; ++i) {
        ledc_channel_config_t channel = {
            .gpio_num = led_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel));
    }

    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));
}

void app_main(void)
{
    configure_hardware();
    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    packet_parser_t parser = {.state = PARSER_SYNC_1};
    bool previous_switch_state = true;
    bool stable_switch_state = true;
    uint64_t last_switch_change_ms = 0;
    uint64_t last_servo_update_ms = 0;
    last_command_ms = millis_now();

    while (true) {
        uint64_t now = millis_now();
        uint8_t input[32];
        int count = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
        for (int i = 0; i < count; ++i) {
            parse_byte(&parser, input[i]);
        }

        if (now - last_command_ms >= COMMAND_TIMEOUT_MS) {
            attach_requested = false;
            set_led_brightness(LEDC_CHANNEL_0, 0);
            set_led_brightness(LEDC_CHANNEL_1, 0);
            set_led_brightness(LEDC_CHANNEL_2, 0);
        }

        if (now - last_servo_update_ms >= SERVO_UPDATE_MS) {
            update_servos();
            last_servo_update_ms = now;
        }

        bool current_switch_state = gpio_get_level(SWITCH_PIN) != 0;
        if (current_switch_state != previous_switch_state) {
            last_switch_change_ms = now;
        }
        if (now - last_switch_change_ms >= SWITCH_DEBOUNCE_MS &&
            current_switch_state != stable_switch_state) {
            stable_switch_state = current_switch_state;
            if (!stable_switch_state) {
                send_switch_event();
            }
        }
        previous_switch_state = current_switch_state;
        sleep_ms(5);
    }
}
