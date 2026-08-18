#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -------------------- Pin assignment --------------------
#define LED1_PIN   GPIO_NUM_1
#define LED2_PIN   GPIO_NUM_0
#define LED3_PIN   GPIO_NUM_3

#define SERVO1_PIN GPIO_NUM_4
#define SERVO2_PIN GPIO_NUM_5

#define SWITCH_PIN GPIO_NUM_10

// -------------------- LED PWM --------------------
#define LED_PWM_FREQUENCY 5000
#define LED_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define LED1_MAX_BRIGHTNESS 10
#define LED2_MIN_BRIGHTNESS 2
#define LED2_MAX_BRIGHTNESS 20
#define LED3_MIN_BRIGHTNESS 2
#define LED3_MAX_BRIGHTNESS 20
#define LED_BREATH_STEP_MS  100
#define LED_BRIGHTEN_STEP_MS 8

// -------------------- Servo --------------------
#define SERVO_PWM_FREQUENCY 50
#define SERVO_PWM_BITS      14
#define SERVO_PWM_PERIOD_US 20000
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2400

#define SERVO_CENTER_ANGLE 90
#define SERVO_STEP_MS      15
#define SERVO_SETTLE_MS    300
#define SEQUENCE_START_DELAY_MS 1000

#define DEBOUNCE_MS 40

static void sleep_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static uint64_t millis_now(void)
{
    return (uint64_t)esp_timer_get_time() / 1000;
}

static ledc_channel_t led_channel_for_pin(gpio_num_t pin)
{
    if (pin == LED1_PIN) {
        return LEDC_CHANNEL_0;
    }
    if (pin == LED2_PIN) {
        return LEDC_CHANNEL_1;
    }
    return LEDC_CHANNEL_2;
}

static void set_led_brightness(gpio_num_t pin, uint8_t brightness)
{
    ledc_channel_t channel = led_channel_for_pin(pin);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, brightness));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void led1_breath_task(void *argument)
{
    (void)argument;
    int brightness = 0;
    int direction = 1;

    while (true) {
        set_led_brightness(LED1_PIN, (uint8_t)brightness);
        brightness += direction;

        if (brightness >= LED1_MAX_BRIGHTNESS) {
            brightness = LED1_MAX_BRIGHTNESS;
            direction = -1;
        } else if (brightness <= 0) {
            brightness = 0;
            direction = 1;
        }

        sleep_ms(LED_BREATH_STEP_MS);
    }
}

typedef struct {
    int servo1_angle;
    int servo2_angle;
    uint32_t delay_after_ms;
} servo_move_t;

static int servo1_position = SERVO_CENTER_ANGLE;
static int servo2_position = SERVO_CENTER_ANGLE;

static const servo_move_t SERVO_SEQUENCE[] = {
    {.servo1_angle = 130, .servo2_angle = 50, .delay_after_ms = 250},
    {.servo1_angle = 130, .servo2_angle = 90, .delay_after_ms = 50},
    {.servo1_angle = 130, .servo2_angle = 50, .delay_after_ms = 50},
    {.servo1_angle = 90, .servo2_angle = 50, .delay_after_ms = 50},

    {.servo1_angle = 130, .servo2_angle = 50, .delay_after_ms = 350},
    {.servo1_angle = 130, .servo2_angle = 90, .delay_after_ms = 650},



    {.servo1_angle = 130, .servo2_angle = 50, .delay_after_ms = 250},
    {.servo1_angle = 90,  .servo2_angle = 90, .delay_after_ms = 250},
};

static void all_leds_off(void)
{
    set_led_brightness(LED1_PIN, 0);
    set_led_brightness(LED2_PIN, LED2_MIN_BRIGHTNESS);
    set_led_brightness(LED3_PIN, LED3_MIN_BRIGHTNESS);
}

static void brighten_action_leds(void)
{
    for (int brightness = 0;
         brightness <= (LED2_MAX_BRIGHTNESS - LED2_MIN_BRIGHTNESS) ||
         brightness <= (LED3_MAX_BRIGHTNESS - LED3_MIN_BRIGHTNESS);
         brightness += 5) {
        int led2_value = LED2_MIN_BRIGHTNESS + brightness;
        int led3_value = LED3_MIN_BRIGHTNESS + brightness;
        uint8_t led2 = led2_value < LED2_MAX_BRIGHTNESS
            ? (uint8_t)led2_value : LED2_MAX_BRIGHTNESS;
        uint8_t led3 = led3_value < LED3_MAX_BRIGHTNESS
            ? (uint8_t)led3_value : LED3_MAX_BRIGHTNESS;
        set_led_brightness(LED2_PIN, led2);
        set_led_brightness(LED3_PIN, led3);
        sleep_ms(LED_BRIGHTEN_STEP_MS);
    }

    set_led_brightness(LED2_PIN, LED2_MAX_BRIGHTNESS);
    set_led_brightness(LED3_PIN, LED3_MAX_BRIGHTNESS);
}

static uint32_t servo_duty_for_angle(int angle)
{
    uint32_t pulse_us = SERVO_MIN_US +
        ((uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180);
    return pulse_us * ((1U << SERVO_PWM_BITS) - 1) / SERVO_PWM_PERIOD_US;
}

static void set_servo_angle(ledc_channel_t channel, int angle)
{
    uint32_t duty = servo_duty_for_angle(angle);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void attach_servo(ledc_channel_t channel, gpio_num_t pin,
                         int current_angle)
{
    ledc_channel_config_t config = {
        .gpio_num = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = servo_duty_for_angle(current_angle),
        .hpoint = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

static void detach_servo(ledc_channel_t channel, gpio_num_t pin)
{
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, channel, 0));
    ESP_ERROR_CHECK(gpio_reset_pin(pin));
}

static int absolute_value(int value)
{
    return value < 0 ? -value : value;
}

// Each move has an independent target angle for each servo. Both servos remain
// attached for the entire sequence and finish each move together.
static void move_servos(servo_move_t move)
{
    int servo1_start = servo1_position;
    int servo2_start = servo2_position;
    int servo1_distance = move.servo1_angle - servo1_start;
    int servo2_distance = move.servo2_angle - servo2_start;
    int steps = absolute_value(servo1_distance);
    int servo2_steps = absolute_value(servo2_distance);

    if (servo2_steps > steps) {
        steps = servo2_steps;
    }

    for (int step = 1; step <= steps; ++step) {
        int servo1_angle = servo1_start + servo1_distance * step / steps;
        int servo2_angle = servo2_start + servo2_distance * step / steps;
        set_servo_angle(LEDC_CHANNEL_3, servo1_angle);
        set_servo_angle(LEDC_CHANNEL_4, servo2_angle);
        sleep_ms(SERVO_STEP_MS);
    }

    if (steps == 0) {
        set_servo_angle(LEDC_CHANNEL_3, move.servo1_angle);
        set_servo_angle(LEDC_CHANNEL_4, move.servo2_angle);
    }

    sleep_ms(SERVO_SETTLE_MS);
    // Do not detach here: the next sequence move keeps continuous PWM.
    servo1_position = move.servo1_angle;
    servo2_position = move.servo2_angle;
}

static void run_test(void)
{
    printf("===== Sequence started =====\n");
    brighten_action_leds();
    attach_servo(LEDC_CHANNEL_3, SERVO1_PIN, servo1_position);
    attach_servo(LEDC_CHANNEL_4, SERVO2_PIN, servo2_position);

    for (size_t i = 0; i < sizeof(SERVO_SEQUENCE) / sizeof(SERVO_SEQUENCE[0]);
         ++i) {
        printf("Move %u: servo 1 -> %d, servo 2 -> %d, delay -> %u ms\n",
               (unsigned)(i + 1),
               SERVO_SEQUENCE[i].servo1_angle,
               SERVO_SEQUENCE[i].servo2_angle,
               (unsigned)SERVO_SEQUENCE[i].delay_after_ms);
        move_servos(SERVO_SEQUENCE[i]);
        sleep_ms(SERVO_SEQUENCE[i].delay_after_ms);
    }

    detach_servo(LEDC_CHANNEL_3, SERVO1_PIN);
    detach_servo(LEDC_CHANNEL_4, SERVO2_PIN);
    set_led_brightness(LED2_PIN, LED2_MIN_BRIGHTNESS);
    set_led_brightness(LED3_PIN, LED3_MIN_BRIGHTNESS);
    printf("===== Sequence completed =====\n");
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
        .duty_resolution = LED_PWM_RESOLUTION,
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
    all_leds_off();
    BaseType_t breath_task_created = xTaskCreate(
        led1_breath_task, "led1_breath", 2048, NULL, 5, NULL);
    ESP_ERROR_CHECK(breath_task_created == pdPASS ? ESP_OK : ESP_FAIL);

    printf("ESP32-C3 hardware test ready.\n");
    printf("Press the switch to start.\n");

    bool previous_switch_state = true;
    bool stable_switch_state = true;
    uint64_t last_debounce_time = 0;

    while (true) {
        uint64_t now = millis_now();
        bool current_switch_state = gpio_get_level(SWITCH_PIN) != 0;

        if (current_switch_state != previous_switch_state) {
            last_debounce_time = now;
        }

        if ((now - last_debounce_time) > DEBOUNCE_MS &&
            current_switch_state != stable_switch_state) {
            stable_switch_state = current_switch_state;

            // INPUT_PULLUP: pressed = low.
            if (!stable_switch_state) {
                printf("Switch pressed; sequence starts in %d ms.\n",
                       SEQUENCE_START_DELAY_MS);
                sleep_ms(SEQUENCE_START_DELAY_MS);
                run_test();
            }
        }

        previous_switch_state = current_switch_state;
        sleep_ms(5);
    }
}
