// BEN - Basic Edge Node
// minimal desktop robot platform for everyone.
//
// Copyright (c) 2026 Minjae Kim
// See LICENSE for the full MIT License text.
//
// RoBoHoN elbow-servo port of the webCamMotion follower (community
// contribution; based on the webCamMotion example).
//
// The laptop bridge (webcam_bridge.py) is UNCHANGED. It still sends BEN's
// original servo "angles" (servo 1: 90-135, servo 2: 50-90, home 90).
// This firmware converts each angle into a RoBoHoN raw position and sends
// Sharp head/elbow-bus frames instead of 50 Hz PWM.
//
// Servo assignment (viewed from the BACK of the robot):
//   LEFT  side = bridge servo 1 = RoBoHoN LEFT_ELBOW  (head bus SID 4)
//   RIGHT side = bridge servo 2 = RoBoHoN RIGHT_ELBOW (head bus SID 5)
//
// Bus: 115200 8N1, TX only, idle HIGH. The same frame is driven on GPIO 4
// AND GPIO 5 (the two BEN servo signal pads), so each servo can sit on
// either pad: it only obeys its own SID.
//
// Frame (30 bytes, verified on the RoBoHoN head bus):
//   77 80 00 03 06 00 | [lo hi flag sid] x5 | trailer lo hi flag | checksum
//   flag 0x40 = move/hold, 0xFF = relax
//   checksum = (256 - sum(bytes 6..28)) & 0xFF

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_sig_map.h"

// ------------------------------------------------------------
// Pins
// ------------------------------------------------------------
#define LED1_PIN        GPIO_NUM_1   // heart
#define LED2_PIN        GPIO_NUM_0   // eye
#define LED3_PIN        GPIO_NUM_3   // eye
#define BUS_PIN_A       GPIO_NUM_4   // BEN servo pad 1 (UART1 TX)
#define BUS_PIN_B       GPIO_NUM_5   // BEN servo pad 2 (mirror of UART1 TX)
#define SWITCH_PIN      GPIO_NUM_10

#define LED_PWM_FREQUENCY 5000

// ------------------------------------------------------------
// RoBoHoN bus
// ------------------------------------------------------------
#define BUS_UART            UART_NUM_1
#define BUS_BAUD            115200
#define FRAME_LEN           30
#define FLAG_MOVE           0x40
#define FLAG_RELAX          0xFF
#define RELAX_FRAME_COUNT   10      // same count the RoBoHoN firmware uses

#define SID_LEFT_ELBOW      4
#define SID_RIGHT_ELBOW     5

// Head-bus home values from a logic-analyzer capture of a stock RoBoHoN.
// SIDs 1-3 are not fitted on BEN; their home values are sent unchanged so
// every frame matches the verified capture apart from SID 4/5 positions.
#define HOME_NECK_TURN      239
#define HOME_HEAD_NOD       (-2530)
#define HOME_HEAD_TILT      2270
#define HOME_LEFT_ELBOW     1565
#define HOME_RIGHT_ELBOW    (-1256)
#define HOME_TRAILER        109     // 6D 00

// Raw counts per degree. 16-bit position = one output revolution
// (65536 / 360). Bench sweeps on a removed actuator measured roughly
// 172-194 counts/deg, so treat this value as a starting point and
// calibrate it (see README).
#define COUNTS_PER_DEG      182.04f

// Rotation sense. BEN's original servos turned CCW (viewed from the horn)
// for an increasing angle; RoBoHoN counts also increase CCW viewed from the
// spline. +1 keeps that relationship. Flip a sign if an arm swings the
// wrong way on the first bench test.
#define LEFT_DIR            (+1)
#define RIGHT_DIR           (+1)

// Hard safety window around home, in counts. BEN's commanded travel is
// 0..+45 deg (left) and -40..0 deg (right); this adds a small margin and
// stays inside the elbow limits measured on a stock RoBoHoN.
#define LEFT_MIN_COUNTS     (HOME_LEFT_ELBOW - 1000)
#define LEFT_MAX_COUNTS     (HOME_LEFT_ELBOW + 9200)
#define RIGHT_MIN_COUNTS    (HOME_RIGHT_ELBOW - 8300)
#define RIGHT_MAX_COUNTS    (HOME_RIGHT_ELBOW + 1000)

// ------------------------------------------------------------
// BEN angle space (unchanged from the PWM firmware / bridge)
// ------------------------------------------------------------
#define SERVO_HOME_ANGLE    90
#define SERVO1_MIN_ANGLE    90
#define SERVO1_MAX_ANGLE    135
#define SERVO2_MIN_ANGLE    50
#define SERVO2_MAX_ANGLE    90

// Motion: one frame every 20 ms, max 243 counts (~1.33 deg) per frame
// = ~67 deg/s, the same speed as the PWM firmware (1 deg / 15 ms).
#define SERVO_UPDATE_MS     20
#define MAX_STEP_COUNTS     243

// ------------------------------------------------------------
// Host link / button
// ------------------------------------------------------------
#define SWITCH_DEBOUNCE_MS   40
#define COMMAND_TIMEOUT_MS   1000
#define PACKET_SYNC_1        0xA5
#define PACKET_SYNC_2        0x5A
#define PACKET_DATA_SIZE     6
#define COMMAND_FLAG_ATTACH  0x01

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

// Current (ramped) and target positions in raw counts.
static int32_t left_pos  = HOME_LEFT_ELBOW;
static int32_t right_pos = HOME_RIGHT_ELBOW;
static int32_t left_target  = HOME_LEFT_ELBOW;
static int32_t right_target = HOME_RIGHT_ELBOW;

static bool attach_requested = false;   // from the bridge
static bool home_hold = false;          // button, no bridge connected
static bool servos_active = false;      // last thing sent was a move frame
static uint64_t last_command_ms = 0;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------
static void sleep_ms(uint32_t ms)
{
    // Always block for at least one tick so IDLE can run (task watchdog).
    TickType_t ticks = pdMS_TO_TICKS(ms);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

static uint64_t millis_now(void)
{
    return (uint64_t)esp_timer_get_time() / 1000;
}

static int clamp_int(int value, int minimum, int maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t angle_to_counts(int angle, int32_t home, int dir)
{
    float delta = (float)(angle - SERVO_HOME_ANGLE) * COUNTS_PER_DEG * (float)dir;
    int32_t offset = (int32_t)(delta >= 0.0f ? delta + 0.5f : delta - 0.5f);
    return home + offset;
}

static int32_t step_toward(int32_t pos, int32_t target)
{
    int32_t diff = target - pos;
    if (diff > MAX_STEP_COUNTS) {
        diff = MAX_STEP_COUNTS;
    } else if (diff < -MAX_STEP_COUNTS) {
        diff = -MAX_STEP_COUNTS;
    }
    return pos + diff;
}

static void set_led_brightness(ledc_channel_t channel, uint8_t brightness)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, brightness));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

// ------------------------------------------------------------
// RoBoHoN frames
// ------------------------------------------------------------
static uint8_t frame_checksum(const uint8_t *frame)
{
    uint32_t sum = 0;
    for (int i = 6; i < 29; ++i) {
        sum += frame[i];
    }
    return (uint8_t)((256 - (sum & 0xFF)) & 0xFF);
}

static void put_slot(uint8_t *buf, int idx, int16_t pos, uint8_t flag, uint8_t sid)
{
    buf[idx + 0] = (uint8_t)(pos & 0xFF);
    buf[idx + 1] = (uint8_t)((pos >> 8) & 0xFF);
    buf[idx + 2] = flag;
    buf[idx + 3] = sid;
}

static void bus_write_frame(const uint8_t *buf)
{
    uart_write_bytes(BUS_UART, (const char *)buf, FRAME_LEN);
}

static void send_move_frame(int32_t left, int32_t right)
{
    uint8_t buf[FRAME_LEN];
    buf[0] = 0x77;
    buf[1] = 0x80;
    buf[2] = 0x00;
    buf[3] = 0x03;
    buf[4] = 0x06;
    buf[5] = 0x00;
    put_slot(buf, 6,  HOME_NECK_TURN,   FLAG_MOVE, 1);
    put_slot(buf, 10, HOME_HEAD_NOD,    FLAG_MOVE, 2);
    put_slot(buf, 14, HOME_HEAD_TILT,   FLAG_MOVE, 3);
    put_slot(buf, 18, (int16_t)left,    FLAG_MOVE, SID_LEFT_ELBOW);
    put_slot(buf, 22, (int16_t)right,   FLAG_MOVE, SID_RIGHT_ELBOW);
    buf[26] = (uint8_t)(HOME_TRAILER & 0xFF);
    buf[27] = (uint8_t)((HOME_TRAILER >> 8) & 0xFF);
    buf[28] = FLAG_MOVE;
    buf[29] = frame_checksum(buf);
    bus_write_frame(buf);
}

static void send_relax(void)
{
    uint8_t buf[FRAME_LEN];
    buf[0] = 0x77;
    buf[1] = 0x80;
    buf[2] = 0x00;
    buf[3] = 0x03;
    buf[4] = 0x06;
    buf[5] = 0x00;
    for (uint8_t sid = 1; sid <= 5; ++sid) {
        put_slot(buf, 6 + (sid - 1) * 4, 0, FLAG_RELAX, sid);
    }
    buf[26] = 0x00;
    buf[27] = 0x00;
    buf[28] = FLAG_RELAX;
    buf[29] = frame_checksum(buf);
    for (int i = 0; i < RELAX_FRAME_COUNT; ++i) {
        bus_write_frame(buf);
        sleep_ms(30);
    }
}

// ------------------------------------------------------------
// Host packets (unchanged protocol)
// ------------------------------------------------------------
static void apply_command(const uint8_t data[PACKET_DATA_SIZE])
{
    int servo1 = clamp_int(data[0], SERVO1_MIN_ANGLE, SERVO1_MAX_ANGLE);
    int servo2 = clamp_int(data[1], SERVO2_MIN_ANGLE, SERVO2_MAX_ANGLE);
    left_target = clamp_int(angle_to_counts(servo1, HOME_LEFT_ELBOW, LEFT_DIR),
                            LEFT_MIN_COUNTS, LEFT_MAX_COUNTS);
    right_target = clamp_int(angle_to_counts(servo2, HOME_RIGHT_ELBOW, RIGHT_DIR),
                             RIGHT_MIN_COUNTS, RIGHT_MAX_COUNTS);
    set_led_brightness(LEDC_CHANNEL_0, data[2]);
    set_led_brightness(LEDC_CHANNEL_1, data[3]);
    set_led_brightness(LEDC_CHANNEL_2, data[4]);
    attach_requested = (data[5] & COMMAND_FLAG_ATTACH) != 0;
    last_command_ms = millis_now();
    home_hold = false;   // bridge has control
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

// ------------------------------------------------------------
// Servo update (called every SERVO_UPDATE_MS)
// ------------------------------------------------------------
static void update_servos(bool host_connected)
{
    bool want_active = (host_connected && attach_requested) || home_hold;

    if (!want_active) {
        if (servos_active) {
            send_relax();
            servos_active = false;
        }
        return;
    }

    if (home_hold && !host_connected) {
        left_target = HOME_LEFT_ELBOW;
        right_target = HOME_RIGHT_ELBOW;
    }

    left_pos = step_toward(left_pos, left_target);
    right_pos = step_toward(right_pos, right_target);
    send_move_frame(left_pos, right_pos);
    servos_active = true;
}

static void send_switch_event(void)
{
    static const char event[] = "EVENT:SPACE\n";
    (void)usb_serial_jtag_write_bytes(
        event, sizeof(event) - 1, pdMS_TO_TICKS(100));
}

// ------------------------------------------------------------
// Hardware setup
// ------------------------------------------------------------
static void configure_bus(void)
{
    uart_config_t config = {
        .baud_rate = BUS_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // RX buffer must exceed the hardware FIFO; RX itself is never used.
    ESP_ERROR_CHECK(uart_driver_install(BUS_UART, 256, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BUS_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(BUS_UART, BUS_PIN_A, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Mirror UART1 TX onto the second servo pad through the GPIO matrix.
    gpio_config_t mirror = {
        .pin_bit_mask = 1ULL << BUS_PIN_B,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&mirror));
    esp_rom_gpio_connect_out_signal(BUS_PIN_B, U1TXD_OUT_IDX, false, false);
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

    configure_bus();
}

void app_main(void)
{
    configure_hardware();
    usb_serial_jtag_driver_config_t usb_config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    // Start relaxed so the arms can be moved by hand at power-up.
    send_relax();

    packet_parser_t parser = {.state = PARSER_SYNC_1};
    bool previous_switch_state = true;
    bool stable_switch_state = true;
    uint64_t last_switch_change_ms = 0;
    uint64_t last_servo_update_ms = 0;
    last_command_ms = 0;
    bool ever_connected = false;

    while (true) {
        uint64_t now = millis_now();
        uint8_t input[32];
        int count = usb_serial_jtag_read_bytes(input, sizeof(input), 0);
        for (int i = 0; i < count; ++i) {
            parse_byte(&parser, input[i]);
        }
        if (last_command_ms != 0) {
            ever_connected = true;
        }

        bool host_connected = ever_connected &&
            (now - last_command_ms < COMMAND_TIMEOUT_MS);

        if (ever_connected && !host_connected && !home_hold) {
            attach_requested = false;
            set_led_brightness(LEDC_CHANNEL_0, 0);
            set_led_brightness(LEDC_CHANNEL_1, 0);
            set_led_brightness(LEDC_CHANNEL_2, 0);
        }

        if (now - last_servo_update_ms >= SERVO_UPDATE_MS) {
            update_servos(host_connected);
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
                if (host_connected) {
                    send_switch_event();
                } else {
                    // No bridge: button toggles "hold home" for fitting arms.
                    home_hold = !home_hold;
                    set_led_brightness(LEDC_CHANNEL_0, home_hold ? 20 : 0);
                }
            }
        }
        previous_switch_state = current_switch_state;
        sleep_ms(5);
    }
}
