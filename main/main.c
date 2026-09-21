#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_app_desc.h"
#include "vl53l1x.h"
#include <math.h>
#include <stdlib.h>

static const char *TAG = "main";

// Waveshare ESP32-C6-LCD-1.47 pin definitions
#define PIN_LCD_MOSI    6
#define PIN_LCD_SCLK    7
#define PIN_LCD_CS      14
#define PIN_LCD_DC      15
#define PIN_LCD_RST     21
#define PIN_LCD_BL      22

// Servo PWM pin definitions
#define PIN_SERVO_LEFT  0
#define PIN_SERVO_RIGHT 1

// Motoron M3T453 I2C motor controller
#define PIN_I2C_SDA     4
#define PIN_I2C_SCL     5
#define MOTORON_ADDR    16    // default 7-bit I2C address

// Qwiic sensors sharing the same I2C bus (GPIO4/5)
#define LSM303_ACC_ADDR 0x19  // LSM303AGR accelerometer
#define LSM303_MAG_ADDR 0x1E  // LSM303AGR magnetometer
#define MOTOR_LEFT_CH   1     // Motoron motor channel 1
#define MOTOR_RIGHT_CH  2     // Motoron motor channel 2
#define MOTOR_SPEED     400   // 0..800 (half power for a gentle demo)
// Wheels are mounted opposite each other: +speed on both would spin in place.
// Right motor is inverted so "forward" on both channels is straight ahead.
#define MOTOR_LEFT_POLARITY   1
#define MOTOR_RIGHT_POLARITY -1
#define STRAIGHT_Y_GAIN  180   // motor units per g of lateral (Y) accel (keep gentle)
#define STRAIGHT_Y_DEAD  0.08f // ignore small Y noise / bumps
#define STRAIGHT_Y_LP    0.12f // EMA blend toward new Y (lower = slower / less twitchy)
#define MOTOR_SPEED_MIN  80
#define MOTOR_SPEED_MAX  800

// Reactive-behaviour tuning
#define LIDAR_STOP_MM   100   // obstacle distance that halts forward driving (10 cm)
#define LIDAR_CLEAR_MM  300   // path ahead must reopen past this before resuming forward
#define LIDAR_MAX_MM    3600  // VL53L1X long-mode max (Adafruit/ST: 360 cm)
#define LIDAR_MIN_MM    30    // below ~30 mm is below the sensor's rated floor
#define STALL_G_THRESH  0.04f // min accel jitter (g) expected while rolling
#define TURN_TIMEOUT_MS 12500 // safety cap if no opening appears while spinning
#define BACK_MS         3000  // reverse duration after hitting a 100 mm obstacle
#define MAP_WIDTH_MM    6000  // canvas width in millimetres (centre to edge = 3000 mm)
#define MAP_RADIUS_MM   (MAP_WIDTH_MM / 2)
#define MAP_LOG_NEAR_MM 30    // log floor so nearby lidar hits are not crushed to the origin
#define MAP_FIFO_LEN    200   // sliding window of lidar hits (~20 s at 100 ms)
#define MAP_SAMPLE_MS   100   // how often a hit is pushed into the FIFO / map

// Servo PWM parameters (50 Hz, 14-bit resolution)
#define SERVO_FREQ_HZ       50
#define SERVO_TIMER_RES     LEDC_TIMER_14_BIT
#define SERVO_PULSE_MIN_US  500   // 0 degrees
#define SERVO_PULSE_MAX_US  2500  // 180 degrees

// Display parameters
#define LCD_H_RES       172
#define LCD_V_RES       320
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_PIXEL_CLK   (40 * 1000 * 1000)
#define LCD_DRAW_LINES  50

// Convert angle (0-180) to LEDC duty value
static uint32_t servo_angle_to_duty(int angle)
{
    uint32_t pulse_us = SERVO_PULSE_MIN_US +
        (uint32_t)(angle) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180;
    // duty = pulse_us / period_us * max_duty
    // period_us = 1000000 / 50 = 20000
    uint32_t max_duty = (1 << 14) - 1;  // 14-bit
    return pulse_us * max_duty / 20000;
}

static void servo_set_angle(ledc_channel_t channel, int angle)
{
    uint32_t duty = servo_angle_to_duty(angle);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void servo_init(void)
{
    // Each servo gets its own LEDC timer so the two 50 Hz signals are fully
    // independent (left=TIMER_0/CH_0/GPIO0, right=TIMER_1/CH_1/GPIO1).
    ledc_timer_config_t timer_left = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_TIMER_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_left));

    ledc_timer_config_t timer_right = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_TIMER_RES,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_right));

    ledc_channel_config_t ch_left = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_LEFT,
        .duty = servo_angle_to_duty(90),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_left));

    ledc_channel_config_t ch_right = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_RIGHT,
        .duty = servo_angle_to_duty(90),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_right));

    ESP_LOGI(TAG, "Servos initialized: LEFT=GPIO%d (TIMER0) RIGHT=GPIO%d (TIMER1)",
             PIN_SERVO_LEFT, PIN_SERVO_RIGHT);
}

// Quick test: pulse each servo pin HIGH for 500ms to verify wiring
static void servo_test_pulse(void)
{
    ESP_LOGI(TAG, "Test pulse: GPIO%d and GPIO%d HIGH for 500ms",
             PIN_SERVO_LEFT, PIN_SERVO_RIGHT);

    // Temporarily disable LEDC and drive pin high as GPIO
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1);
    ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 1);

    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_SERVO_LEFT) | (1ULL << PIN_SERVO_RIGHT),
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_SERVO_LEFT, 1);
    gpio_set_level(PIN_SERVO_RIGHT, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(PIN_SERVO_LEFT, 0);
    gpio_set_level(PIN_SERVO_RIGHT, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Test pulse done. Re-initializing LEDC...");

    // Re-init LEDC channels (timer already configured)
    ledc_channel_config_t ch_left = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_LEFT,
        .duty = servo_angle_to_duty(90),
        .hpoint = 0,
    };
    ledc_channel_config(&ch_left);

    ledc_channel_config_t ch_right = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_SERVO_RIGHT,
        .duty = servo_angle_to_duty(90),
        .hpoint = 0,
    };
    ledc_channel_config(&ch_right);
}

// --- Motoron M3T453 I2C motor controller ---
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_motoron = NULL;

// Tracks whether the Motoron is currently acknowledging, to avoid log spam
static bool s_motoron_ok = true;

static void motoron_send(const uint8_t *cmd, size_t len)
{
    if (!s_motoron) {
        return;
    }
    esp_err_t err = i2c_master_transmit(s_motoron, cmd, len, 100);
    if (err != ESP_OK) {
        if (s_motoron_ok) {
            ESP_LOGW(TAG, "Motoron not responding (%s) - check VDD/GND/SDA/SCL "
                          "wiring and that addr=%d is correct",
                     esp_err_to_name(err), MOTORON_ADDR);
        }
        s_motoron_ok = false;
    } else {
        if (!s_motoron_ok) {
            ESP_LOGI(TAG, "Motoron responding again.");
        }
        s_motoron_ok = true;
    }
}

// Scan the I2C bus and report what (if anything) is present
static void motoron_scan(void)
{
    ESP_LOGI(TAG, "Scanning I2C bus (SDA=GPIO%d SCL=GPIO%d)...",
             PIN_I2C_SDA, PIN_I2C_SCL);
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            const char *name = "?";
            switch (addr) {
            case MOTORON_ADDR:      name = "Motoron M3T453";       break;
            case LSM303_ACC_ADDR:   name = "LSM303AGR accel";      break;
            case LSM303_MAG_ADDR:   name = "LSM303AGR mag";        break;
            case VL53L1X_I2C_ADDR:  name = "VL53L1X lidar";        break;
            default:                                               break;
            }
            ESP_LOGI(TAG, "  found I2C device at 0x%02X (%d) - %s", addr, addr, name);
            found++;
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "  no I2C devices found - check Motoron power and wiring");
    }
    if (i2c_master_probe(s_i2c_bus, MOTORON_ADDR, 50) == ESP_OK) {
        ESP_LOGI(TAG, "Motoron present at address %d.", MOTORON_ADDR);
    } else {
        ESP_LOGW(TAG, "Motoron NOT found at address %d.", MOTORON_ADDR);
    }
}

static void motoron_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = PIN_I2C_SCL,
        .sda_io_num = PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));

    motoron_scan();

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MOTORON_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_motoron));

    // Reinitialize to default state (CRC byte ok even if CRC disabled)
    motoron_send((const uint8_t[]){0x96, 0x74}, 2);
    vTaskDelay(pdMS_TO_TICKS(5));
    // Disable CRC for commands so no per-command CRC byte is needed
    motoron_send((const uint8_t[]){0x8B, 0x04, 0x7B, 0x43}, 4);
    // Clear the reset flag (otherwise motors are disabled as an error)
    motoron_send((const uint8_t[]){0xA9, 0x00, 0x04}, 3);
    // Set variable: Error mask = 0 (offset 0x10) to disable the command
    // timeout as an error, so motors hold across the multi-second servo
    // phases. "Set variable" command byte is 0x9C, motor 0 (general).
    motoron_send((const uint8_t[]){0x9C, 0x00, 0x10, 0x00, 0x00}, 5);

    ESP_LOGI(TAG, "Motoron initialized on I2C SDA=GPIO%d SCL=GPIO%d addr=%d",
             PIN_I2C_SDA, PIN_I2C_SCL, MOTORON_ADDR);
}

// Set Speed (now mode): speed is -800..800, encoded as 14-bit two's complement
static void motoron_set_speed(uint8_t motor, int16_t speed)
{
    if (motor == MOTOR_LEFT_CH) {
        speed = (int16_t)(speed * MOTOR_LEFT_POLARITY);
    } else if (motor == MOTOR_RIGHT_CH) {
        speed = (int16_t)(speed * MOTOR_RIGHT_POLARITY);
    }
    uint8_t cmd[4] = {
        0xD2,
        (uint8_t)(motor & 0x7F),
        (uint8_t)(speed & 0x7F),
        (uint8_t)((speed >> 7) & 0x7F),
    };
    motoron_send(cmd, sizeof(cmd));
}

static inline int16_t clamp_motor_speed(int v)
{
    if (v > MOTOR_SPEED_MAX) {
        return MOTOR_SPEED_MAX;
    }
    if (v < MOTOR_SPEED_MIN) {
        return MOTOR_SPEED_MIN;
    }
    return (int16_t)v;
}

static inline void motor_fwd(uint8_t motor)  { motoron_set_speed(motor, MOTOR_SPEED); }
static inline void motor_stop(uint8_t motor) { motoron_set_speed(motor, 0); }
static inline void motor_back(uint8_t motor) { motoron_set_speed(motor, (int16_t)(-MOTOR_SPEED)); }

// --- Qwiic sensors: LSM303AGR (accel + mag) and VL53L1X (lidar) ---
static i2c_master_dev_handle_t s_lsm_acc = NULL;
static i2c_master_dev_handle_t s_lsm_mag = NULL;
static vl53l1x_t s_lidar;

static bool s_have_accel = false;
static bool s_have_mag = false;
static bool s_have_lidar = false;

// Latest readings, shared with the UI (only the sensor task writes them)
static uint16_t s_lidar_mm = 0;
static int s_look_deg = 90;   // current head angle (90 = forward)
static float s_acc_g[3] = {0};
static float s_mag_ut[3] = {0};
static float s_y_filt = 0.0f;
static bool s_next_turn_left = true;

// Drive forward while steering with accel Y: +Y (leftward) means we are
// veering left, so speed up the left wheel / slow the right. Y is low-pass
// filtered so bumps and vibration do not yank the wheels.
static void motor_drive_straight(void)
{
    int left = MOTOR_SPEED;
    int right = MOTOR_SPEED;
    if (s_have_accel) {
        s_y_filt += STRAIGHT_Y_LP * (s_acc_g[1] - s_y_filt);
        if (fabsf(s_y_filt) > STRAIGHT_Y_DEAD) {
            int corr = (int)(s_y_filt * (float)STRAIGHT_Y_GAIN);
            left += corr;
            right -= corr;
        }
    }
    motoron_set_speed(MOTOR_LEFT_CH, clamp_motor_speed(left));
    motoron_set_speed(MOTOR_RIGHT_CH, clamp_motor_speed(right));
}

static esp_err_t lsm_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, sizeof(buf), 100);
}

// Read n bytes. The accelerometer needs the auto-increment bit (0x80) set on the
// sub-address; the magnetometer auto-increments on its own.
static esp_err_t lsm_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, n, 100);
}

static bool lsm303_init(void)
{
    i2c_device_config_t acc_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM303_ACC_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_device_config_t mag_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM303_MAG_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &acc_cfg, &s_lsm_acc));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &mag_cfg, &s_lsm_mag));

    uint8_t who = 0;
    // Accelerometer: WHO_AM_I_A (0x0F) == 0x33; enable XYZ @ 100 Hz, ±2 g
    if (lsm_read(s_lsm_acc, 0x0F, &who, 1) == ESP_OK && who == 0x33) {
        lsm_write8(s_lsm_acc, 0x20, 0x57);   // CTRL_REG1_A: 100 Hz, XYZ enabled
        lsm_write8(s_lsm_acc, 0x23, 0x00);   // CTRL_REG4_A: ±2 g, normal mode
        s_have_accel = true;
    } else {
        ESP_LOGW(TAG, "LSM303AGR accel not found at 0x%02X", LSM303_ACC_ADDR);
    }

    // Magnetometer: WHO_AM_I_M (0x4F) == 0x40; continuous mode, temp comp
    who = 0;
    if (lsm_read(s_lsm_mag, 0x4F, &who, 1) == ESP_OK && who == 0x40) {
        lsm_write8(s_lsm_mag, 0x60, 0x80);   // CFG_REG_A_M: continuous, temp comp
        s_have_mag = true;
    } else {
        ESP_LOGW(TAG, "LSM303AGR mag not found at 0x%02X", LSM303_MAG_ADDR);
    }

    return s_have_accel || s_have_mag;
}

static void lsm303_read_accel(void)
{
    uint8_t b[6];
    if (lsm_read(s_lsm_acc, 0x28 | 0x80, b, sizeof(b)) != ESP_OK) {
        return;
    }
    // Normal mode: 10-bit left-justified; ±2 g -> 3.9 mg per LSB
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
        s_acc_g[i] = (raw >> 6) * 0.0039f;
    }
}

static void lsm303_read_mag(void)
{
    uint8_t b[6];
    if (lsm_read(s_lsm_mag, 0x68, b, sizeof(b)) != ESP_OK) {
        return;
    }
    // LSM303AGR magnetometer: 1.5 mgauss per LSB -> 0.15 uT per LSB
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((uint16_t)b[2 * i] | ((uint16_t)b[2 * i + 1] << 8));
        s_mag_ut[i] = raw * 0.15f;
    }
}

static void sensors_init(void)
{
    lsm303_init();
    if (vl53l1x_init(s_i2c_bus, &s_lidar) == ESP_OK) {
        s_have_lidar = true;
    } else {
        ESP_LOGW(TAG, "VL53L1X lidar not available");
    }
    ESP_LOGI(TAG, "Sensors: lidar=%d accel=%d mag=%d",
             s_have_lidar, s_have_accel, s_have_mag);
}

// --- LVGL status tiles (2x2 grid) ---
typedef enum {
    TILE_LSERVO = 0,
    TILE_RSERVO,
    TILE_LMOTOR,
    TILE_RMOTOR,
    TILE_COUNT,
} tile_id_t;

static lv_obj_t *tile_panel[TILE_COUNT];
static lv_obj_t *tile_value[TILE_COUNT];

// Full-width sensor readout below the 2x2 grid (lidar / accel / mag)
static lv_obj_t *s_sensor_label = NULL;

// Polar occupancy map in the top half of the screen
static lv_obj_t *s_map_canvas = NULL;
static lv_draw_buf_t *s_map_buf = NULL;
static int s_map_w = 0;
static int s_map_h = 0;

typedef struct {
    uint16_t mm;
    float bearing_deg;
} map_hit_t;

static map_hit_t s_map_fifo[MAP_FIFO_LEN];
static uint16_t s_map_fifo_head = 0;   // next write index
static uint16_t s_map_fifo_count = 0;

#define MAP_BG    lv_color_make(0x05, 0x07, 0x0D)
#define MAP_DOT   lv_color_make(0x33, 0xE1, 0xFF)
#define MAP_CROSS lv_color_make(0x3A, 0x3C, 0x4A)
#define MAP_NORTH lv_color_make(0xE0, 0x28, 0x28)
#define MAP_NORTH_R 3

// Update a tile's value text and highlight it as active (blue) or idle (dark)
static void set_tile(tile_id_t tile, const char *text, bool active)
{
    if (tile >= TILE_COUNT || !tile_panel[tile]) {
        return;
    }
    lv_label_set_text(tile_value[tile], text);
    lv_obj_set_style_bg_color(tile_panel[tile],
        active ? lv_color_make(0x00, 0x44, 0x60) : lv_color_make(0x20, 0x20, 0x30),
        LV_PART_MAIN);
    lv_obj_set_style_border_color(tile_panel[tile],
        active ? lv_color_make(0x33, 0xE1, 0xFF) : lv_color_make(0x40, 0x40, 0x50),
        LV_PART_MAIN);
    lv_obj_set_style_text_color(tile_value[tile],
        active ? lv_color_make(0x33, 0xE1, 0xFF) : lv_color_make(0xFF, 0xFF, 0x00),
        LV_PART_MAIN);
}

// Update all four status tiles at once (servo angles + motor states)
static void ui_set_tiles(const char *ls, const char *rs,
                         const char *lm, const char *rm,
                         bool ls_a, bool rs_a, bool lm_a, bool rm_a)
{
    if (lvgl_port_lock(0)) {
        set_tile(TILE_LSERVO, ls, ls_a);
        set_tile(TILE_RSERVO, rs, rs_a);
        set_tile(TILE_LMOTOR, lm, lm_a);
        set_tile(TILE_RMOTOR, rm, rm_a);
        lvgl_port_unlock();
    }
}

// Current lidar distance in mm, or 0 when there is no valid reading
static uint16_t lidar_mm_now(void)
{
    if (!s_have_lidar) {
        return 0;
    }
    uint16_t d = s_lidar_mm;
    // 0 is what the driver reports for no-target / out-of-range (range status
    // not valid). Distances outside the long-mode window are also discarded.
    if (d == 0 || d < LIDAR_MIN_MM || d > LIDAR_MAX_MM) {
        return 0;
    }
    return d;
}

// Magnitude of the accelerometer vector (~1 g at rest)
static float accel_mag(void)
{
    return sqrtf(s_acc_g[0] * s_acc_g[0] +
                 s_acc_g[1] * s_acc_g[1] +
                 s_acc_g[2] * s_acc_g[2]);
}

// Compass heading in degrees [0, 360) from the horizontal magnetometer axes.
// No tilt compensation - fine for a robot that stays flat on the ground.
static float mag_heading_deg(void)
{
    float heading = atan2f(s_mag_ut[1], s_mag_ut[0]) * 180.0f / (float)M_PI;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

// Log-scaled radius so a 30 mm hit is well away from the origin and MAP_RADIUS_MM
// (3000 mm, half of the 6000 mm map width) lands on the canvas border.
static float map_log_radius_px(uint16_t dist_mm, int r_max)
{
    float t = logf(1.0f + (float)dist_mm / (float)MAP_LOG_NEAR_MM) /
              logf(1.0f + (float)MAP_RADIUS_MM / (float)MAP_LOG_NEAR_MM);
    if (t > 1.0f) {
        t = 1.0f;
    }
    return t * (float)r_max;
}

static int map_r_max(void)
{
    int r = ((s_map_w < s_map_h) ? s_map_w : s_map_h) / 2 - 1;
    return r > 0 ? r : 0;
}

static void map_polar_xy(float bearing_deg, float radius, int *x, int *y)
{
    float rad = bearing_deg * (float)M_PI / 180.0f;
    *x = s_map_w / 2 + (int)roundf(radius * sinf(rad));
    *y = s_map_h / 2 - (int)roundf(radius * cosf(rad));
}

static void map_set_px(int x, int y, lv_color_t c)
{
    if (x >= 0 && x < s_map_w && y >= 0 && y < s_map_h) {
        lv_canvas_set_px(s_map_canvas, x, y, c, LV_OPA_COVER);
    }
}

static void map_draw_line(int x0, int y0, int x1, int y1, lv_color_t c)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        map_set_px(x0, y0, c);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

static void map_fill_circle(int cx, int cy, int r, lv_color_t c)
{
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r) {
                map_set_px(cx + dx, cy + dy, c);
            }
        }
    }
}

// Dark N-S / E-W cross in the same polar frame as the lidar dots. Magnetic
// north is the bottom of the screen; a red circle marks that tip.
static void map_draw_compass(void)
{
    if (!s_map_canvas) {
        return;
    }
    int r = map_r_max() - MAP_NORTH_R - 1;
    if (r < 8) {
        r = map_r_max();
    }

    lv_display_t *disp = lv_obj_get_display(s_map_canvas);
    lv_display_enable_invalidation(disp, false);

    int x0, y0, x1, y1;
    map_polar_xy(180.0f, (float)r, &x0, &y0);
    map_polar_xy(0.0f, (float)r, &x1, &y1);
    map_draw_line(x0, y0, x1, y1, MAP_CROSS);
    map_polar_xy(270.0f, (float)r, &x0, &y0);
    map_polar_xy(90.0f, (float)r, &x1, &y1);
    map_draw_line(x0, y0, x1, y1, MAP_CROSS);

    int nx, ny;
    map_polar_xy(180.0f, (float)r, &nx, &ny);
    map_fill_circle(nx, ny, MAP_NORTH_R, MAP_NORTH);

    lv_display_enable_invalidation(disp, true);
    lv_obj_invalidate(s_map_canvas);
}

static void map_fifo_push(uint16_t mm, float bearing_deg)
{
    s_map_fifo[s_map_fifo_head].mm = mm;
    s_map_fifo[s_map_fifo_head].bearing_deg = bearing_deg;
    s_map_fifo_head = (uint16_t)((s_map_fifo_head + 1) % MAP_FIFO_LEN);
    if (s_map_fifo_count < MAP_FIFO_LEN) {
        s_map_fifo_count++;
    }
}

static void map_plot_sample(uint16_t mm, float bearing_deg)
{
    if (mm == 0) {
        return;
    }
    int r_max = map_r_max();
    float r = map_log_radius_px(mm, r_max);
    int x, y;
    map_polar_xy(bearing_deg, r, &x, &y);
    map_set_px(x, y, MAP_DOT);
}

// Redraw the last MAP_FIFO_LEN hits so the cloud scrolls as old samples drop off.
static void map_redraw(void)
{
    if (!s_map_canvas) {
        return;
    }
    if (!lvgl_port_lock(0)) {
        return;
    }

    lv_display_t *disp = lv_obj_get_display(s_map_canvas);
    lv_display_enable_invalidation(disp, false);

    lv_canvas_fill_bg(s_map_canvas, MAP_BG, LV_OPA_COVER);

    uint16_t start = (uint16_t)((s_map_fifo_head + MAP_FIFO_LEN - s_map_fifo_count) % MAP_FIFO_LEN);
    for (uint16_t i = 0; i < s_map_fifo_count; i++) {
        map_hit_t *h = &s_map_fifo[(start + i) % MAP_FIFO_LEN];
        map_plot_sample(h->mm, h->bearing_deg);
    }

    lv_display_enable_invalidation(disp, true);
    map_draw_compass();

    lvgl_port_unlock();
}

// Shortest signed angle from `from` to `to`, in (-180, 180] degrees
static float heading_diff_deg(float from, float to)
{
    float d = to - from;
    while (d > 180.0f) { d -= 360.0f; }
    while (d < -180.0f) { d += 360.0f; }
    return d;
}

// Point both servos to an angle as a "look" gesture and reflect it on the tiles
static void look(int angle)
{
    s_look_deg = angle;
    servo_set_angle(LEDC_CHANNEL_0, angle);
    servo_set_angle(LEDC_CHANNEL_1, angle);
    char buf[8];
    snprintf(buf, sizeof(buf), "%d\xC2\xB0", angle);
    ui_set_tiles(buf, buf, "--", "--", true, true, false, false);
}

// Accumulate accel "jitter" over ~600 ms, aborting if something comes within
// LIDAR_STOP_MM so we do not roll closer than 100 mm.
static float measure_movement(void)
{
    float prev = s_have_accel ? accel_mag() : 1.0f;
    float total = 0.0f;
    for (int i = 0; i < 6; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        uint16_t d = lidar_mm_now();
        if (d != 0 && d <= LIDAR_STOP_MM) {
            motor_stop(MOTOR_LEFT_CH);
            motor_stop(MOTOR_RIGHT_CH);
            return 1.0f;  // obstacle handled by the caller; not a stall
        }
        motor_drive_straight();
        if (s_have_accel) {
            float m = accel_mag();
            total += fabsf(m - prev);
            prev = m;
        }
    }
    return s_have_accel ? total : 1.0f;
}

// Turn in place toward `go_left` until the lidar ahead is open (out of range
// or past LIDAR_CLEAR_MM). Compass heading is logged; timeout is the safety cap.
static void turn_toward(bool go_left)
{
    ESP_LOGI(TAG, "Turning %s until path opens", go_left ? "left" : "right");

    if (go_left) {
        motor_back(MOTOR_LEFT_CH);
        motor_fwd(MOTOR_RIGHT_CH);
        ui_set_tiles("90\xC2\xB0", "90\xC2\xB0", "Rev", "Fwd",
                     false, false, true, true);
    } else {
        motor_fwd(MOTOR_LEFT_CH);
        motor_back(MOTOR_RIGHT_CH);
        ui_set_tiles("90\xC2\xB0", "90\xC2\xB0", "Fwd", "Rev",
                     false, false, true, true);
    }

    float start_heading = s_have_mag ? mag_heading_deg() : 0.0f;
    TickType_t t0 = xTaskGetTickCount();
    const char *stop_reason = "timeout";
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(TURN_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(250));
        uint16_t dd = lidar_mm_now();
        if (dd == 0 || dd >= LIDAR_CLEAR_MM) {
            stop_reason = "path reopened";
            break;
        }
    }
    motor_stop(MOTOR_LEFT_CH);
    motor_stop(MOTOR_RIGHT_CH);

    uint32_t elapsed_ms = (xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
    float turned = s_have_mag ? heading_diff_deg(start_heading, mag_heading_deg()) : 0.0f;
    ESP_LOGI(TAG, "Turn %s done: %s (turned %.1f deg in %u ms)",
             go_left ? "left" : "right", stop_reason, turned, (unsigned int)elapsed_ms);
}

// Reactive behaviour: drive forward while the path is clear (> 10 cm). On an
// obstacle, back up, then immediately spin until the path ahead opens. Accel Y
// steers gently to keep the robot going straight.
static void behave_task(void *arg)
{
    look(90);   // center the head

    while (1) {
        uint16_t d = lidar_mm_now();

        if (d != 0 && d <= LIDAR_STOP_MM) {
            motor_stop(MOTOR_LEFT_CH);
            motor_stop(MOTOR_RIGHT_CH);
            ESP_LOGI(TAG, "Obstacle at %u mm - backing up then turning", d);

            motor_back(MOTOR_LEFT_CH);
            motor_back(MOTOR_RIGHT_CH);
            ui_set_tiles("90\xC2\xB0", "90\xC2\xB0", "Rev", "Rev",
                         false, false, true, true);
            vTaskDelay(pdMS_TO_TICKS(BACK_MS));

            // Spin in place until the lidar (still looking forward) finds an opening.
            bool go_left = s_next_turn_left;
            s_next_turn_left = !s_next_turn_left;
            ESP_LOGI(TAG, "Backup done - turning %s to find a path",
                     go_left ? "left" : "right");
            turn_toward(go_left);
            continue;
        }

        // Path clear: walk forward, steering with accel Y
        motor_drive_straight();
        ui_set_tiles("90\xC2\xB0", "90\xC2\xB0", "Fwd", "Fwd",
                     false, false, true, true);

        // Confirm we are actually rolling via the accelerometer
        float moved = measure_movement();
        uint16_t d2 = lidar_mm_now();
        if (d2 != 0 && d2 <= LIDAR_STOP_MM) {
            continue;   // obstacle appeared while driving - handle it next loop
        }
        if (s_have_accel && moved < STALL_G_THRESH) {
            ESP_LOGW(TAG, "Wheels not moving (jitter %.3f g < %.3f) - stuck?",
                     moved, (float)STALL_G_THRESH);
            motor_stop(MOTOR_LEFT_CH);
            motor_stop(MOTOR_RIGHT_CH);
            ui_set_tiles("90\xC2\xB0", "90\xC2\xB0", "STUCK", "STUCK",
                         false, false, true, true);
            vTaskDelay(pdMS_TO_TICKS(800));
        } else {
            ESP_LOGI(TAG, "Rolling: dist=%u mm jitter=%.3f g", d2, moved);
        }
    }
}

// Poll the Qwiic sensors (~10 Hz) and push a lidar hit into the map FIFO at 100 ms
static void sensor_task(void *arg)
{
    char buf[128];
    TickType_t last_map = 0;
    while (1) {
        if (s_have_lidar) {
            uint16_t mm;
            esp_err_t err = vl53l1x_read_mm(&s_lidar, &mm);
            if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) {
                s_lidar_mm = mm;  // 0 when out of range / no target
            }
            // ESP_ERR_TIMEOUT (sample not ready yet) keeps the previous value
        }
        if (s_have_accel) {
            lsm303_read_accel();
        }
        if (s_have_mag) {
            lsm303_read_mag();
        }

        char lidar_line[32];
        char acc_line[48];
        char mag_line[48];

        if (s_have_lidar && s_lidar_mm != 0) {
            snprintf(lidar_line, sizeof(lidar_line), "LIDAR  %u mm", s_lidar_mm);
        } else {
            // Missing sensor, or a valid sample with no target / out of range
            snprintf(lidar_line, sizeof(lidar_line), "LIDAR  --");
        }
        if (s_have_accel) {
            snprintf(acc_line, sizeof(acc_line), "ACC  %+.2f %+.2f %+.2f g",
                     s_acc_g[0], s_acc_g[1], s_acc_g[2]);
        } else {
            snprintf(acc_line, sizeof(acc_line), "ACC  --");
        }
        if (s_have_mag) {
            snprintf(mag_line, sizeof(mag_line), "MAG  %+.1f %+.1f %+.1f uT",
                     s_mag_ut[0], s_mag_ut[1], s_mag_ut[2]);
        } else {
            snprintf(mag_line, sizeof(mag_line), "MAG  --");
        }

        snprintf(buf, sizeof(buf), "%s\n%s\n%s", lidar_line, acc_line, mag_line);

        if (s_sensor_label && lvgl_port_lock(0)) {
            lv_label_set_text(s_sensor_label, buf);
            lvgl_port_unlock();
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - last_map) >= pdMS_TO_TICKS(MAP_SAMPLE_MS)) {
            float heading = s_have_mag ? mag_heading_deg() : 0.0f;
            float bearing = heading + (float)(s_look_deg - 90);
            map_fifo_push(lidar_mm_now(), bearing);
            map_redraw();
            last_map = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Create one status tile inside the grid
static void make_tile(lv_obj_t *parent, tile_id_t id, const char *caption,
                      int w, int h, int x, int y)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_size(panel, w, h);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_style_bg_color(panel, lv_color_make(0x20, 0x20, 0x30), LV_PART_MAIN);
    lv_obj_set_style_border_width(panel, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(panel, lv_color_make(0x40, 0x40, 0x50), LV_PART_MAIN);
    lv_obj_set_style_radius(panel, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(panel, 4, LV_PART_MAIN);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *cap = lv_label_create(panel);
    lv_label_set_text(cap, caption);
    lv_obj_set_style_text_color(cap, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(cap, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *val = lv_label_create(panel);
    lv_label_set_text(val, "--");
    lv_obj_set_style_text_color(val, lv_color_make(0xFF, 0xFF, 0x00), LV_PART_MAIN);
    lv_obj_set_style_text_font(val, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(val, LV_ALIGN_BOTTOM_MID, 0, -2);

    tile_panel[id] = panel;
    tile_value[id] = val;
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 4, LV_PART_MAIN);

    // --- Top panel: polar lidar map, ~50% of the screen ---
    const int top_h = (LCD_V_RES - 12) / 2;
    lv_obj_t *top_panel = lv_obj_create(scr);
    lv_obj_set_size(top_panel, LCD_H_RES - 8, top_h);
    lv_obj_align(top_panel, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_panel, MAP_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(top_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(top_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(top_panel, 0, LV_PART_MAIN);
    lv_obj_clear_flag(top_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_map_w = LCD_H_RES - 8;
    s_map_h = top_h;
    s_map_canvas = lv_canvas_create(top_panel);
    lv_obj_set_size(s_map_canvas, s_map_w, s_map_h);
    lv_obj_align(s_map_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_clear_flag(s_map_canvas, LV_OBJ_FLAG_SCROLLABLE);
    s_map_buf = lv_draw_buf_create((uint32_t)s_map_w, (uint32_t)s_map_h,
                                   LV_COLOR_FORMAT_RGB565, 0);
    if (s_map_buf) {
        lv_canvas_set_draw_buf(s_map_canvas, s_map_buf);
        lv_canvas_fill_bg(s_map_canvas, MAP_BG, LV_OPA_COVER);
        map_draw_compass();
    } else {
        ESP_LOGE(TAG, "polar map canvas buffer alloc failed");
        s_map_canvas = NULL;
    }

    const esp_app_desc_t *app_desc = esp_app_get_description();
    static char ver_buf[48];
    snprintf(ver_buf, sizeof(ver_buf), "FW: %s", app_desc->version);
    lv_obj_t *lbl_ver = lv_label_create(top_panel);
    lv_label_set_text(lbl_ver, ver_buf);
    lv_obj_set_style_text_color(lbl_ver, lv_color_make(0x30, 0x40, 0x50), LV_PART_MAIN);
    lv_obj_align(lbl_ver, LV_ALIGN_BOTTOM_LEFT, 2, -2);

    // --- Bottom panel with 2x2 tile grid (remaining ~50%) ---
    lv_obj_t *bottom_panel = lv_obj_create(scr);
    lv_obj_set_size(bottom_panel, LCD_H_RES - 8, LCD_V_RES - top_h - 16);
    lv_obj_align(bottom_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bottom_panel, lv_color_make(0x10, 0x10, 0x18), LV_PART_MAIN);
    lv_obj_set_style_border_width(bottom_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bottom_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bottom_panel, 6, LV_PART_MAIN);
    lv_obj_clear_flag(bottom_panel, LV_OBJ_FLAG_SCROLLABLE);

    int inner_w = LCD_H_RES - 8 - 12;
    int inner_h = LCD_V_RES - top_h - 16 - 12;
    int gap = 8;
    int strip_h = 60;                       // sensor readout strip at the bottom
    int grid_h = inner_h - strip_h - gap;   // remaining height for the 2x2 grid
    int tw = (inner_w - gap) / 2;
    int th = (grid_h - gap) / 2;

    make_tile(bottom_panel, TILE_LSERVO, "Left Servo",  tw, th, 0,         0);
    make_tile(bottom_panel, TILE_RSERVO, "Right Servo", tw, th, tw + gap,  0);
    make_tile(bottom_panel, TILE_LMOTOR, "Left Motor",  tw, th, 0,         th + gap);
    make_tile(bottom_panel, TILE_RMOTOR, "Right Motor", tw, th, tw + gap,  th + gap);

    // Sensor readout strip (lidar distance + accelerometer + magnetometer)
    lv_obj_t *strip = lv_obj_create(bottom_panel);
    lv_obj_set_size(strip, inner_w, strip_h);
    lv_obj_set_pos(strip, 0, grid_h + gap);
    lv_obj_set_style_bg_color(strip, lv_color_make(0x20, 0x20, 0x30), LV_PART_MAIN);
    lv_obj_set_style_border_width(strip, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(strip, lv_color_make(0x40, 0x40, 0x50), LV_PART_MAIN);
    lv_obj_set_style_radius(strip, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_all(strip, 4, LV_PART_MAIN);
    lv_obj_clear_flag(strip, LV_OBJ_FLAG_SCROLLABLE);

    s_sensor_label = lv_label_create(strip);
    lv_label_set_text(s_sensor_label, "LIDAR  --\nACC  --\nMAG  --");
    lv_obj_set_style_text_color(s_sensor_label, lv_color_make(0x33, 0xE1, 0xFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(s_sensor_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(s_sensor_label, LV_ALIGN_LEFT_MID, 0, 0);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();

    ESP_LOGI(TAG, "Initializing Motoron motor controller...");
    motoron_init();

    ESP_LOGI(TAG, "Initializing Qwiic sensors...");
    sensors_init();

    ESP_LOGI(TAG, "Initializing display...");

    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(PIN_LCD_BL, 1);

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = PIN_LCD_SCLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_DRAW_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLK,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 34, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Display initialized. Setting up LVGL...");

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_DRAW_LINES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        },
    };
    lv_display_t *disp = lvgl_port_add_disp(&disp_cfg);
    (void)disp;

    ESP_LOGI(TAG, "LVGL initialized. Creating UI...");

    if (lvgl_port_lock(0)) {
        create_ui();
        lvgl_port_unlock();
    }

    ESP_LOGI(TAG, "UI ready. Starting reactive behaviour...");
    xTaskCreate(sensor_task, "sensors", 4096, NULL, 5, NULL);
    xTaskCreate(behave_task, "behave", 4096, NULL, 4, NULL);
}
