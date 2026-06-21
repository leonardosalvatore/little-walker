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
#define MOTOR_LEFT_CH   1     // Motoron motor channel 1
#define MOTOR_RIGHT_CH  2     // Motoron motor channel 2
#define MOTOR_SPEED     400   // 0..800 (half power for a gentle demo)

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
            ESP_LOGI(TAG, "  found I2C device at 0x%02X (%d)", addr, addr);
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
    uint8_t cmd[4] = {
        0xD2,
        (uint8_t)(motor & 0x7F),
        (uint8_t)(speed & 0x7F),
        (uint8_t)((speed >> 7) & 0x7F),
    };
    motoron_send(cmd, sizeof(cmd));
}

static inline void motor_fwd(uint8_t motor)  { motoron_set_speed(motor, MOTOR_SPEED); }
static inline void motor_stop(uint8_t motor) { motoron_set_speed(motor, 0); }
static inline void motor_back(uint8_t motor) { motoron_set_speed(motor, -MOTOR_SPEED); }

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

#define STEP_MS 800

static void demo_delay(void)
{
    vTaskDelay(pdMS_TO_TICKS(STEP_MS));
}

// Update one tile (active) and reset the others to idle, holding their last text
static void show_active(tile_id_t active, const char *text)
{
    static const char *last[TILE_COUNT] = {"--", "--", "--", "--"};
    last[active] = text;
    if (lvgl_port_lock(0)) {
        for (int i = 0; i < TILE_COUNT; i++) {
            set_tile((tile_id_t)i, last[i], i == active);
        }
        lvgl_port_unlock();
    }
}

// --- Robot face (two eyes) ---
typedef enum {
    FACE_SLEEP,
    FACE_SURPRISE,
    FACE_IDLE,
    FACE_SAD,
    FACE_HAPPY,
    FACE_FURIOUS,
} face_expr_t;

#define EYE_W 48
#define EYE_H 62
#define EYE_SLANT 240   // slant angle for sad/furious eyes (0.1 deg units)
#define FACE_BG lv_color_make(0x05, 0x07, 0x0D)

// Each eye has two background-coloured eyelids (top + bottom) that set_face()
// reshapes to carve every expression.
static lv_obj_t *s_eye_lid_top[2];
static lv_obj_t *s_eye_lid_bot[2];
static lv_obj_t *s_zzz = NULL;   // "z z z" shown only while sleeping
static lv_obj_t *s_ahah = NULL;  // "Ahah" shown only while happy
static lv_obj_t *s_grrr = NULL;  // "Grrr!" shown only while happy

// Position/size one lid; opa == TRANSP hides it.
static void set_lid(lv_obj_t *lid, lv_opa_t opa, int w, int h, int x, int y, int rot)
{
    if (!lid) {
        return;
    }
    lv_obj_set_style_opa(lid, opa, LV_PART_MAIN);
    lv_obj_set_size(lid, w, h);
    lv_obj_set_pos(lid, x, y);
    lv_obj_set_style_transform_pivot_x(lid, w / 2, LV_PART_MAIN);
    lv_obj_set_style_transform_pivot_y(lid, h / 2, LV_PART_MAIN);
    lv_obj_set_style_transform_rotation(lid, rot, LV_PART_MAIN);
}

// Carve each expression by sizing/positioning the eyelids over each eye:
//   surprise -> both lids hidden (full open eye)
//   idle     -> top + bottom lids leave a short, centred band (+ pupil)
//   sleep    -> top lid covers all but a thin bottom slit
//   happy    -> bottom lid covers the bottom, leaving an upward dome
//   sad      -> slanted top lid, inner corner low
//   furious  -> slanted top lid, inner corner high
static void set_face(face_expr_t e)
{
    for (int i = 0; i < 2; i++) {
        lv_obj_t *top = s_eye_lid_top[i];
        lv_obj_t *bot = s_eye_lid_bot[i];

        // Defaults: both lids hidden (used by "surprise")
        set_lid(top, LV_OPA_TRANSP, EYE_W, EYE_H, 0, 0, 0);
        set_lid(bot, LV_OPA_TRANSP, EYE_W, EYE_H, 0, 0, 0);

        switch (e) {
        case FACE_SURPRISE:             // full open eye
            break;
        case FACE_IDLE:                 // short centred band + pupil
            set_lid(top, LV_OPA_COVER, EYE_W, 18, 0, 0, 0);
            set_lid(bot, LV_OPA_COVER, EYE_W, 18, 0, EYE_H - 18, 0);
            break;
        case FACE_SLEEP:                // cover all but a thin bottom slit
            set_lid(top, LV_OPA_COVER, EYE_W, EYE_H - 8, 0, 0, 0);
            break;
        case FACE_HAPPY:                // cover bottom -> upward dome
            set_lid(bot, LV_OPA_COVER, EYE_W, 40, 0, EYE_H - 40, 0);
            break;
        case FACE_SAD:                  // eyes slant "/ \" (inner corners high)
            set_lid(top, LV_OPA_COVER, EYE_W + 80, 90, (EYE_W - (EYE_W + 80)) / 2,
                    30 - 90, (i == 0) ? -EYE_SLANT : EYE_SLANT);
            break;
        case FACE_FURIOUS:              // eyes slant "\ /" (inner corners low)
            set_lid(top, LV_OPA_COVER, EYE_W + 80, 90, (EYE_W - (EYE_W + 80)) / 2,
                    30 - 90, (i == 0) ? EYE_SLANT : -EYE_SLANT);
            break;
        }
    }
    if (s_zzz) {
        lv_obj_set_style_opa(s_zzz,
            e == FACE_SLEEP ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    }
    if (s_ahah) {
        lv_obj_set_style_opa(s_ahah,
            e == FACE_HAPPY ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    }
    if (s_grrr) {
        lv_obj_set_style_opa(s_grrr,
            e == FACE_FURIOUS ? LV_OPA_COVER : LV_OPA_TRANSP, LV_PART_MAIN);
    }
}

static const char *face_name(face_expr_t e)
{
    switch (e) {
    case FACE_SLEEP:   return "sleep";
    case FACE_SURPRISE:return "surprise";
    case FACE_IDLE:    return "idle";
    case FACE_SAD:     return "sad";
    case FACE_HAPPY:   return "happy";
    case FACE_FURIOUS: return "furious";
    default:           return "?";
    }
}

static void show_face(face_expr_t e)
{
    ESP_LOGI(TAG, "Expression: %s", face_name(e));
    if (lvgl_port_lock(0)) {
        set_face(e);
        lvgl_port_unlock();
    }
}

static void demo_task(void *arg)
{
    while (1) {
        // Resting beats
        show_face(FACE_IDLE);
        demo_delay();
        show_face(FACE_SURPRISE);
        demo_delay();

        // Left servo: 0 -> 90 -> 180
        show_face(FACE_SLEEP);
        const int angles[] = {0, 90, 180};
        for (int i = 0; i < 3; i++) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d\xC2\xB0", angles[i]);
            servo_set_angle(LEDC_CHANNEL_0, angles[i]);
            ESP_LOGI(TAG, "Left servo: %d", angles[i]);
            show_active(TILE_LSERVO, buf);
            demo_delay();
        }

        // Left motor: Fwd -> Stop -> Back -> Stop
        show_face(FACE_FURIOUS);
        motor_fwd(MOTOR_LEFT_CH);  show_active(TILE_LMOTOR, "Fwd");  demo_delay();
        motor_stop(MOTOR_LEFT_CH); show_active(TILE_LMOTOR, "Stop"); demo_delay();
        motor_back(MOTOR_LEFT_CH); show_active(TILE_LMOTOR, "Back"); demo_delay();
        motor_stop(MOTOR_LEFT_CH); show_active(TILE_LMOTOR, "Stop"); demo_delay();

        // Right servo: 0 -> 90 -> 180
        show_face(FACE_SAD);
        for (int i = 0; i < 3; i++) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d\xC2\xB0", angles[i]);
            servo_set_angle(LEDC_CHANNEL_1, angles[i]);
            ESP_LOGI(TAG, "Right servo: %d", angles[i]);
            show_active(TILE_RSERVO, buf);
            demo_delay();
        }

        // Right motor: Fwd -> Stop -> Back -> Stop
        show_face(FACE_HAPPY);
        motor_fwd(MOTOR_RIGHT_CH);  show_active(TILE_RMOTOR, "Fwd");  demo_delay();
        motor_stop(MOTOR_RIGHT_CH); show_active(TILE_RMOTOR, "Stop"); demo_delay();
        motor_back(MOTOR_RIGHT_CH); show_active(TILE_RMOTOR, "Back"); demo_delay();
        motor_stop(MOTOR_RIGHT_CH); show_active(TILE_RMOTOR, "Stop"); demo_delay();
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
    lv_obj_align(val, LV_ALIGN_CENTER, 0, 8);

    tile_panel[id] = panel;
    tile_value[id] = val;
}

// Create one eye (bright rounded rect) with a background-coloured eyelid
// overlay that set_face() reshapes to make expressions.
static void make_eye(lv_obj_t *parent, int idx, int x_off)
{
    lv_obj_t *eye = lv_obj_create(parent);
    lv_obj_set_size(eye, EYE_W, EYE_H);
    lv_obj_align(eye, LV_ALIGN_CENTER, x_off, -6);
    lv_obj_set_style_bg_color(eye, lv_color_make(0x33, 0xE1, 0xFF), LV_PART_MAIN);
    lv_obj_set_style_border_width(eye, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(eye, 18, LV_PART_MAIN);
    lv_obj_set_style_pad_all(eye, 0, LV_PART_MAIN);
    lv_obj_set_style_clip_corner(eye, true, LV_PART_MAIN);
    lv_obj_clear_flag(eye, LV_OBJ_FLAG_SCROLLABLE);

    // Tiny black round pupil in the middle of the eye
    lv_obj_t *pupil = lv_obj_create(eye);
    lv_obj_set_size(pupil, 12, 12);
    lv_obj_center(pupil);
    lv_obj_set_style_bg_color(pupil, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_border_width(pupil, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(pupil, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_pad_all(pupil, 0, LV_PART_MAIN);
    lv_obj_clear_flag(pupil, LV_OBJ_FLAG_SCROLLABLE);

    // Two eyelids (top + bottom) drawn over the pupil; reshaped by set_face()
    for (int k = 0; k < 2; k++) {
        lv_obj_t *lid = lv_obj_create(eye);
        lv_obj_set_size(lid, 0, 0);
        lv_obj_set_pos(lid, 0, 0);
        lv_obj_set_style_bg_color(lid, FACE_BG, LV_PART_MAIN);
        lv_obj_set_style_border_width(lid, 0, LV_PART_MAIN);
        lv_obj_set_style_radius(lid, 16, LV_PART_MAIN);
        lv_obj_set_style_pad_all(lid, 0, LV_PART_MAIN);
        lv_obj_clear_flag(lid, LV_OBJ_FLAG_SCROLLABLE);
        if (k == 0) {
            s_eye_lid_top[idx] = lid;
        } else {
            s_eye_lid_bot[idx] = lid;
        }
    }
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 4, LV_PART_MAIN);

    // --- Top panel: robot face (two eyes), ~50% of the screen ---
    const int top_h = (LCD_V_RES - 12) / 2;
    lv_obj_t *top_panel = lv_obj_create(scr);
    lv_obj_set_size(top_panel, LCD_H_RES - 8, top_h);
    lv_obj_align(top_panel, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_panel, FACE_BG, LV_PART_MAIN);
    lv_obj_set_style_border_width(top_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(top_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(top_panel, 6, LV_PART_MAIN);
    lv_obj_clear_flag(top_panel, LV_OBJ_FLAG_SCROLLABLE);

    // Two eyes spaced symmetrically around the centre
    make_eye(top_panel, 0, -(EYE_W / 2 + 14));
    make_eye(top_panel, 1,  (EYE_W / 2 + 14));

    // "z Z z" shown only while sleeping
    s_zzz = lv_label_create(top_panel);
    lv_label_set_text(s_zzz, "z Z z");
    lv_obj_set_style_text_color(s_zzz, lv_color_make(0x60, 0x90, 0xC0), LV_PART_MAIN);
    lv_obj_align(s_zzz, LV_ALIGN_TOP_RIGHT, -6, 2);
    lv_obj_set_style_opa(s_zzz, LV_OPA_TRANSP, LV_PART_MAIN);

    // "Ahah" shown only while happy
    s_ahah = lv_label_create(top_panel);
    lv_label_set_text(s_ahah, "Ahah");
    lv_obj_set_style_text_color(s_ahah, lv_color_make(0x33, 0xE1, 0xFF), LV_PART_MAIN);
    lv_obj_align(s_ahah, LV_ALIGN_TOP_LEFT, -6, 2);
    lv_obj_set_style_opa(s_ahah, LV_OPA_TRANSP, LV_PART_MAIN);

    // "Grrr!" shown only while happy
    s_grrr = lv_label_create(top_panel);
    lv_label_set_text(s_grrr, "Grrr!");
    lv_obj_set_style_text_color(s_grrr, lv_color_make(0x33, 0xE1, 0xFF), LV_PART_MAIN);
    lv_obj_align(s_grrr, LV_ALIGN_TOP_RIGHT, -6, 2);
    lv_obj_set_style_opa(s_grrr, LV_OPA_TRANSP, LV_PART_MAIN);

    // Tiny, dim firmware version in the corner
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
    int tw = (inner_w - gap) / 2;
    int th = (inner_h - gap) / 2;

    make_tile(bottom_panel, TILE_LSERVO, "Left Servo",  tw, th, 0,         0);
    make_tile(bottom_panel, TILE_RSERVO, "Right Servo", tw, th, tw + gap,  0);
    make_tile(bottom_panel, TILE_LMOTOR, "Left Motor",  tw, th, 0,         th + gap);
    make_tile(bottom_panel, TILE_RMOTOR, "Right Motor", tw, th, tw + gap,  th + gap);

    set_face(FACE_IDLE);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing servos...");
    servo_init();

    ESP_LOGI(TAG, "Initializing Motoron motor controller...");
    motoron_init();

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
            .mirror_x = false,
            .mirror_y = false,
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

    ESP_LOGI(TAG, "UI ready. Starting motor demo...");
    xTaskCreate(demo_task, "demo", 3072, NULL, 5, NULL);
}
