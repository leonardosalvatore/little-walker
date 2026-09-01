#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
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

// Display parameters
#define LCD_H_RES       172
#define LCD_V_RES       320
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_PIXEL_CLK   (40 * 1000 * 1000)
#define LCD_DRAW_LINES  50

// Button event callback
static void btn_event_cb(lv_event_t *e)
{
    const char *label = lv_label_get_text(lv_obj_get_child(lv_event_get_target(e), 0));
    ESP_LOGI(TAG, "Button pressed: %s", label);
}

static void create_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_pad_all(scr, 4, LV_PART_MAIN);

    // --- Top panel (20%) ---
    lv_obj_t *top_panel = lv_obj_create(scr);
    lv_obj_set_size(top_panel, LCD_H_RES - 8, 60);
    lv_obj_align(top_panel, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(top_panel, lv_color_make(0x20, 0x20, 0x30), LV_PART_MAIN);
    lv_obj_set_style_border_width(top_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(top_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(top_panel, 6, LV_PART_MAIN);
    lv_obj_set_flex_flow(top_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(top_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(top_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl_hello = lv_label_create(top_panel);
    lv_label_set_text(lbl_hello, "Hello =]");
    lv_obj_set_style_text_color(lbl_hello, lv_color_make(0x00, 0xFF, 0x80), LV_PART_MAIN);
    lv_obj_set_style_text_font(lbl_hello, &lv_font_montserrat_14, LV_PART_MAIN);

    const esp_app_desc_t *app_desc = esp_app_get_description();
    static char ver_buf[32];
    snprintf(ver_buf, sizeof(ver_buf), "FW: %.8s", app_desc->app_elf_sha256);

    lv_obj_t *lbl_ver = lv_label_create(top_panel);
    lv_label_set_text(lbl_ver, ver_buf);
    lv_obj_set_style_text_color(lbl_ver, lv_color_make(0x80, 0x80, 0x80), LV_PART_MAIN);

    // --- Bottom panel (80%) with 6 buttons ---
    lv_obj_t *bot_panel = lv_obj_create(scr);
    lv_obj_set_size(bot_panel, LCD_H_RES - 8, LCD_V_RES - 60 - 16);
    lv_obj_align(bot_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(bot_panel, lv_color_make(0x10, 0x10, 0x18), LV_PART_MAIN);
    lv_obj_set_style_border_width(bot_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(bot_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bot_panel, 6, LV_PART_MAIN);
    lv_obj_set_style_pad_gap(bot_panel, 6, LV_PART_MAIN);
    lv_obj_clear_flag(bot_panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *hdr_left = lv_label_create(bot_panel);
    lv_label_set_text(hdr_left, "LEFT");
    lv_obj_set_style_text_color(hdr_left, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(hdr_left, LV_ALIGN_TOP_LEFT, 16, 0);

    lv_obj_t *hdr_right = lv_label_create(bot_panel);
    lv_label_set_text(hdr_right, "RIGHT");
    lv_obj_set_style_text_color(hdr_right, lv_color_white(), LV_PART_MAIN);
    lv_obj_align(hdr_right, LV_ALIGN_TOP_RIGHT, -12, 0);

    static const char *btn_labels[3][2] = {
        {"L FWD",  "R FWD"},
        {"L STOP", "R STOP"},
        {"L BWD",  "R BWD"},
    };
    lv_color_t btn_colors[3] = {
        lv_color_make(0x40, 0xC0, 0x40),
        lv_color_make(0xE0, 0x80, 0x40),
        lv_color_make(0x40, 0x40, 0xE0),
    };

    int btn_w = (LCD_H_RES - 8 - 12 - 8) / 2;
    int btn_h = (LCD_V_RES - 60 - 16 - 12 - 24 - 6 * 2) / 3;
    int y_start = 22;

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 2; col++) {
            lv_obj_t *btn = lv_button_create(bot_panel);
            lv_obj_set_size(btn, btn_w, btn_h);
            lv_obj_set_pos(btn, col * (btn_w + 8), y_start + row * (btn_h + 6));
            lv_obj_set_style_bg_color(btn, btn_colors[row], LV_PART_MAIN);
            lv_obj_set_style_radius(btn, 8, LV_PART_MAIN);
            lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

            lv_obj_t *lbl = lv_label_create(btn);
            lv_label_set_text(lbl, btn_labels[row][col]);
            lv_obj_center(lbl);
        }
    }
}

void app_main(void)
{
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

    ESP_LOGI(TAG, "UI ready.");
}
