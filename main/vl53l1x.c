#include "vl53l1x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "vl53l1x";

// VL53L1X registers (16-bit addresses, big-endian on the wire)
#define REG_SOFT_RESET              0x0000
#define REG_MODEL_ID               0x010F
#define REG_VHV_CONFIG_TIMEOUT     0x0008
#define REG_GPIO_HV_STATUS         0x0031
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0086
#define REG_SYSTEM_MODE_START      0x0087
#define REG_RESULT_RANGE_STATUS    0x0089
#define REG_RESULT_DISTANCE        0x0096
#define REG_FIRMWARE_SYSTEM_STATUS 0x00E5

#define MODEL_ID_EXPECTED          0xEACC
#define I2C_TIMEOUT_MS             100

// ST ULD default configuration, written in one block starting at 0x002D.
// The final byte (register 0x0087 = 0x40) starts ranging.
static const uint8_t VL53L1X_DEFAULT_CONFIG[] = {
    0x00, /* 0x2d */ 0x00, /* 0x2e */ 0x00, /* 0x2f */ 0x01, /* 0x30 */
    0x02, /* 0x31 */ 0x00, /* 0x32 */ 0x02, /* 0x33 */ 0x08, /* 0x34 */
    0x00, /* 0x35 */ 0x08, /* 0x36 */ 0x10, /* 0x37 */ 0x01, /* 0x38 */
    0x01, /* 0x39 */ 0x00, /* 0x3a */ 0x00, /* 0x3b */ 0x00, /* 0x3c */
    0x00, /* 0x3d */ 0xff, /* 0x3e */ 0x00, /* 0x3f */ 0x0f, /* 0x40 */
    0x00, /* 0x41 */ 0x00, /* 0x42 */ 0x00, /* 0x43 */ 0x00, /* 0x44 */
    0x00, /* 0x45 */ 0x20, /* 0x46 */ 0x0b, /* 0x47 */ 0x00, /* 0x48 */
    0x00, /* 0x49 */ 0x02, /* 0x4a */ 0x0a, /* 0x4b */ 0x21, /* 0x4c */
    0x00, /* 0x4d */ 0x00, /* 0x4e */ 0x05, /* 0x4f */ 0x00, /* 0x50 */
    0x00, /* 0x51 */ 0x00, /* 0x52 */ 0x00, /* 0x53 */ 0xc8, /* 0x54 */
    0x00, /* 0x55 */ 0x00, /* 0x56 */ 0x38, /* 0x57 */ 0xff, /* 0x58 */
    0x01, /* 0x59 */ 0x00, /* 0x5a */ 0x08, /* 0x5b */ 0x00, /* 0x5c */
    0x00, /* 0x5d */ 0x01, /* 0x5e */ 0xcc, /* 0x5f */ 0x0f, /* 0x60 */
    0x01, /* 0x61 */ 0xf1, /* 0x62 */ 0x0d, /* 0x63 */ 0x01, /* 0x64 */
    0x68, /* 0x65 */ 0x00, /* 0x66 */ 0x80, /* 0x67 */ 0x08, /* 0x68 */
    0xb8, /* 0x69 */ 0x00, /* 0x6a */ 0x00, /* 0x6b */ 0x00, /* 0x6c */
    0x00, /* 0x6d */ 0x0f, /* 0x6e */ 0x89, /* 0x6f */ 0x00, /* 0x70 */
    0x00, /* 0x71 */ 0x00, /* 0x72 */ 0x00, /* 0x73 */ 0x00, /* 0x74 */
    0x00, /* 0x75 */ 0x00, /* 0x76 */ 0x01, /* 0x77 */ 0x0f, /* 0x78 */
    0x0d, /* 0x79 */ 0x0e, /* 0x7a */ 0x0e, /* 0x7b */ 0x00, /* 0x7c */
    0x00, /* 0x7d */ 0x02, /* 0x7e */ 0xc7, /* 0x7f */ 0xff, /* 0x80 */
    0x9b, /* 0x81 */ 0x00, /* 0x82 */ 0x00, /* 0x83 */ 0x00, /* 0x84 */
    0x01, /* 0x85 */ 0x00, /* 0x86 */ 0x40, /* 0x87 */
};

static esp_err_t reg_write(vl53l1x_t *s, uint16_t reg, const uint8_t *data, size_t n)
{
    uint8_t buf[2 + 91];
    if (n > sizeof(buf) - 2) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    for (size_t i = 0; i < n; i++) {
        buf[2 + i] = data[i];
    }
    return i2c_master_transmit(s->dev, buf, n + 2, I2C_TIMEOUT_MS);
}

static esp_err_t reg_write8(vl53l1x_t *s, uint16_t reg, uint8_t val)
{
    return reg_write(s, reg, &val, 1);
}

static esp_err_t reg_read(vl53l1x_t *s, uint16_t reg, uint8_t *data, size_t n)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    return i2c_master_transmit_receive(s->dev, addr, 2, data, n, I2C_TIMEOUT_MS);
}

static esp_err_t reg_read8(vl53l1x_t *s, uint16_t reg, uint8_t *val)
{
    return reg_read(s, reg, val, 1);
}

static esp_err_t reg_read16(vl53l1x_t *s, uint16_t reg, uint16_t *val)
{
    uint8_t b[2];
    esp_err_t err = reg_read(s, reg, b, 2);
    if (err == ESP_OK) {
        *val = ((uint16_t)b[0] << 8) | b[1];
    }
    return err;
}

// Data-ready when the interrupt line reflects the (active-high) polarity that
// the default configuration selects.
static bool data_ready(vl53l1x_t *s)
{
    uint8_t status = 0;
    if (reg_read8(s, REG_GPIO_HV_STATUS, &status) != ESP_OK) {
        return false;
    }
    return (status & 0x01) == 1;
}

static esp_err_t wait_data_ready(vl53l1x_t *s, int timeout_ms)
{
    for (int waited = 0; waited < timeout_ms; waited += 5) {
        if (data_ready(s)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t vl53l1x_init(i2c_master_bus_handle_t bus, vl53l1x_t *sensor)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L1X_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &sensor->dev);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t model = 0;
    err = reg_read16(sensor, REG_MODEL_ID, &model);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no response at 0x%02X (%s)", VL53L1X_I2C_ADDR,
                 esp_err_to_name(err));
        return err;
    }
    if (model != MODEL_ID_EXPECTED) {
        ESP_LOGW(TAG, "unexpected model id 0x%04X (expected 0x%04X)",
                 model, MODEL_ID_EXPECTED);
    }

    // Wait for firmware boot
    for (int waited = 0; waited < 100; waited += 5) {
        uint8_t sys = 0;
        if (reg_read8(sensor, REG_FIRMWARE_SYSTEM_STATUS, &sys) == ESP_OK &&
            (sys & 0x01)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Load the ULD default configuration (also starts ranging via 0x87=0x40)
    err = reg_write(sensor, 0x002D, VL53L1X_DEFAULT_CONFIG,
                    sizeof(VL53L1X_DEFAULT_CONFIG));
    if (err != ESP_OK) {
        return err;
    }

    // One-shot boot ranging + calibration dance, per ST ULD SensorInit
    if (wait_data_ready(sensor, 100) == ESP_OK) {
        reg_write8(sensor, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    }
    reg_write8(sensor, REG_SYSTEM_MODE_START, 0x00);   // stop ranging
    reg_write8(sensor, REG_VHV_CONFIG_TIMEOUT, 0x09);
    reg_write8(sensor, 0x000B, 0x00);
    err = reg_write8(sensor, REG_SYSTEM_MODE_START, 0x40);  // start ranging
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "VL53L1X ranging started at 0x%02X", VL53L1X_I2C_ADDR);
    return ESP_OK;
}

esp_err_t vl53l1x_read_mm(vl53l1x_t *sensor, uint16_t *mm)
{
    if (!data_ready(sensor)) {
        return ESP_ERR_TIMEOUT;
    }

    uint16_t distance = 0;
    esp_err_t err = reg_read16(sensor, REG_RESULT_DISTANCE, &distance);

    // Clear the interrupt so the next sample can be produced
    reg_write8(sensor, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (err == ESP_OK) {
        *mm = distance;
    }
    return err;
}
