// Minimal VL53L1X Time-of-Flight driver (ST ULD default configuration).
// Only what Little Walker needs: bring the sensor up and read distance in mm.
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define VL53L1X_I2C_ADDR 0x29

// ST / Adafruit distance modes. Short is more immune to ambient light but
// tops out around 1360 mm. Long reaches about 3600 mm (covers 0-3000 mm).
typedef enum {
    VL53L1X_DISTANCE_SHORT = 1,
    VL53L1X_DISTANCE_LONG  = 2,
} vl53l1x_distance_mode_t;

typedef struct {
    i2c_master_dev_handle_t dev;
} vl53l1x_t;

// Add the sensor to an existing I2C master bus and run the ULD init sequence
// (long distance mode, up to ~3600 mm).
esp_err_t vl53l1x_init(i2c_master_bus_handle_t bus, vl53l1x_t *sensor);

// Switch ranging profile. Call while ranging is stopped, or during init.
esp_err_t vl53l1x_set_distance_mode(vl53l1x_t *sensor, vl53l1x_distance_mode_t mode);

// Read the latest ranging result in millimetres.
// ESP_OK: valid range, *mm is the distance.
// ESP_ERR_NOT_FOUND: a sample was produced but it is not a valid range
//   (no target / out of range / wrap-around). *mm is set to 0.
// ESP_ERR_TIMEOUT: no fresh sample was ready; *mm is left untouched.
esp_err_t vl53l1x_read_mm(vl53l1x_t *sensor, uint16_t *mm);
