// Minimal VL53L1X Time-of-Flight driver (ST ULD default configuration).
// Only what Little Walker needs: bring the sensor up and read distance in mm.
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#define VL53L1X_I2C_ADDR 0x29

typedef struct {
    i2c_master_dev_handle_t dev;
} vl53l1x_t;

// Add the sensor to an existing I2C master bus and run the ULD init sequence.
esp_err_t vl53l1x_init(i2c_master_bus_handle_t bus, vl53l1x_t *sensor);

// Read the latest ranging result in millimetres. Returns ESP_ERR_TIMEOUT if no
// fresh sample was ready in time; *mm is left untouched in that case.
esp_err_t vl53l1x_read_mm(vl53l1x_t *sensor, uint16_t *mm);
