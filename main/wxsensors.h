/* Dual use Weather and Splot Phase Energy Meter

Intended to be used with CircuitSetup.us Split Phase Energy Meter and some extra tacked on sensors.

The MIT License (MIT)
Copyright (c) 2019 npotts
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.  
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <string.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define MPL_ADDR       0x60 // Unshifted 7-bit I2C address for sensor
#define MPL_STATUS     0x00
#define MPL_OUT_P_MSB  0x01
#define MPL_OUT_P_CSB  0x02
#define MPL_OUT_P_LSB  0x03
#define MPL_OUT_T_MSB  0x04
#define MPL_OUT_T_LSB  0x05
#define MPL_DR_STATUS  0x06
#define MPL_OUT_P_DELTA_MSB  0x07
#define MPL_OUT_P_DELTA_CSB  0x08
#define MPL_OUT_P_DELTA_LSB  0x09
#define MPL_OUT_T_DELTA_MSB  0x0A
#define MPL_OUT_T_DELTA_LSB  0x0B
#define MPL_WHO_AM_I   0x0C
#define MPL_F_STATUS   0x0D
#define MPL_F_DATA     0x0E
#define MPL_F_SETUP    0x0F
#define MPL_TIME_DLY   0x10
#define MPL_SYSMOD     0x11
#define MPL_INT_SOURCE 0x12
#define MPL_PT_DATA_CFG 0x13
#define MPL_BAR_IN_MSB 0x14
#define MPL_BAR_IN_LSB 0x15
#define MPL_P_TGT_MSB  0x16
#define MPL_P_TGT_LSB  0x17
#define MPL_T_TGT      0x18
#define MPL_P_WND_MSB  0x19
#define MPL_P_WND_LSB  0x1A
#define MPL_T_WND      0x1B
#define MPL_P_MIN_MSB  0x1C
#define MPL_P_MIN_CSB  0x1D
#define MPL_P_MIN_LSB  0x1E
#define MPL_T_MIN_MSB  0x1F
#define MPL_T_MIN_LSB  0x20
#define MPL_P_MAX_MSB  0x21
#define MPL_P_MAX_CSB  0x22
#define MPL_P_MAX_LSB  0x23
#define MPL_T_MAX_MSB  0x24
#define MPL_T_MAX_LSB  0x25
#define MPL_CTRL_REG1  0x26
#define MPL_CTRL_REG2  0x27
#define MPL_CTRL_REG3  0x28
#define MPL_CTRL_REG4  0x29
#define MPL_CTRL_REG5  0x2A
#define MPL_OFF_P      0x2B
#define MPL_OFF_T      0x2C
#define MPL_OFF_H      0x2D



#define SI7021_ADDR	0x40
#define SI7021_MEASRH_HOLD_CMD           0xE5 /**< Measure Relative Humidity, Hold Master Mode */
#define SI7021_MEASRH_NOHOLD_CMD         0xF5 /**< Measure Relative Humidity, No Hold Master Mode */
#define SI7021_MEASTEMP_HOLD_CMD         0xE3 /**< Measure Temperature, Hold Master Mode */
#define SI7021_MEASTEMP_NOHOLD_CMD       0xF3 /**< Measure Temperature, No Hold Master Mode */
#define SI7021_READPREVTEMP_CMD          0xE0 /**< Read Temperature Value from Previous RH Measurement */
#define SI7021_RESET_CMD                 0xFE /**< Reset Command */
#define SI7021_WRITERHT_REG_CMD          0xE6 /**< Write RH/T User Register 1 */
#define SI7021_READRHT_REG_CMD           0xE7 /**< Read RH/T User Register 1 */
#define SI7021_WRITEHEATER_REG_CMD       0x51 /**< Write Heater Control Register */
#define SI7021_READHEATER_REG_CMD        0x11 /**< Read Heater Control Register */
#define SI7021_ID1_CMD                   0xFA0F /**< Read Electronic ID 1st Byte */
#define SI7021_ID2_CMD                   0xFCC9 /**< Read Electronic ID 2nd Byte */
#define SI7021_FIRMVERS_CMD              0x84B8 /**< Read Firmware Revision */

#define SI7021_REV_1					0xff  /**< Sensor revision 1 */
#define SI7021_REV_2					0x20  /**< Sensor revision 2 */

esp_err_t init_i2c_bus(const i2c_config_t *cfg);
esp_err_t i2c_probe(uint8_t addr);
esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len);
esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
esp_err_t i2c_write_data(uint8_t addr, void *data, uint8_t len);
esp_err_t i2c_read_data(uint8_t addr, void *data, uint8_t len);
void wxstation_init(void);
