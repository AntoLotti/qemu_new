/*
 * LIS3DH 
 *
 * Copyright (c) 2025 Antonio Lotti Villar (antoniolottivillar@gmail.com)
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef INC_LIS3DH_TYPES_H_
#define INC_LIS3DH_TYPES_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* ==================== ENUMERATIONS ======================= */

/**
 * @brief LIS3DH Operating Modes
 *
 * The different operating modes that the LIS3DH can be configured with.
 * After the boot is completed, the device is automatically configured in power-down mode.
 *
 * Datasheet Pg.16/54
 * Application Notes Pg.9/59
 *
 * Operating mode 		CTRL_REG1[3]	CTRL_REG4[3]
 * 						(LPen bit) 		 (HR bit)
 *
 * Low-power mode
 * (8-bit data output)		1 				0
 *
 * Normal mode
 * (10-bit data output)     0 				0
 *
 * High-resolution mode
 * (12-bit data output) 	0 				1
 *
 * Not allowed 				1 				1
 *
 */
typedef enum lis3dh_mode_e
{
    LIS3DH_MODE_NORMAL      = 0,
    LIS3DH_MODE_HIGH_RES    = 1,
    LIS3DH_MODE_LOW_POWER   = 2,
    LIS3DH_MODE_NOT_ALLOWED = 3,
}lis3dh_mode_t;

/**
 * @brief Data Rate Selection Configuration Modes
 * 
 * Datasheet Pg.35/54
 *
 * ODR3		ODR2	ODR1	ODR0	Power mode selection
 *
 * 0 		0 	  	0 		0 		Power-down mode
 * 0 		0 		0 		1 		HR / Normal / Low-power mode (1 Hz)
 * 0 		0 		1 		0 		HR / Normal / Low-power mode (10 Hz)
 * 0 		0 		1 		1 		HR / Normal / Low-power mode (25 Hz)
 * 0 		1 		0 		0 		HR / Normal / Low-power mode (50 Hz)
 * 0 		1 		0 		1 		HR / Normal / Low-power mode (100 Hz)
 * 0 		1 		1 		0 		HR / Normal / Low-power mode (200 Hz)
 * 0 		1 		1 		1 		HR / Normal / Low-power mode (400 Hz)
 * 1 		0 		0 		0 		Low power mode (1.60 kHz)
 * 1 		0 		0 		1 		HR / normal (1.344 kHz); Low-power mode (5.376 kHz)
 *
 */
typedef enum lis3dh_odr_e
{
    LIS3DH_ODR_POWER_DOWN   = 0,
    LIS3DH_ODR_1            = 1,
    LIS3DH_ODR_10           = 2,
    LIS3DH_ODR_25           = 3,
    LIS3DH_ODR_50           = 4,
    LIS3DH_ODR_100          = 5,
    LIS3DH_ODR_200          = 6,
    LIS3DH_ODR_400          = 7,
    LIS3DH_ODR_1600         = 8,
    LIS3DH_ODR_1250         = 9,
}lis3dh_odr_t;

/**
 * @brief Full Scale Modes
 *
 * This type refers to the maximum and minimum acceleration values ​​that the sensor can detect.
 * It's defined in the FS[1:0] bits of the CTRL_REG4 register
 *
 * Datasheet Pg.37/54
 *
 * FS1		FS0		Mode
 * 0		0 		±2g
 * 0 		1 		±4g
 * 1 		0		±10g
 * 1 		1		±16g
 */
typedef enum lis3dh_fscale_e
{
    LIS3DH_FS_2G    = 0,
    LIS3DH_FS_4G    = 1,
    LIS3DH_FS_8G    = 2,
    LIS3DH_FS_16G   = 3
}lis3dh_fscale_t;


/* ==================== DATA STRUCTURES ==================== */

/**
 * @struct lis3dh_config_t
 * @brief Internal Peripheral Configuration
 */
typedef struct lis3dh_config_s
{
    lis3dh_fscale_t fscale; /**< Full Scale Range (Measurement range)   */
    lis3dh_mode_t   mode;   /**< Operating Mode     */
    lis3dh_odr_t    odr;    /**< Output Data Rate   */
    bool temp_enable;       /**< Enable Temperature Sensor */
    bool high_pass_filter;  /**< Enable HP Filter   */
    bool fifo_enabled;      /**< Enable FIFO        */
    bool int1_active;       // Track if INT1 is currently active
    bool int2_active;       // Track if INT2 is currently active
}lis3dh_config_t;

/**
 * @struct lis3dh_acc_params_s
 * @brief Internal Parameters
 * 
 * Its values are setted up depending of a
 * lis3dh_config_t type variable
 */
typedef struct lis3dh_params_s
{
    float acc_sensitivity;      /**< Sensitivity (mg/LSB) for current mode and fscale */
    int64_t acc_max_range;      /**< Maximum acceleration in current mode (mg) */
    int64_t acc_min_range;      /**< Minimum acceleration in current mode (mg) */
    int8_t acc_shifts;          /**< Number of left-shifts for raw_data in current mode */
}lis3dh_params_t;

/**
 * @struct lis3dh_i2c_params_s
 * @brief Internal parameters used in the internall I2C logic
 * 
 * lis3dh_config_t type variable
 */
typedef struct lis3dh_i2c_params_s
{
    uint8_t address;       /**< 7-bit I2C address */
	uint8_t ptr;           /**< Current register pointer */
	bool auto_increment;   /**< Auto-advance pointer after access */
	bool address_phase;    /**< I2C command phase tracker */
}lis3dh_i2c_params_t;

#endif /* INC_LIS3DH_TYPES_H_ */