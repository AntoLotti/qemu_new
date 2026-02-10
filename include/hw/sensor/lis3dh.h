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

#ifndef INC_LIS3DH_I2C_H_
#define INC_LIS3DH_I2C_H_

#include "lis3dh_types.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "qom/object.h"


/* ==================== REGISTERS ADDRESSES ========================================== */

#define LIS3DH_ADDR_STATUS_REG_AUX      ((uint8_t)(0x07))	// Status Auxiliary Register

#define LIS3DH_ADDR_OUT_ADC1_L          ((uint8_t)(0x08))	// 1-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_ADC1_H          ((uint8_t)(0x09))	// 1-Axis Acceleration Data High Register
#define LIS3DH_ADDR_OUT_ADC2_L          ((uint8_t)(0x0A))	// 2-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_ADC2_H          ((uint8_t)(0x0B))	// 2-Axis Acceleration Data High Register
#define LIS3DH_ADDR_OUT_ADC3_L          ((uint8_t)(0x0C))	// 3-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_ADC3_H          ((uint8_t)(0x0D))	// 3-Axis Acceleration Data High Register

#define LIS3DH_ADDR_WHO_AM_I            ((uint8_t)(0x0F))	// Device identification Register

#define LIS3DH_ADDR_CTRL_REG0           ((uint8_t)(0x1E))	//
#define LIS3DH_ADDR_TEMP_CFG_REG        ((uint8_t)(0x1F))	// Temperature Sensor Register
#define LIS3DH_ADDR_CTRL_REG1           ((uint8_t)(0x20))	// Accelerometer Control Register 1
#define LIS3DH_ADDR_CTRL_REG2           ((uint8_t)(0x21))	// Accelerometer Control Register 2
#define LIS3DH_ADDR_CTRL_REG3           ((uint8_t)(0x22))	// Accelerometer Control Register 3
#define LIS3DH_ADDR_CTRL_REG4           ((uint8_t)(0x23))	// Accelerometer Control Register 4
#define LIS3DH_ADDR_CTRL_REG5           ((uint8_t)(0x24))	// Accelerometer Control Register 5
#define LIS3DH_ADDR_CTRL_REG6           ((uint8_t)(0x25))	// Accelerometer Control Register 6

#define LIS3DH_ADDR_REFERENCE           ((uint8_t)(0x26))	// Reference/Datacapture Register

#define LIS3DH_ADDR_STATUS_REG          ((uint8_t)(0x27))	// Status Register

#define LIS3DH_ADDR_OUT_X_L             ((uint8_t)(0x28))	// X-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_X_H             ((uint8_t)(0x29))	// X-Axis Acceleration Data High Register
#define LIS3DH_ADDR_OUT_Y_L             ((uint8_t)(0x2A))	// Y-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_Y_H             ((uint8_t)(0x2B))	// Y-Axis Acceleration Data High Register
#define LIS3DH_ADDR_OUT_Z_L             ((uint8_t)(0x2C))	// Z-Axis Acceleration Data Low Register
#define LIS3DH_ADDR_OUT_Z_H             ((uint8_t)(0x2D))	// Z-Axis Acceleration Data High Register

#define LIS3DH_ADDR_FIFO_CTRL           ((uint8_t)(0x2E))	// FIFO Control Register
#define LIS3DH_ADDR_FIFO_SRC            ((uint8_t)(0x2F))	// FIFO Source Register

#define LIS3DH_ADDR_INT1_CFG            ((uint8_t)(0x30))	// Interrupt 1 Configuration Register
#define LIS3DH_ADDR_INT1_SRC            ((uint8_t)(0x31))	// Interrupt 1 Source Register
#define LIS3DH_ADDR_INT1_THS            ((uint8_t)(0x32))	// Interrupt 1 Threshold Register
#define LIS3DH_ADDR_INT1_DURATION       ((uint8_t)(0x33))	// Interrupt 1 Duration Register

#define LIS3DH_ADDR_INT2_CFG            ((uint8_t)(0x34))	// Interrupt 2 Configuration Register
#define LIS3DH_ADDR_INT2_SRC            ((uint8_t)(0x35))	// Interrupt 2 Source Register
#define LIS3DH_ADDR_INT2_THS            ((uint8_t)(0x36))	// Interrupt 2 Threshold Register
#define LIS3DH_ADDR_INT2_DURATION       ((uint8_t)(0x37))	// Interrupt 2 Duration Register

#define LIS3DH_ADDR_CLICK_CFG           ((uint8_t)(0x38))	// Interrupt Click Recognition Register
#define LIS3DH_ADDR_CLICK_SRC           ((uint8_t)(0x39))	// Interrupt Click Source Register
#define LIS3DH_ADDR_CLICK_THS           ((uint8_t)(0x3A))	// Interrupt Click Threshold Register

#define LIS3DH_ADDR_TIME_LIMIT          ((uint8_t)(0x3B))	// Click Time Limit Register
#define LIS3DH_ADDR_TIME_LATENCY        ((uint8_t)(0x3C))	// Click Time Latency Register
#define LIS3DH_ADDR_TIME_WINDOW         ((uint8_t)(0x3D))	// Click Time Window Register

#define LIS3DH_ADDR_ACT_THS             ((uint8_t)(0x3E))	//
#define LIS3DH_ADDR_ACT_DUR             ((uint8_t)(0x3F))	//


/* ==================== REGISTERS DEFAULT VALUES ===================================== */

/* Default */
#define LIS3DH_REGS_DEF                     ((uint8_t)0x00)
#define LIS3DH_OUTPUTS_DEF                  ((uint8_t)0x00)

#define LIS3DH_STATUS_REG_AUX_DEF           LIS3DH_OUTPUTS_DEF  // Status Register
#define LIS3DH_ADC_1_L_DEF                  LIS3DH_OUTPUTS_DEF  // 1-Axis Acceleration Data Low Register
#define LIS3DH_ADC_1_H_DEF                  LIS3DH_OUTPUTS_DEF  // 1-Axis Acceleration Data High Register
#define LIS3DH_ADC_2_L_DEF                  LIS3DH_OUTPUTS_DEF  // 2-Axis Acceleration Data Low Register
#define LIS3DH_ADC_2_H_DEF                  LIS3DH_OUTPUTS_DEF  // 2-Axis Acceleration Data High Register
#define LIS3DH_ADC_3_L_DEF                  LIS3DH_OUTPUTS_DEF  // 3-Axis Acceleration Data Low Register
#define LIS3DH_ADC_3_H_DEF                  LIS3DH_OUTPUTS_DEF  // 3-Axis Acceleration Data High Register

#define LIS3DH_WHO_AM_I_DEF                 ((uint8_t)0x33)     // Device identification Register 00110011 (default in write)

#define LIS3DH_CTRL_REG0_DEF                ((uint8_t)0x10)     // 
#define LIS3DH_TEMP_CFG_REG_DEF             LIS3DH_REGS_DEF     // Temperature Sensor Register
#define LIS3DH_CTRL_REG1_DEF                ((uint8_t)0x07)     // Accelerometer Control Register 1
#define LIS3DH_CTRL_REG2_DEF                LIS3DH_REGS_DEF     // Accelerometer Control Register 2
#define LIS3DH_CTRL_REG3_DEF                LIS3DH_REGS_DEF     // Accelerometer Control Register 3
#define LIS3DH_CTRL_REG4_DEF                LIS3DH_REGS_DEF     // Accelerometer Control Register 4
#define LIS3DH_CTRL_REG5_DEF                LIS3DH_REGS_DEF     // Accelerometer Control Register 5
#define LIS3DH_CTRL_REG6_DEF                LIS3DH_REGS_DEF     // Accelerometer Control Register 6

#define LIS3DH_REFERENCE_DEF                LIS3DH_REGS_DEF     // Reference/Datacapture Register

#define LIS3DH_STATUS_REG_DEF               LIS3DH_OUTPUTS_DEF  // Status Register 2

#define LIS3DH_OUT_X_L_DEF                  LIS3DH_OUTPUTS_DEF  // X-Axis Acceleration Data Low Register
#define LIS3DH_OUT_X_H_DEF                  LIS3DH_OUTPUTS_DEF  // X-Axis Acceleration Data High Register
#define LIS3DH_OUT_Y_L_DEF                  LIS3DH_OUTPUTS_DEF  // Y-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Y_H_DEF                  LIS3DH_OUTPUTS_DEF  // Y-Axis Acceleration Data High Register
#define LIS3DH_OUT_Z_L_DEF                  LIS3DH_OUTPUTS_DEF  // Z-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Z_H_DEF                  LIS3DH_OUTPUTS_DEF  // Z-Axis Acceleration Data High Register

#define LIS3DH_FIFO_CTRL_REG_DEF            LIS3DH_REGS_DEF     // FIFO Control Register
#define LIS3DH_FIFO_SRC_REG_DEF             LIS3DH_REGS_DEF     // FIFO Source Register

#define LIS3DH_INT1_CFG_DEF                 LIS3DH_REGS_DEF     // Interrupt 1 Configuration Register
#define LIS3DH_INT1_SRC_DEF                 LIS3DH_OUTPUTS_DEF  // Interrupt 1 Source Register
#define LIS3DH_INT1_THS_DEF                 LIS3DH_REGS_DEF     // Interrupt 1 Threshold Register
#define LIS3DH_INT1_DURATION_DEF            LIS3DH_REGS_DEF     // Interrupt 1 Duration Register

#define LIS3DH_INT2_CFG_DEF                 LIS3DH_REGS_DEF     // Interrupt 2 Configuration Register
#define LIS3DH_INT2_SRC_DEF                 LIS3DH_OUTPUTS_DEF  // Interrupt 2 Source Register
#define LIS3DH_INT2_THS_DEF                 LIS3DH_REGS_DEF     // Interrupt 2 Threshold Register
#define LIS3DH_INT2_DURATION_DEF            LIS3DH_REGS_DEF     // Interrupt 2 Duration Register

#define LIS3DH_CLICK_CFG_DEF                LIS3DH_REGS_DEF     // Interrupt Click Recognition Register
#define LIS3DH_CLICK_SRC_DEF                LIS3DH_OUTPUTS_DEF  // Interrupt Click Source Register
#define LIS3DH_CLICK_THS_DEF                LIS3DH_REGS_DEF     // Interrupt Click Threshold Register

#define LIS3DH_TIME_LIMIT_DEF               LIS3DH_REGS_DEF     // Click Time Limit Register
#define LIS3DH_TIME_LATENCY_DEF             LIS3DH_REGS_DEF     // Click Time Latency Register
#define LIS3DH_TIME_WINDOW_DEF              LIS3DH_REGS_DEF     // Click Time Window Register

#define LIS3DH_ACT_THS_DEF                  LIS3DH_REGS_DEF     // 
#define LIS3DH_ACT_DUR_DEF                  LIS3DH_REGS_DEF     // 


/* ==================== BITS OF EACH REGISTER ======================================== */

#define LIS3DH_STATUS_REG_AUX_BIT_31OR      (uint8_t)BIT(7)
#define LIS3DH_STATUS_REG_AUX_BIT_3OR       (uint8_t)BIT(6)
#define LIS3DH_STATUS_REG_AUX_BIT_2OR       (uint8_t)BIT(5)
#define LIS3DH_STATUS_REG_AUX_BIT_1OR       (uint8_t)BIT(4)
#define LIS3DH_STATUS_REG_AUX_BIT_321DA     (uint8_t)BIT(3)
#define LIS3DH_STATUS_REG_AUX_BIT_3DA       (uint8_t)BIT(2)
#define LIS3DH_STATUS_REG_AUX_BIT_2DA       (uint8_t)BIT(1)
#define LIS3DH_STATUS_REG_AUX_BIT_1DA       (uint8_t)BIT(0)

#define LIS3DH_CTRL_REG0_BIT_SDO_PU_DISC    (uint8_t)BIT(7)

#define LIS3DH_TEMP_CFG_BIT_REG_ADC_EN      (uint8_t)BIT(7)
#define LIS3DH_TEMP_CFG_BIT_REG_TEMP_EN     (uint8_t)BIT(6)

#define LIS3DH_CTRL_REG1_BITS_ODR           (uint8_t)(0xF0)
#define LIS3DH_CTRL_REG1_BIT_LPEN           (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG1_BIT_ZEN            (uint8_t)BIT(2)
#define LIS3DH_CTRL_REG1_BIT_YEN            (uint8_t)BIT(1)
#define LIS3DH_CTRL_REG1_BIT_XEN            (uint8_t)BIT(0)

#define LIS3DH_CTRL_REG2_BITS_HMP           (uint8_t)(0xC0)
#define LIS3DH_CTRL_REG2_BITS_HMCF          (uint8_t)(0x30)
#define LIS3DH_CTRL_REG2_BIT_FDS            (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG2_BIT_HPCLICK        (uint8_t)BIT(2)
#define LIS3DH_CTRL_REG2_BIT_HP_IA2         (uint8_t)BIT(1)
#define LIS3DH_CTRL_REG2_BIT_HP_IA1         (uint8_t)BIT(0)

#define LIS3DH_CTRL_REG3_BIT_I1_CLICK       (uint8_t)BIT(7)
#define LIS3DH_CTRL_REG3_BIT_I1_IA1         (uint8_t)BIT(6)
#define LIS3DH_CTRL_REG3_BIT_I1_IA2         (uint8_t)BIT(5)
#define LIS3DH_CTRL_REG3_BIT_I1_ZYXDA       (uint8_t)BIT(4)
#define LIS3DH_CTRL_REG3_BIT_I1_321DA       (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG3_BIT_I1_WTM         (uint8_t)BIT(2)
#define LIS3DH_CTRL_REG3_BIT_I1_OVERRUN     (uint8_t)BIT(1)

#define LIS3DH_CTRL_REG4_BIT_BDU            (uint8_t)BIT(7)
#define LIS3DH_CTRL_REG4_BIT_BLE            (uint8_t)BIT(6)
#define LIS3DH_CTRL_REG4_BITS_FS            (uint8_t)(0x30)
#define LIS3DH_CTRL_REG4_BIT_HR             (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG4_BITS_ST            (uint8_t)(0x06)
#define LIS3DH_CTRL_REG4_BIT_SIM            (uint8_t)BIT(0)

#define LIS3DH_CTRL_REG5_BIT_BOOT           (uint8_t)BIT(7)
#define LIS3DH_CTRL_REG5_BIT_FIFO_EN        (uint8_t)BIT(6)
#define LIS3DH_CTRL_REG5_BIT_LIR_INT1       (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG5_BIT_D4D_INT1       (uint8_t)BIT(2)
#define LIS3DH_CTRL_REG5_BIT_LIR_INT2       (uint8_t)BIT(1)
#define LIS3DH_CTRL_REG5_BIT_D4D_INT2       (uint8_t)BIT(0)

#define LIS3DH_CTRL_REG6_BIT_I2_CLICK       (uint8_t)BIT(7)
#define LIS3DH_CTRL_REG6_BIT_I2_IA1         (uint8_t)BIT(6)
#define LIS3DH_CTRL_REG6_BIT_I2_IA2         (uint8_t)BIT(5)
#define LIS3DH_CTRL_REG6_BIT_I2_BOOT        (uint8_t)BIT(4)
#define LIS3DH_CTRL_REG6_BIT_I2_ACT         (uint8_t)BIT(3)
#define LIS3DH_CTRL_REG6_BIT_I2_POLARITY    (uint8_t)BIT(1)

#define LIS3DH_REFERENCE                    (uint8_t)(0xFF)

#define LIS3DH_STATUS_REG_BIT_ZYXOR         (uint8_t)BIT(7)
#define LIS3DH_STATUS_REG_BIT_ZOR           (uint8_t)BIT(6)
#define LIS3DH_STATUS_REG_BIT_YOR           (uint8_t)BIT(5)
#define LIS3DH_STATUS_REG_BIT_XOR           (uint8_t)BIT(4)
#define LIS3DH_STATUS_REG_BIT_ZYXDA         (uint8_t)BIT(3)
#define LIS3DH_STATUS_REG_BIT_ZDA           (uint8_t)BIT(2)
#define LIS3DH_STATUS_REG_BIT_YDA           (uint8_t)BIT(1)
#define LIS3DH_STATUS_REG_BIT_XDA           (uint8_t)BIT(0)

#define LIS3DH_FIFO_CTRL_REG_BITS_FM        (uint8_t)(0xC0)
#define LIS3DH_FIFO_CTRL_REG_BIT_TR         (uint8_t)BIT(5)

#define LIS3DH_FIFO_SRC_REG_BIT_WTM         (uint8_t)BIT(7)
#define LIS3DH_FIFO_SRC_REG_BIT_OVRN_FIFO   (uint8_t)BIT(6)
#define LIS3DH_FIFO_SRC_REG_BIT_EMPTY       (uint8_t)BIT(5)
#define LIS3DH_FIFO_SRC_REG_BITS_FSS        (uint8_t)(0x1F)

#define LIS3DH_INT1_CFG_AOI_BIT             (uint8_t)BIT(7)
#define LIS3DH_INT1_CFG_6D_BIT              (uint8_t)BIT(6)
#define LIS3DH_INT1_CFG_ZHIE_BIT            (uint8_t)BIT(5)
#define LIS3DH_INT1_CFG_ZLIE_BIT            (uint8_t)BIT(4)
#define LIS3DH_INT1_CFG_YHIE_BIT            (uint8_t)BIT(3)
#define LIS3DH_INT1_CFG_YLIE_BIT            (uint8_t)BIT(2)
#define LIS3DH_INT1_CFG_XHIE_BIT            (uint8_t)BIT(1)
#define LIS3DH_INT1_CFG_XLIE_BIT            (uint8_t)BIT(0)         

#define LIS3DH_INT1_SRC_IA_BIT              (uint8_t)BIT(6)
#define LIS3DH_INT1_SRC_ZH_BIT              (uint8_t)BIT(5)
#define LIS3DH_INT1_SRC_ZL_BIT              (uint8_t)BIT(4)
#define LIS3DH_INT1_SRC_YH_BIT              (uint8_t)BIT(3)
#define LIS3DH_INT1_SRC_YL_BIT              (uint8_t)BIT(2)
#define LIS3DH_INT1_SRC_XH_BIT              (uint8_t)BIT(1)
#define LIS3DH_INT1_SRC_XL_BIT              (uint8_t)BIT(0)     

#define LIS3DH_INT1_THS_BITS                (uint8_t)(0x7F)

#define LIS3DH_INT1_DURATION_BITS           (uint8_t)(0x7F)   

/* ==================== INTERNAL MACROS ============================================== */

#define LIS3DH_DEFAULT_ADDRESS          ((uint8_t)0x18 << 1)    // if SDO/SA0 = 1 -> 0001 1000, then 0001 1000 << 1 = 00011 0000
#define LIS3DH_ALTERNATIVE_ADDRESS      ((uint8_t)0x19 << 1)    // if SDO/SA0 = 0 -> 0001 1001, then 0001 1001 << 1 = 00011 0010 

#define LIS3DH_SUB_REG_MASK             0x7F                    // Mask to get the LIS3DH register direcction
#define LIS3DH_SUB_AUTO_INC_MASK        0x80                    // Mask to get the Auto-increment bit

#define LIS3DH_ACCEL_CONST    (9.81f)   // Gravity constant (m/s²)

#define LIS3DH_TEMP_MAX    -40  // MIN Value for the temperature sensor
#define LIS3DH_TEMP_MIN     85  // MAX Value for the temperature sensor

#define LIS3DH_MAX_2G_RANGE      2000   // Acc MAX value (in mg) when FS = 2 
#define LIS3DH_MIN_2G_RANGE     -2000   // Acc MIN value (in mg) when FS = 2 

#define LIS3DH_MAX_4G_RANGE      4000   // Acc MAX value (in mg) when FS = 4
#define LIS3DH_MIN_4G_RANGE     -4000   // Acc MIN value (in mg) when FS = 4

#define LIS3DH_MAX_8G_RANGE      8000   // Acc MAX value (in mg) when FS = 8
#define LIS3DH_MIN_8G_RANGE     -8000   // Acc MIN value (in mg) when FS = 8

#define LIS3DH_MAX_16G_RANGE     16000  // Acc MAX value (in mg) when FS = 16
#define LIS3DH_MIN_16G_RANGE    -16000  // Acc MIN value (in mg) when FS = 16


#define LIS3DH_So_HIG_RES_2G    1.0f	// mg/LSB or mg/digit
#define LIS3DH_So_HIG_RES_4G    2.0f    // mg/LSB or mg/digit
#define LIS3DH_So_HIG_RES_8G    4.0f    // mg/LSB or mg/digit
#define LIS3DH_So_HIG_RES_16G   12.0f   // mg/LSB or mg/digit

#define LIS3DH_So_NORMAL_2G     4.0f    // mg/LSB or mg/digit
#define LIS3DH_So_NORMAL_4G     8.0f    // mg/LSB or mg/digit
#define LIS3DH_So_NORMAL_8G     16.0f   // mg/LSB or mg/digit
#define LIS3DH_So_NORMAL_16G    48.0f   // mg/LSB or mg/digit

#define LIS3DH_So_LOW_POWER_2G  16.0f   // mg/LSB or mg/digit
#define LIS3DH_So_LOW_POWER_4G  32.0f   // mg/LSB or mg/digit
#define LIS3DH_So_LOW_POWER_8G  64.0f   // mg/LSB or mg/digit
#define LIS3DH_So_LOW_POWER_16G 192.0f  // mg/LSB or mg/digit


#define CASE_READ_RETURN(VAL, REG, FIELD)   case REG: VAL = (src->FIELD); break;

/**************************************************************************
    DEVICE STRUCTURES AND QOM DECLARATION
**************************************************************************/

/* Declaration of the QOM for the LIS3DH */
#define TYPE_LIS3DH "lis3dh"
OBJECT_DECLARE_SIMPLE_TYPE(LIS3DHState, LIS3DH);

/* LIS3DH State struct requirements (the hardware) */
typedef struct LIS3DHState
{
    /* Parent object */
    I2CSlave i2c;

    lis3dh_i2c_params_t i2c_params;    
    lis3dh_config_t *config;
    lis3dh_params_t *params;
    
    qemu_irq int1;                    /* Interrupt 1 line to STM32 */
    qemu_irq int2;                    /* Interrupt 2 line to STM32 */

    /* Registers */
    uint8_t status_reg_aux;     // Status Register
    uint8_t adc_1_l;            // 1-Axis Acceleration Data Low Register
    uint8_t adc_1_h;            // 1-Axis Acceleration Data High Register
    uint8_t adc_2_l;            // 2-Axis Acceleration Data Low Register
    uint8_t adc_2_h;            // 2-Axis Acceleration Data High Register
    uint8_t adc_3_l;            // 3-Axis Acceleration Data Low Register
    uint8_t adc_3_h;            // 3-Axis Acceleration Data High Register
    
    uint8_t who_am_i;           // Device identification Register 
        
    uint8_t ctrl_reg0;          //
    uint8_t temp_cfg_reg;       // Temperature Sensor Register
    uint8_t ctrl_reg1;          // Accelerometer Control Register 1
    uint8_t ctrl_reg2;          // Accelerometer Control Register 2
    uint8_t ctrl_reg3;          // Accelerometer Control Register 3
    uint8_t ctrl_reg4;          // Accelerometer Control Register 4
    uint8_t ctrl_reg5;          // Accelerometer Control Register 5
    uint8_t ctrl_reg6;          // Accelerometer Control Register 6
    
    uint8_t reference;          // Reference/Datacapture Register
    uint8_t status_reg;         // Status Register 2
    
    uint8_t out_x_l;            // X-Axis Acceleration Data Low Register
    uint8_t out_x_h;            // X-Axis Acceleration Data High Register
    uint8_t out_y_l;            // Y-Axis Acceleration Data Low Register
    uint8_t out_y_h;            // Y-Axis Acceleration Data High Register
    uint8_t out_z_l;            // Z-Axis Acceleration Data Low Register
    uint8_t out_z_h;            // Z-Axis Acceleration Data High Register
    
    uint8_t fifo_ctrl_reg;      // FIFO Control Register
    uint8_t fifo_src_reg;       // FIFO Source Register

    uint8_t int1_cfg;           // Interrupt 1 Configuration Register
    uint8_t int1_src;           // Interrupt 1 Source Register
    uint8_t int1_ths;           // Interrupt 1 Threshold Register
    uint8_t int1_duration;      // Interrupt 1 Duration Register
    
    uint8_t int2_cfg;           // Interrupt 2 Configuration Register
    uint8_t int2_src;           // Interrupt 2 Source Register
    uint8_t int2_ths;           // Interrupt 2 Threshold Register
    uint8_t int2_duration;      // Interrupt 2 Duration Register
    
    uint8_t click_cfg;          // Interrupt Click Recognition Register
    uint8_t click_src;          // Interrupt Click Source Register
    uint8_t click_ths;          // Interrupt Click Threshold Register
    
    uint8_t time_limit;         // Click Time Limit Register
    uint8_t time_latency;       // Click Time Latency Register
    uint8_t time_window;        // Click Time Window Register
    
    uint8_t act_ths;            //
    uint8_t act_dur;            //
} LIS3DHState;

#endif