/*
 *  STM32F4XX I2C 
 *
 * Copyright (c) 2025 Antonio Lotti Villar <antoniolottivillar@gmail.com>
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

#ifndef HW_STM32F4XX_I2C_H
#define HW_STM32F4XX_I2C_H

#include <stdint.h>
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"

/**************************************************************************
    I2C REGISTERS ADDRESSES
**************************************************************************/
#define  STM_I2C_REG_CR1    ((uint8_t)(0x00))	// 
#define  STM_I2C_REG_CR2    ((uint8_t)(0x04))	// 
#define  STM_I2C_REG_OAR1   ((uint8_t)(0x08))	// 
#define  STM_I2C_REG_OAR2   ((uint8_t)(0x0C))	// 
#define  STM_I2C_REG_DR     ((uint8_t)(0x10))	// 
#define  STM_I2C_REG_SR1    ((uint8_t)(0x14))	// 
#define  STM_I2C_REG_SR2    ((uint8_t)(0x18))	// 
#define  STM_I2C_REG_CCR    ((uint8_t)(0x1C))	// 
#define  STM_I2C_REG_TRISE  ((uint8_t)(0x20))	// 
#define  STM_I2C_REG_FLTR   ((uint8_t)(0x24))	// 

/**************************************************************************
    ACCELEROMETER REGISTERS DEFAULT VALUES
**************************************************************************/
#define  STM_I2C_REG_CR1_DEF    ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_CR2_DEF    ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_OAR1_DEF   ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_OAR2_DEF   ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_DR_DEF     ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_SR1_DEF    ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_SR2_DEF    ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_CCR_DEF    ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_TRISE_DEF  ((uint32_t)(0x00))	// 
#define  STM_I2C_REG_FLTR_DEF   ((uint32_t)(0x00))	// 

//#define BCM2835_I2C_C_I2CEN     BIT(15)           /* I2C enable */

/**************************************************************************
    DEVICE STRUCTURES AND QOM DECLARATION
**************************************************************************/
/* Declaration of the QOM for the I2C of the stm32f4xx */
#define TYPE_STM32F4XX_I2C "stm32f4xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXI2CState, STM32F4XX_I2C)

/* LIS3DH State struct requirements (the hardware) */
typedef struct STM32F4XXI2CState{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

    uint32_t i2c_cr1; 
    uint32_t i2c_cr2; 
    uint32_t i2c_oar1; 
    uint32_t i2c_oar2; 
    uint32_t i2c_dr; 
    uint32_t i2c_sr1; 
    uint32_t i2c_sr2; 
    uint32_t i2c_ccr; 
    uint32_t i2c_trise; 
    uint32_t i2c_fltr; 

}STM32F4XXI2CState;

#endif /* HW_STM32F4XX_I2C_H */