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
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "stm32f4xx_types_i2c.h"

/* ====== STM32F4 I2C REGISTERS ADDRESSES ====== */

#define STM32F4_I2C_CR1_ADDR       ((uint8_t)(0x00))
#define STM32F4_I2C_CR2_ADDR       ((uint8_t)(0x04))
#define STM32F4_I2C_OAR1_ADDR      ((uint8_t)(0x08))
#define STM32F4_I2C_OAR2_ADDR      ((uint8_t)(0x0C))
#define STM32F4_I2C_DR_ADDR        ((uint8_t)(0x10))
#define STM32F4_I2C_SR1_ADDR       ((uint8_t)(0x14))
#define STM32F4_I2C_SR2_ADDR       ((uint8_t)(0x18))
#define STM32F4_I2C_CCR_ADDR       ((uint8_t)(0x1C))
#define STM32F4_I2C_TRISE_ADDR     ((uint8_t)(0x20))
#define STM32F4_I2C_FLTR_ADDR      ((uint8_t)(0x24))

/* ====== STM32F4 I2C REGISTERS DEFAULT VALUES ====== */

#define STM32F4_I2C_CR1_DEF        ((uint32_t)(0x0000))
#define STM32F4_I2C_CR2_DEF        ((uint32_t)(0x0000))
#define STM32F4_I2C_OAR1_DEF       ((uint32_t)(0x0000))
#define STM32F4_I2C_OAR2_DEF       ((uint32_t)(0x0000))
#define STM32F4_I2C_DR_DEF         ((uint32_t)(0x0000))
#define STM32F4_I2C_SR1_DEF        ((uint32_t)(0x0000))
#define STM32F4_I2C_SR2_DEF        ((uint32_t)(0x0000))
#define STM32F4_I2C_CCR_DEF        ((uint32_t)(0x0000))
#define STM32F4_I2C_TRISE_DEF      ((uint32_t)(0x0000))
#define STM32F4_I2C_FLTR_DEF       ((uint32_t)(0x0000))

/* ====== STM32F4 I2C REGISTERS BITS ====== */

#define STM32F4_I2C_SWRST_BIT       (uint32_t)BIT(15)
#define STM32F4_I2C_CR1_RE2         (uint32_t)BIT(14)   // Typically reserved
#define STM32F4_I2C_ALERT_BIT       (uint32_t)BIT(13)
#define STM32F4_I2C_PEC_BIT         (uint32_t)BIT(12)
#define STM32F4_I2C_POS_BIT         (uint32_t)BIT(11)
#define STM32F4_I2C_ACK_BIT         (uint32_t)BIT(10)
#define STM32F4_I2C_STOP_BIT        (uint32_t)BIT(9)
#define STM32F4_I2C_START_BIT       (uint32_t)BIT(8)
#define STM32F4_I2C_NOSTRETCH_BIT   (uint32_t)BIT(7)
#define STM32F4_I2C_ENGC_BIT        (uint32_t)BIT(6)
#define STM32F4_I2C_ENPEC_BIT       (uint32_t)BIT(5)
#define STM32F4_I2C_ENARP_BIT       (uint32_t)BIT(4)
#define STM32F4_I2C_SMBTYPE_BIT     (uint32_t)BIT(3)
#define STM32F4_I2C_CR1_RE1         (uint32_t)BIT(2)    // Typically reserved
#define STM32F4_I2C_SMBUS_BIT       (uint32_t)BIT(1)
#define STM32F4_I2C_PE_BIT          (uint32_t)BIT(0)

#define STM32F4_I2C_LAST_BIT        (uint32_t)BIT(12)
#define STM32F4_I2C_DMAEN_BIT       (uint32_t)BIT(11)
#define STM32F4_I2C_ITBUFEN_BIT     (uint32_t)BIT(10)
#define STM32F4_I2C_ITEVTEN_BIT     (uint32_t)BIT(9)
#define STM32F4_I2C_ITERREN_BIT     (uint32_t)BIT(8)
#define STM32F4_I2C_FREQ_MASK       (uint32_t)0x003F

#define STM32F4_I2C_ADDMODE_BIT     (uint32_t)BIT(15)        
#define STM32F4_I2C_ADD_MASK        (uint32_t)0x03FE  
#define STM32F4_I2C_ADD0_BIT        (uint32_t)BIT(0)

#define STM32F4_I2C_ADD2_MASK       (uint32_t)0x00FE
#define STM32F4_I2C_ENDUAL_BIT      (uint32_t)BIT(0)

#define STM32F4_I2C_DR_BITS         (uint32_t)0xFF

#define STM32F4_I2C_SMBALERT_BIT    (uint32_t)BIT(15) 
#define STM32F4_I2C_TIMEOUT_BIT     (uint32_t)BIT(14)
#define STM32F4_I2C_SR1_RE2         (uint32_t)BIT(13)
#define STM32F4_I2C_PECERR_BIT      (uint32_t)BIT(12)
#define STM32F4_I2C_OVR_BIT         (uint32_t)BIT(11)
#define STM32F4_I2C_AF_BIT          (uint32_t)BIT(10)
#define STM32F4_I2C_ARLO_BIT        (uint32_t)BIT(9)
#define STM32F4_I2C_BERR_BIT        (uint32_t)BIT(8)
#define STM32F4_I2C_TXE_BIT         (uint32_t)BIT(7)
#define STM32F4_I2C_RXNE_BIT        (uint32_t)BIT(6)
#define STM32F4_I2C_SR1_RES1        (uint32_t)BIT(5)
#define STM32F4_I2C_STOPF_BIT       (uint32_t)BIT(4)
#define STM32F4_I2C_ADD10_BIT       (uint32_t)BIT(3)
#define STM32F4_I2C_BTF_BIT         (uint32_t)BIT(2)
#define STM32F4_I2C_ADDR_BIT        (uint32_t)BIT(1)
#define STM32F4_I2C_SB_BIT          (uint32_t)BIT(0)

#define STM32F4_I2C_PEC_BITS        (uint32_t)0xF0
#define STM32F4_I2C_DUALF_BIT       (uint32_t)BIT(7)
#define STM32F4_I2C_SMBHOST_BIT     (uint32_t)BIT(6)
#define STM32F4_I2C_SMBDEFAUL_BIT   (uint32_t)BIT(5)
#define STM32F4_I2C_GENCALL_BIT     (uint32_t)BIT(4)
#define STM32F4_I2C_TRA_BIT         (uint32_t)BIT(2)
#define STM32F4_I2C_BUSY_BIT        (uint32_t)BIT(1)
#define STM32F4_I2C_MSL_BIT         (uint32_t)BIT(0)

#define STM32F4_I2C_CCR_FS_BIT      (uint32_t)BIT(15)                  
#define STM32F4_I2C_CCR_DUTY_BIT    (uint32_t)BIT(14)
#define STM32F4_I2C_CCR_MASK        (uint32_t)0x0FFF

#define STM32F4_I2C_TRISE_BITS      (uint32_t)0x003F

#define STM32F4_I2C_ANOFF_BIT       (uint32_t)BIT(5) 
#define STM32F4_I2C_DNF_BITS        (uint32_t)0x000F          


/* ====== STM32F4 I2C QOM DECLARATIONS ====== */

/* Declaration of the QOM for the I2C of the stm32f4xx */
#define TYPE_STM32F4XX_I2C "stm32f4xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXI2CState, STM32F4XX_I2C)

/* stm32f4xx I2C fsm type */
typedef struct stm32f4xx_i2c_fsm_s stm32f4xx_i2c_fsm_t;

/* STM32 I2C State struct requirements */
typedef struct STM32F4XXI2CState
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion iomem;
    I2CBus *bus;

    qemu_irq irq_event;
    qemu_irq irq_error;
    
    char *bus_name;

    /* for internall logic */
    stm32f4xx_i2c_config_t config;
    stm32f4xx_i2c_fsm_t *fsm;

   /* Registers*/ 
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

QEMU_BUILD_BUG_ON(sizeof(STM32F4XXI2CState) < 256);  // Minimum expected size

#endif /* HW_STM32F4XX_I2C_H */