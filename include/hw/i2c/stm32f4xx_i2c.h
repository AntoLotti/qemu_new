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

/**************************************************************************
    REGISTERS BIT
**************************************************************************/
#define  STM_I2C_SWRST_BIT          (uint32_t)BIT(15)
#define  STM32F4xx_I2C_CR1_RE2      (uint32_t)BIT(14)
#define  STM_I2C_ALERT_BIT          (uint32_t)BIT(13)
#define  STM_I2C_PEC_BIT            (uint32_t)BIT(12)
#define  STM_I2C_POS_BIT            (uint32_t)BIT(11)
#define  STM_I2C_ACK_BIT            (uint32_t)BIT(10)
#define  STM_I2C_STOP_BIT           (uint32_t)BIT(9)
#define  STM_I2C_START_BIT          (uint32_t)BIT(8)
#define  STM_I2C_NOSTRETCH_BIT      (uint32_t)BIT(7)
#define  STM_I2C_ENGC_BIT           (uint32_t)BIT(6)
#define  STM_I2C_ENPEC_BIT          (uint32_t)BIT(5)
#define  STM_I2C_ENARP_BIT          (uint32_t)BIT(4)
#define  STM_I2C_SMBTYPE_BIT        (uint32_t)BIT(3)
#define  STM32F4xx_I2C_CR1_RE1      (uint32_t)BIT(2)
#define  STM_I2C_SMBUS_BIT          (uint32_t)BIT(1)
#define  STM_I2C_PE_BIT             (uint32_t)BIT(0)

#define STM_I2C_LAST_BIT            (uint32_t)BIT(12)
#define STM_I2C_DMAEN_BIT           (uint32_t)BIT(11)
#define STM_I2C_ITBUFEN_BIT         (uint32_t)BIT(10)
#define STM_I2C_ITEVTEN_BIT         (uint32_t)BIT(9)
#define STM_I2C_ITERREN_BIT         (uint32_t)BIT(8)
#define STM_I2C_FREQ_BITS           (uint32_t)0x3F

#define STM_I2C_ADDMODE_BIT         (uint32_t)BIT(15)        
#define STM_I2C_ADD_BITS_9_TO_8     (uint32_t)0x300   
#define STM_I2C_ADD_BITS_7_TO_1     (uint32_t)0xFE       
#define STM_I2C_ADD0_BIT            (uint32_t)BIT(0)

#define STM_I2C_ADD2_BITS           (uint32_t)0x0E 
#define STM_I2C_ENDUAL_BIT          (uint32_t)BIT(0)

#define STM_I2C_DR_BITS             (uint32_t)0x0F

#define STM_I2C_SMBALERT_BIT        (uint32_t)BIT(15) 
#define STM_I2C_TIMEOUT_BIT         (uint32_t)BIT(14)
#define STM32F4xx_I2C_SR1_RE2       (uint32_t)BIT(13)
#define STM_I2C_PECERR_BIT          (uint32_t)BIT(12)
#define STM_I2C_OVR_BIT             (uint32_t)BIT(11)
#define STM_I2C_AF_BIT              (uint32_t)BIT(10)
#define STM_I2C_ARLO_BIT            (uint32_t)BIT(9)
#define STM_I2C_BERR_BIT            (uint32_t)BIT(8)
#define STM_I2C_TXE_BIT             (uint32_t)BIT(7)
#define STM_I2C_RXNE_BIT            (uint32_t)BIT(6)
#define STM32F4xx_I2C_SR1_RES1      (uint32_t)BIT(5)
#define STM_I2C_STOPF_BIT           (uint32_t)BIT(4)
#define STM_I2C_ADD10_BIT           (uint32_t)BIT(3)
#define STM_I2C_BTF_BIT             (uint32_t)BIT(2)
#define STM_I2C_ADDR_BIT            (uint32_t)BIT(1)
#define STM_I2C_SB_BIT              (uint32_t)BIT(0)

#define STM_I2C_PEC_BITS            (uint32_t)0xF0
#define STM_I2C_DUALF_BIT           (uint32_t)BIT(7)
#define STM_I2C_SMBHOST_BIT         (uint32_t)BIT(6)
#define STM_I2C_SMBDEFAUL_BIT       (uint32_t)BIT(5)
#define STM_I2C_GENCALL_BIT         (uint32_t)BIT(4)
#define STM_I2C_TRA_BIT             (uint32_t)BIT(2)
#define STM_I2C_BUSY_BIT            (uint32_t)BIT(1)
#define STM_I2C_MSL_BIT             (uint32_t)BIT(0)

#define STM_I2C_F_S_BIT             (uint32_t)BIT(15)                  
#define STM_I2C_DUTY_BIT            (uint32_t)BIT(14)
#define STM_I2C_CCR_BITS            (uint32_t)0xFFF

#define STM_I2C_TRISE_BITS          (uint32_t)0x3F

#define STM_I2C_ANOFF_BIT           (uint32_t)BIT(5) 
#define STM_I2C_DNF_BITS            (uint32_t)0x0F          

/**************************************************************************
    DEVICE STRUCTURES AND QOM DECLARATION
**************************************************************************/
/* Declaration of the QOM for the I2C of the stm32f4xx */
#define TYPE_STM32F4XX_I2C "stm32f4xx-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(STM32F4XXI2CState, STM32F4XX_I2C)

/* I2C state machine */
typedef enum states_e
{
    STM32F4xx_I2C_STATE_DISABLED,
    STM32F4xx_I2C_STATE_IDLE,
    STM32F4xx_I2C_STATE_START_SENT,           // START sent, waiting for address
    STM32F4xx_I2C_STATE_ADDR_SENT_READ,       // Address sent (read mode), waiting for ADDR clear
    STM32F4xx_I2C_STATE_ADDR_SENT_WRITE,      // Address sent (write mode), waiting for ADDR clear  
    STM32F4xx_I2C_STATE_RECEIVING,            // In receiver mode, receiving data
    STM32F4xx_I2C_STATE_TRANSMITTING,         // In transmitter mode, sending data
}states_t;

typedef struct flags_s
{
    bool flg_sb;
    bool flg_stop;
    bool flg_addr;
}flags_t;

/* STM32 I2C State struct requirements (the hardware) */
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
    states_t state;
    flags_t flags;

    uint8_t slv_address;

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