/*
 * Broadcom Serial Controller (BSC)
 *
 * Copyright (c) 2024 Rayhan Faizel <rayhan.faizel@gmail.com>
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

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/i2c/stm32f4xx_i2c.h"
#include "migration/vmstate.h"

static bool flg_sb      = false;
static bool flg_addr    = false;
static bool flg_stop    = false;

/**************************************************************************
    I2C FUNCTIONS
**************************************************************************/
static bool __stm32f4xx_i2c_start_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_IDLE) 
        || (src->state == STM32F4xx_I2C_STATE_TRANSMITTING )
    );
}

static bool __stm32f4xx_i2c_re_start_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->i2c_sr2 & STM_I2C_BUSY_BIT) != 0 
    );
}

static bool __stm32f4xx_i2c_stop_condition(STM32F4XXI2CState* src)
{
    printf("\nflg_stop = %s", flg_stop ? "TRUE" : "FALSE");
    printf("\nsrc->i2c_cr1 & STM_I2C_ACK_BIT = 0x%x", (src->i2c_cr1 & STM_I2C_ACK_BIT));

    return
    ( 
        ( (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_READ)
            || (src->state == STM32F4xx_I2C_STATE_RECEIVING) )
        && ( (src->i2c_cr1 & STM_I2C_ACK_BIT) == 0 )
        && flg_stop
    );
}

static bool __stm32f4xx_i2c_address_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        src->state == STM32F4xx_I2C_STATE_START_SENT 
    );
}

static bool __stm32f4xx_i2c_transmitting_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_WRITE) 
        || (src->state == STM32F4xx_I2C_STATE_TRANSMITTING)
    );
}

static bool __stm32f4xx_i2c_reciving_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_READ)
        || (src->state == STM32F4xx_I2C_STATE_RECEIVING)
    );
}


static void stm32f4xx_i2c_begin_transfer(STM32F4XXI2CState* src, uint64_t value)
{

    uint8_t address = (uint8_t)(extract32(value, 1, 7));
    uint8_t mode    = (uint8_t)(extract32(value, 0, 1));

    printf("\n STM address: 0x%x \n", address);
 
    if ( i2c_start_transfer( src->bus, address, mode ) )
    {
        printf("Error during i2c_start_transfer \n");
        return;
    }
    else
    {
        printf("All ok during i2c_start_transfer \n");

        src->slv_address = address;

        src->i2c_sr2 = 
        ( (mode == 0) ? (src->i2c_sr2 & ~STM_I2C_TRA_BIT) : (src->i2c_sr2 | STM_I2C_TRA_BIT) );

        src->i2c_sr1 |= STM_I2C_ADDR_BIT;

        if ( mode == 0 )
        {
            printf("\n STM32F4xx_I2C_STATE_ADDR_SENT_WRITE \n");
            src->i2c_sr1 |= STM_I2C_TXE_BIT;
            src->state = STM32F4xx_I2C_STATE_ADDR_SENT_WRITE;
        }
        else
        {
            printf("\n STM32F4xx_I2C_STATE_ADDR_SENT_READ \n");        
            src->i2c_sr1 |= STM_I2C_RXNE_BIT;
            src->state = STM32F4xx_I2C_STATE_ADDR_SENT_READ;
        }

    }

}

static void stm32f4xx_i2c_stop_generation(STM32F4XXI2CState* src)
{
    src->state = STM32F4xx_I2C_STATE_IDLE;  // Change state to IDLE
    src->i2c_sr2 &= ~STM_I2C_MSL_BIT;       // Clear the MSL bit in SR2
    src->i2c_sr2 &= ~STM_I2C_BUSY_BIT;      // Bus no longer busy
    src->i2c_sr1 &= ~STM_I2C_TXE_BIT;

    flg_stop = false;
    flg_addr = false;
    flg_sb = false;
}

static void stm32f4xx_i2c_data_transfer(STM32F4XXI2CState *src, uint64_t value)
{
    if ( i2c_send(src->bus, value) )
    {
        printf("\n Error during i2c_send \n");
    }else
    {
        printf("\n All ok during i2c_send \n");
        src->i2c_sr1 |= STM_I2C_TXE_BIT;
        src->state = STM32F4xx_I2C_STATE_TRANSMITTING;
    }
    
}

static void stm32f4xx_i2c_data_recive(STM32F4XXI2CState *src)
{
    uint8_t ret = i2c_recv(src->bus);
    
    printf("\n Data received: 0x%x \n", ret);
    
    if ( __stm32f4xx_i2c_stop_condition(src) )
    {
        printf("\n STM32 Stop Condition \n");

        i2c_nack(src->bus);
        i2c_end_transfer(src->bus);
        stm32f4xx_i2c_stop_generation(src);
    }else
    {
        src->state = STM32F4xx_I2C_STATE_RECEIVING;
    }
    
    src->i2c_dr = ret;
}


static bool __stm32f4xx_i2c_sr1_non_writeable_bits_condition( STM32F4XXI2CState* src , uint64_t value )
{
    return
    (  
        ( (src->i2c_sr1 & STM_I2C_TXE_BIT)         != (value & STM_I2C_TXE_BIT)        )
        || ( (src->i2c_sr1 & STM_I2C_RXNE_BIT)        != (value & STM_I2C_RXNE_BIT)       )
        || ( (src->i2c_sr1 & STM32F4xx_I2C_SR1_RES1)  != (value & STM32F4xx_I2C_SR1_RES1) )
        || ( (src->i2c_sr1 & STM_I2C_STOPF_BIT)       != (value & STM_I2C_STOPF_BIT)      )
        || ( (src->i2c_sr1 & STM_I2C_ADD10_BIT)       != (value & STM_I2C_ADD10_BIT)      )
        || ( (src->i2c_sr1 & STM_I2C_BTF_BIT)         != (value & STM_I2C_BTF_BIT)        )
        || ( (src->i2c_sr1 & STM_I2C_ADDR_BIT)        != (value & STM_I2C_ADDR_BIT)       )
        || ( (src->i2c_sr1 & STM_I2C_SB_BIT)          != (value & STM_I2C_SB_BIT)         )
    );

}

static void __stm32f4xx_i2c_sr1_non_writeable_bits_acctions( STM32F4XXI2CState* src , uint64_t* value )
{
    uint64_t temp = *value;

    temp = ( (src->i2c_sr1 & STM_I2C_TXE_BIT)         != 0 ? (*value | (uint64_t)STM_I2C_TXE_BIT)        : (*value & ~((uint64_t)STM_I2C_TXE_BIT)       ) );
    temp = ( (src->i2c_sr1 & STM_I2C_RXNE_BIT)        != 0 ? (*value | (uint64_t)STM_I2C_RXNE_BIT)       : (*value & ~((uint64_t)STM_I2C_RXNE_BIT)      ) );
    temp = ( (src->i2c_sr1 & STM32F4xx_I2C_SR1_RES1)  != 0 ? (*value | (uint64_t)STM32F4xx_I2C_SR1_RES1) : (*value & ~((uint64_t)STM32F4xx_I2C_SR1_RES1)) );
    temp = ( (src->i2c_sr1 & STM_I2C_STOPF_BIT)       != 0 ? (*value | (uint64_t)STM_I2C_STOPF_BIT)      : (*value & ~((uint64_t)STM_I2C_STOPF_BIT)     ) );
    temp = ( (src->i2c_sr1 & STM_I2C_ADD10_BIT)       != 0 ? (*value | (uint64_t)STM_I2C_ADD10_BIT)      : (*value & ~((uint64_t)STM_I2C_ADD10_BIT)     ) );
    temp = ( (src->i2c_sr1 & STM_I2C_BTF_BIT)         != 0 ? (*value | (uint64_t)STM_I2C_BTF_BIT)        : (*value & ~((uint64_t)STM_I2C_BTF_BIT)       ) );
    temp = ( (src->i2c_sr1 & STM_I2C_ADDR_BIT)        != 0 ? (*value | (uint64_t)STM_I2C_ADDR_BIT)       : (*value & ~((uint64_t)STM_I2C_ADDR_BIT)      ) );
    temp = ( (src->i2c_sr1 & STM_I2C_SB_BIT)          != 0 ? (*value | (uint64_t)STM_I2C_SB_BIT)         : (*value & ~((uint64_t)STM_I2C_SB_BIT)        ) );

    *value = temp;
}

static bool __stm32f4xx_i2c_cr1_non_writeable_bits_condition( STM32F4XXI2CState* src , uint64_t value )
{
    return
    ( 
        ( (src->i2c_cr1 & STM32F4xx_I2C_CR1_RE1)    != (value & STM32F4xx_I2C_CR1_RE1) )
        || ( (src->i2c_cr1 & STM32F4xx_I2C_CR1_RE2) != (value & STM32F4xx_I2C_CR1_RE2) )
    );
}

static void __stm32f4xx_i2c_cr1_non_writeable_bits_acctions( STM32F4XXI2CState* src , uint64_t* value )
{
    uint64_t temp = *value;

    temp = ( (src->i2c_cr1 & STM32F4xx_I2C_CR1_RE1) != 0 ? (*value | (uint64_t)STM32F4xx_I2C_CR1_RE1) : (*value & ~((uint64_t)STM32F4xx_I2C_CR1_RE1)) );
    temp = ( (src->i2c_cr1 & STM32F4xx_I2C_CR1_RE2) != 0 ? (*value | (uint64_t)STM32F4xx_I2C_CR1_RE2) : (*value & ~((uint64_t)STM32F4xx_I2C_CR1_RE2)) );

    *value = temp;
}

/**************************************************************************
    INTERNAL BEHAVIOR
**************************************************************************/
static void stm32f4xx_i2c_reset_reg(STM32F4XXI2CState* stm32f4xx)
{
    stm32f4xx->state        = STM32F4xx_I2C_STATE_DISABLED;
    stm32f4xx->slv_address  = 0xFF;
//    stm32f4xx->flg1         = false;
    flg_sb                  = false;
    flg_addr                = false;
    flg_stop                = false;

    stm32f4xx->i2c_cr1      = STM_I2C_REG_CR1_DEF;
    stm32f4xx->i2c_cr2      = STM_I2C_REG_CR2_DEF;
    stm32f4xx->i2c_oar1     = STM_I2C_REG_OAR1_DEF;
    stm32f4xx->i2c_oar2     = STM_I2C_REG_OAR2_DEF;
    stm32f4xx->i2c_dr       = STM_I2C_REG_DR_DEF;
    stm32f4xx->i2c_sr1      = STM_I2C_REG_SR1_DEF;
    stm32f4xx->i2c_sr2      = STM_I2C_REG_SR2_DEF;
    stm32f4xx->i2c_ccr      = STM_I2C_REG_CCR_DEF;
    stm32f4xx->i2c_trise    = STM_I2C_REG_TRISE_DEF;
    stm32f4xx->i2c_fltr     = STM_I2C_REG_FLTR_DEF;
}

static uint64_t stm32f4xx_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    STM32F4XXI2CState *s = opaque;
    uint64_t readval = 0x00U;
    
    printf("\nSTM32 I2C READ: addr=0x%02lx;", addr);

    if ( flg_addr && addr != STM_I2C_REG_SR2)
        flg_addr = false;

    if ( flg_sb )
        flg_sb = false;

    switch (addr) 
    {
        case STM_I2C_REG_CR1:   readval = (uint64_t)(s->i2c_cr1   ); break;
        case STM_I2C_REG_CR2:   readval = (uint64_t)(s->i2c_cr2   ); break;
        case STM_I2C_REG_OAR1:  readval = (uint64_t)(s->i2c_oar1  ); break;
        case STM_I2C_REG_OAR2:  readval = (uint64_t)(s->i2c_oar2  ); break;
        
        case STM_I2C_REG_DR:

            // Handling RXNE Cleaning
            s->i2c_sr1 &= ~STM_I2C_RXNE_BIT;

            // Handling Data Reception
            if ( __stm32f4xx_i2c_reciving_condition(s) )
                stm32f4xx_i2c_data_recive(s);
            
            readval = (uint64_t)(s->i2c_dr);

            break;
        
        case STM_I2C_REG_SR1:
            
            flg_sb      = true;
            flg_addr    = true;

            readval = (uint64_t)(s->i2c_sr1);
            
            break;
        
        case STM_I2C_REG_SR2:   
            
            // Handling ADDR Bit Clearing 
            if ( flg_addr )
            {
                flg_addr = false;
                s->i2c_sr1 &= ~STM_I2C_ADDR_BIT;
            }

            readval = (uint64_t)(s->i2c_sr2);

            break;
        
        case STM_I2C_REG_CCR:   readval = (uint64_t)(s->i2c_ccr   ); break;
        case STM_I2C_REG_TRISE: readval = (uint64_t)(s->i2c_trise ); break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR, 
                    "%s: Bad offset at 0x%" HWADDR_PRIx "\n",
                    __func__, addr);
    }

    printf(" Value read = 0x%02lx", readval);
    return readval;
}

static void stm32f4xx_i2c_write(void *opaque, hwaddr addr, uint64_t value, uint32_t size)
{
    /**
     * TODO: Implement slave mode
     * TODO: Hardware automatically sends ACK (if ACK=1).
     * TODO: Handle the clearing of TRA bit
     */
    
    STM32F4XXI2CState *s = opaque;
    
    printf("\nSTM32 I2C WRITE: addr=0x%02lx, value=0x%02lx\n", addr, value);
    printf("--> i2c_sr1 value=0x%02x\n", s->i2c_sr1);
    printf("--> i2c_sr2 value=0x%02x\n", s->i2c_sr2);

    if ( flg_sb && (addr != STM_I2C_REG_DR) )
        flg_sb = false;

    if ( flg_addr )
        flg_addr = false;
    
    switch (addr) 
    {
        case STM_I2C_REG_CR1:

            // Handle non-writeable bits
            if (__stm32f4xx_i2c_cr1_non_writeable_bits_condition(s, value))
            {
                __stm32f4xx_i2c_cr1_non_writeable_bits_acctions(s, &value);
            }

            // PE bit handling (Bit 0)
            if ( (s->i2c_cr1 & STM_I2C_PE_BIT) != (value & STM_I2C_PE_BIT) )
            {
                if (value & STM_I2C_PE_BIT) 
                {
                    // Enable the peripheral
                    s->i2c_cr1 |= STM_I2C_PE_BIT;

                    // Reset the fsm to idle state
                    s->state = STM32F4xx_I2C_STATE_IDLE;
                } 
                else 
                {
                    // Disable the peripheral
                    s->i2c_cr1 &= ~STM_I2C_PE_BIT;

                    // Set the fsm to disble state
                    s->state = STM32F4xx_I2C_STATE_DISABLED;
                    
                    /**
                     * TODO: Clear the corresponding bits/registers
                     */

                    s->i2c_sr1 = 0;
                    s->i2c_sr2 &= ~STM_I2C_MSL_BIT;

                    /**
                     * TODO: if we are in the middle of a transfer, we might need to release the bus
                     */
                }
            }

            /**
             * TODO: Handle the bits: 1,3,4,6 and 7
             */

            // START bit handling (Bit 8) - self-clearing
            if ( value & STM_I2C_START_BIT ) 
            {
                // Check if the peripheral is enable
                if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
                {
                    printf("ERROR: START requested but peripheral not enabled\n");
                    return;
                }

                if ( __stm32f4xx_i2c_re_start_condition(s) ) 
                {
                    printf("ReStart condition requested (bus busy)\n");
                    
                    //i2c_end_transfer(s->bus);

                    s->i2c_sr1 |= STM_I2C_START_BIT;    // Start Bit flag
                    s->i2c_sr1 |= STM_I2C_SB_BIT;       // Start Bit flag
                    s->i2c_sr2 |= STM_I2C_MSL_BIT;      // Master mode
                    s->i2c_sr2 |= STM_I2C_BUSY_BIT;     // Bus busy

                    s->i2c_sr1 &= ~STM_I2C_TXE_BIT;     // clear TXE Bit
                    
                    s->state = STM32F4xx_I2C_STATE_START_SENT;

                    /**
                     * TODO: handle/generate interrupt
                     */

                }
                else if ( __stm32f4xx_i2c_start_condition(s) )
                {  
                    printf("STM32 Start condition requested\n");
                    
                    s->i2c_sr1 |= STM_I2C_START_BIT;    // Start Bit flag
                    s->i2c_sr1 |= STM_I2C_SB_BIT;       // Start Bit flag
                    s->i2c_sr2 |= STM_I2C_MSL_BIT;      // Master mode
                    s->i2c_sr2 |= STM_I2C_BUSY_BIT;     // Bus busy

                    s->i2c_sr1 &= ~STM_I2C_TXE_BIT;     // clear TXE Bit

                    s->state = STM32F4xx_I2C_STATE_START_SENT;
                    
                    /**
                     * TODO: handle/generate interrupt
                     */
                }
                else 
                {
                    /**
                     * TODO: set an error flag
                     */
                }

                value &= ~STM_I2C_START_BIT;
            }
            else
            {
                value &= ~STM_I2C_START_BIT;
            }

            // STOP bit handling (Bit 9) - self-clearing
            if ( value & STM_I2C_STOP_BIT )
            {
                // Check if the peripheral is enable
                if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
                {
                    printf("ERROR: STOP requested but peripheral not enabled\n");
                    return;
                }

                printf("Stop condition requested\n");
                flg_stop = true;

                // Generate a STOP condition
            //    if ( __stm32f4xx_i2c_stop_condition(s) ) 
            //    {
            //        printf("Stop condition requested\n");
            //        
            //        flg_stop = true;
            //
            //        /**
            //         * TODO: handle/generate interrupt
            //         */
            //        
            //    } else 
            //    {
            //        /**
            //         * TODO: set an error flag
            //         */
            //    }
        
                value &= ~STM_I2C_STOP_BIT;
            }
            else
            {
                value &= ~STM_I2C_STOP_BIT;
            }

             // ACK bit handling (Bit 10)
            if ( (s->i2c_cr1 & STM_I2C_ACK_BIT) != (value & STM_I2C_ACK_BIT) ) 
            {
                // Check if the peripheral is enable
                if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
                {
                    printf("ERROR: ACK bit seted but peripheral not enabled\n");
                    return;
                }
            }
            else
            {
                //printf("ACK disabled (NACK)\n");
                //s->i2c_cr1 &= ~STM_I2C_ACK_BIT;
            }
            
            // POS bit handling (Bit 11)
            if ( (s->i2c_cr1 & STM_I2C_POS_BIT) != (value & STM_I2C_POS_BIT) ) 
            {
                // Check if the peripheral is enable
                if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
                {
                    printf("ERROR: POS bit seted but peripheral not enabled\n");
                    return;
                }
                printf("ACK position: %s\n", (value & STM_I2C_POS_BIT) ? "next byte" : "current byte");
            }
            
            /**
             * TODO: Handle the bits: 12 and 13
             */

            // SWRST bit handling (Bit 15) - self-clearing
            if (value & STM_I2C_SWRST_BIT) 
            {
                printf("Software Reset\n");
                stm32f4xx_i2c_reset_reg(s);
                return;
            }

            s->i2c_cr1 = (uint32_t)value;

            break;

        case STM_I2C_REG_CR2:   s->i2c_cr2  = value;    break;
        case STM_I2C_REG_OAR1:  s->i2c_oar1 = value;    break;
        case STM_I2C_REG_OAR2:  s->i2c_oar2 = value;    break;

        case STM_I2C_REG_DR:

            // Handling TXE Cleaning
            s->i2c_sr1 &= ~STM_I2C_TXE_BIT;
            
            // Handling RXNE Cleaning
            s->i2c_sr1 &= ~STM_I2C_RXNE_BIT;

            // Handling SB Bit Clearing 
            if ( flg_sb )
            {
                flg_sb = false;
                s->i2c_sr1 &= ~STM_I2C_SB_BIT;
            }

            // Handeling Data Sending To Slave
            if ( __stm32f4xx_i2c_transmitting_condition(s) )
                stm32f4xx_i2c_data_transfer(s, value);
            
            // Handeling Address Sending To Slave
            if ( __stm32f4xx_i2c_address_condition(s) )
                stm32f4xx_i2c_begin_transfer(s, value);
            
            s->i2c_dr = (uint32_t)value;
                        
            break;

        case STM_I2C_REG_SR1:

            // Handle non-writeable bits
            if ( __stm32f4xx_i2c_sr1_non_writeable_bits_condition(s, value) )
                __stm32f4xx_i2c_sr1_non_writeable_bits_acctions(s, &value);

            /**
             * TODO: Handle [8...15] bits
             */

            s->i2c_sr1 = (uint32_t)value;

            break;
        
        case STM_I2C_REG_SR2:

            /**
             * Only read
             */
            
            break;
        
        case STM_I2C_REG_CCR:

            /**
             * TODO: Add a clock to the struct and config it from this
             * register
             */

            s->i2c_ccr = (uint32_t)value;
            
            break;

        case STM_I2C_REG_TRISE: 
            
            s->i2c_trise = (uint32_t)value;   
            
            break;

        case STM_I2C_REG_FLTR:
            
            s->i2c_fltr = (uint32_t)value;
            
            break;

        default:

            qemu_log_mask(LOG_GUEST_ERROR, 
                "%s: Bad offset at 0x%" HWADDR_PRIx "\n",
                __func__, addr
            );

            break;
    }
}

/**************************************************************************
    DEVICE LIFE FUNCTIONS
**************************************************************************/
static Property stm32f4xx_i2c_properties[] = 
{
    DEFINE_PROP_STRING("bus-name", STM32F4XXI2CState, bus_name),
};

static void stm32f4xx_i2c_reset(DeviceState *dev)
{
    STM32F4XXI2CState* src = STM32F4XX_I2C(dev);
    stm32f4xx_i2c_reset_reg(src);
}

static const VMStateDescription vmstate_stm32f2xx_i2c = 
{
    .name = TYPE_STM32F4XX_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[])
    {
        VMSTATE_UINT32(i2c_cr1,     STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_cr2,     STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_oar1,    STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_oar2,    STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_dr,      STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_sr1,     STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_sr2,     STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_ccr,     STM32F4XXI2CState),
        VMSTATE_UINT32(i2c_trise,   STM32F4XXI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static const MemoryRegionOps stm32f4xx_i2c_ops = 
{
    .read   = stm32f4xx_i2c_read,
    .write  = stm32f4xx_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32f4xx_i2c_realize(DeviceState *dev, Error **errp)
{
    printf("\n\n Realizing STM32F4XX I2C controller\n");

    STM32F4XXI2CState   *stm32  = STM32F4XX_I2C(dev);
    SysBusDevice        *sbd    = SYS_BUS_DEVICE(dev);

    if (!stm32->bus_name) 
    {
        stm32->bus_name = g_strdup("i2c");
    }

    printf("\n stm32->bus_name = %s \n", stm32->bus_name );
    stm32->bus = i2c_init_bus(dev, stm32->bus_name);
    
    memory_region_init_io
    (
        &stm32->iomem, OBJECT(dev), 
        &stm32f4xx_i2c_ops, stm32,
        TYPE_STM32F4XX_I2C, 0x400
    );

    sysbus_init_mmio(sbd, &stm32->iomem);
    
    /* Initialize interrupt lines */
    sysbus_init_irq(sbd, &stm32->irq_event);
    sysbus_init_irq(sbd, &stm32->irq_error);


    printf("I2C bus created: %p\n", stm32->bus);
    //printf("I2C bus name: %s\n", stm32->bus->name);

    if (!stm32->bus) {
        error_setg(errp, "stm32f4xx_i2c: I2C bus not initialized");
        return;
    }

    printf("STM32F4XX I2C realized successfully\n");
}


/**************************************************************************
    LIS3DH REGISTRATION IN QEMU 
**************************************************************************/
static void stm32f4xx_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_props(dc, stm32f4xx_i2c_properties);
    device_class_set_legacy_reset(dc, stm32f4xx_i2c_reset);
    dc->realize = stm32f4xx_i2c_realize;
    dc->vmsd    = &vmstate_stm32f2xx_i2c;
}

static const TypeInfo stm32f4xx_i2c_info = 
{
    .name           = TYPE_STM32F4XX_I2C,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(STM32F4XXI2CState),
    // .instance_init  = stm32f4xx_i2c_init,
    .class_init     = stm32f4xx_i2c_class_init,
};

static void stm32f4xx_i2c_register_types(void)
{
    type_register_static(&stm32f4xx_i2c_info);
}

type_init(stm32f4xx_i2c_register_types)
