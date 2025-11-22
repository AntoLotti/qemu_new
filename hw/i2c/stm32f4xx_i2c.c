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


/**
 * TODO: 
 * - In The address phase, set the ADDR bit when reciving a ACK 
 * - Transmision more than one bit
 */

static void stm32f4xx_i2c_reset_reg(STM32F4XXI2CState* stm32f4xx);

/** ============================================================================
 *                      INTERRUPT GENERATION FUNCTIONS
 * =============================================================================
 */

static void stm32f4xx_i2c_update_irq(STM32F4XXI2CState *s)
{
    uint32_t event_irq_mask = 0;
    uint32_t error_irq_mask = 0;

    // Event interrupts - only trigger if ITEVTEN is set
    if ((s->i2c_cr2 & STM_I2C_ITEVTEN_BIT))
    {
        if (s->i2c_sr1 & STM_I2C_SB_BIT)
            event_irq_mask |= 1;
        
        if (s->i2c_sr1 & STM_I2C_ADDR_BIT)
            event_irq_mask |= 1;

        if (s->i2c_sr1 & STM_I2C_ADD10_BIT)
            event_irq_mask |= 1;
        
        if (s->i2c_sr1 & STM_I2C_BTF_BIT)
            event_irq_mask |= 1;

        if (s->i2c_sr1 & STM_I2C_STOPF_BIT)
            event_irq_mask |= 1;

        // Buffer interrupts - only trigger if ITBUFEN is set
        if ((s->i2c_cr2 & STM_I2C_ITBUFEN_BIT)) 
        {
            if (s->i2c_sr1 & STM_I2C_TXE_BIT)
                event_irq_mask |= 1;
            if (s->i2c_sr1 & STM_I2C_RXNE_BIT)
                event_irq_mask |= 1;
        }
    }

    // Error interrupts - only trigger if ITERREN is set
    if ((s->i2c_cr2 & STM_I2C_ITERREN_BIT)) 
    {
        if (s->i2c_sr1 & STM_I2C_BERR_BIT)
            error_irq_mask |= 1;

        if (s->i2c_sr1 & STM_I2C_ARLO_BIT)
            error_irq_mask |= 1;
        
        if (s->i2c_sr1 & STM_I2C_AF_BIT)
            error_irq_mask |= 1;
    
        if (s->i2c_sr1 & STM_I2C_OVR_BIT)
            error_irq_mask |= 1;
    
        if (s->i2c_sr1 & STM_I2C_PECERR_BIT)
            error_irq_mask |= 1;
    
        if (s->i2c_sr1 & STM_I2C_TIMEOUT_BIT)
            error_irq_mask |= 1;
    
        if (s->i2c_sr1 & STM_I2C_SMBALERT_BIT)
            error_irq_mask |= 1;
    }

    // Set IRQ lines
    qemu_set_irq(s->irq_event, event_irq_mask);
    qemu_set_irq(s->irq_error, error_irq_mask);
}

static void stm32f4xx_i2c_set_status_flag(STM32F4XXI2CState *s, uint32_t flag)
{
    s->i2c_sr1 |= flag;
    stm32f4xx_i2c_update_irq(s);
}

static void stm32f4xx_i2c_clear_status_flag(STM32F4XXI2CState *s, uint32_t flag)
{
    s->i2c_sr1 &= ~flag;
    stm32f4xx_i2c_update_irq(s);
}


/** ============================================================================
 *                        INTERNAL BUFFER FUNCTIONS
 * =============================================================================
 */




/** ============================================================================
 *                              I2C FUNCTIONS
 * =============================================================================
 */

static bool i2c_start_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_IDLE) 
        || (src->state == STM32F4xx_I2C_STATE_TRANSMITTING )
        || (src->i2c_sr2 & STM_I2C_BUSY_BIT) != 0 
    );
}

static bool i2c_transmitter_stop_condition(STM32F4XXI2CState* src)
{
    printf("STM32: flg_stop = %s\n", src->flags.flg_stop ? "TRUE" : "FALSE");
    printf("src->i2c_cr1 & STM_I2C_ACK_BIT = 0x%x\n", (src->i2c_cr1 & STM_I2C_ACK_BIT));

    return
    ( 
        ( (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_WRITE)
            || (src->state == STM32F4xx_I2C_STATE_TRANSMITTING) )
        && ( (src->i2c_cr1 & STM_I2C_ACK_BIT) == 0 )
        && src->flags.flg_stop
    );
}

static bool i2c_receiver_stop_condition(STM32F4XXI2CState* src)
{
    printf("STM32 I2C: flg_stop = %s\n", src->flags.flg_stop ? "TRUE" : "FALSE");
    printf("STM32 I2C: src->i2c_cr1 & STM_I2C_ACK_BIT = 0x%x\n", (src->i2c_cr1 & STM_I2C_ACK_BIT));

    return
    ( 
        ((src->state == STM32F4xx_I2C_STATE_ADDR_SENT_READ) ||
         (src->state == STM32F4xx_I2C_STATE_RECEIVING)) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0) &&
        src->flags.flg_stop
    );
}

static bool i2c_transmitting_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_WRITE) 
        || (src->state == STM32F4xx_I2C_STATE_TRANSMITTING)
    );
}

static bool i2c_reciving_condition(STM32F4XXI2CState* src)
{
    return
    ( 
        (src->state == STM32F4xx_I2C_STATE_ADDR_SENT_READ)
        || (src->state == STM32F4xx_I2C_STATE_RECEIVING)
    );
}


static void stm32f4xx_i2c_begin_communication(STM32F4XXI2CState* src)
{
   
    uint8_t address = (uint8_t)(extract32(src->i2c_dr, 1, 7));
    uint8_t mode    = (uint8_t)(extract32(src->i2c_dr, 0, 1));

    printf("STM I2C: Address: 0x%02x, Mode=%s\n",
                  address, mode ? "READ" : "WRITE");
 
    if ( i2c_start_transfer( src->bus, address, mode ) )
    {
        printf("Error during i2c_start_transfer \n");
        src->i2c_sr1 |= STM_I2C_AF_BIT;  /* Set Acknowledge Failure flag */
        //
        return;
    }

    src->slv_address = address;

    printf("All ok during i2c_start_transfer \n");

    src->slv_address = address;

    src->i2c_sr2 = 
    ( (mode == 0) ? (src->i2c_sr2 & ~STM_I2C_TRA_BIT) : (src->i2c_sr2 | STM_I2C_TRA_BIT) );

    src->i2c_sr1 |= STM_I2C_ADDR_BIT;
    src->i2c_dr   = 0U;

    if ( mode == 0 )
    {
        printf("\n STM32F4xx_I2C_STATE_ADDR_SENT_WRITE \n");
        src->i2c_sr1 |= STM_I2C_TXE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
        src->state = STM32F4xx_I2C_STATE_ADDR_SENT_WRITE;
    }
    else
    {
        printf("\n STM32F4xx_I2C_STATE_ADDR_SENT_READ \n");        
        src->i2c_sr1 |= STM_I2C_RXNE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
        src->state = STM32F4xx_I2C_STATE_ADDR_SENT_READ;
    }
}

static void stm32f4xx_i2c_stop_generation(STM32F4XXI2CState* src)
{
    src->i2c_sr2 &= ~STM_I2C_MSL_BIT;
    src->i2c_sr2 &= ~STM_I2C_BUSY_BIT;

    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_BTF_BIT);
    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_TXE_BIT);
    
    src->state = STM32F4xx_I2C_STATE_IDLE;  // Change state to IDLE
    
    src->flags.flg_sb   = false;
    src->flags.flg_stop = false;
    src->flags.flg_addr = false;
}

static void stm32f4xx_i2c_transferring_data(STM32F4XXI2CState *src)
{
    if ( i2c_send(src->bus, src->i2c_dr) )
    {
        printf("\n Error during i2c_send \n");
    }
    else
    {
        printf("\n All ok during i2c_send \n");
        src->i2c_sr1 |= STM_I2C_TXE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
        src->i2c_dr   = 0U;
        src->state = STM32F4xx_I2C_STATE_TRANSMITTING;
    }
}

static void stm32f4xx_i2c_data_recive(STM32F4XXI2CState *src)
{
    uint8_t ret = i2c_recv(src->bus);
        
    if ( i2c_receiver_stop_condition(src) )
    {
        printf("STM32 Stop Condition\n");

        i2c_nack(src->bus);
        i2c_end_transfer(src->bus);
        stm32f4xx_i2c_stop_generation(src);
    }
    else
    {
        // 

        src->i2c_sr1 |= STM_I2C_RXNE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
        src->state = STM32F4xx_I2C_STATE_RECEIVING;
    }
    
    src->i2c_dr = ret;
}


/**
 * ============================================================================
 *              REGISTER READ/WRITE HANDLERS
 * ============================================================================ 
 */

static void __i2c_write_cr1(STM32F4XXI2CState *s, uint64_t value)
{
    // PE bit handling (Bit 0)
    if ( (s->i2c_cr1 & STM_I2C_PE_BIT) != (value & STM_I2C_PE_BIT) )
    {
        if (value & STM_I2C_PE_BIT) 
        {
            s->i2c_cr1 |= STM_I2C_PE_BIT;
            s->state = STM32F4xx_I2C_STATE_IDLE;
        } 
        else 
        {
            s->i2c_cr1 &= ~STM_I2C_PE_BIT;
            s->i2c_sr1 = 0;
            s->i2c_sr2 &= ~STM_I2C_MSL_BIT;
            s->state = STM32F4xx_I2C_STATE_DISABLED;
        }
    }

    /**
     * TODO: Handle the bits: 1,3,4,6 and 7
     */

    // START bit handling (Bit 8) - self-clearing
    if ( value & STM_I2C_START_BIT ) 
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
        {
            printf("ERROR: START requested but peripheral not enabled\n");
            return;
        }

        if (i2c_start_condition(s)) 
        {
            
            s->i2c_sr2 |= STM_I2C_MSL_BIT;
            s->i2c_sr2 |= STM_I2C_BUSY_BIT;
                        
            stm32f4xx_i2c_set_status_flag(s, STM_I2C_SB_BIT);
            stm32f4xx_i2c_set_status_flag(s, STM_I2C_BTF_BIT);
            
            stm32f4xx_i2c_clear_status_flag(s, STM_I2C_TXE_BIT);
            
            s->state = STM32F4xx_I2C_STATE_START_SENT;

            qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: START condition sent\n");
        }

        value &= ~STM_I2C_START_BIT;
    }

    // STOP bit handling (Bit 9) - self-clearing
    if ( value & STM_I2C_STOP_BIT )
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
        {
            printf("ERROR: STOP requested but peripheral not enabled\n");
            return;
        }

        printf("Stop condition requested\n");
        s->flags.flg_stop = true;

        // Generate a transmision STOP condition
        if (i2c_transmitter_stop_condition(s))
        {
            stm32f4xx_i2c_stop_generation(s);
        }
        

        value &= ~STM_I2C_STOP_BIT;
    }

    // ACK bit handling (Bit 10)
    if ( (s->i2c_cr1 & STM_I2C_ACK_BIT) != (value & STM_I2C_ACK_BIT) ) 
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT))
        {
            printf("ERROR: ACK bit seted but peripheral not enabled\n");
            return;
        }

        if (value & STM_I2C_ACK_BIT) 
        {
            s->i2c_cr1 |= STM_I2C_ACK_BIT;
            printf("\nSTM32 I2C: ACK enabled\n");
        }
        else 
        {
            s->i2c_cr1 &= ~STM_I2C_ACK_BIT;
            printf("\nSTM32 I2C: NACK enabled\n");            
        }
    }
    
    // POS bit handling (Bit 11)
    if ( (s->i2c_cr1 & STM_I2C_POS_BIT) != (value & STM_I2C_POS_BIT) ) 
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT))
        {
            printf("ERROR: POS bit seted but peripheral not enabled\n");
            return;
        }
        
        printf("STM32 I2C: POS bit set\n");

        if (value & STM_I2C_POS_BIT) 
        {
            s->i2c_cr1 |= STM_I2C_POS_BIT;
        }
        else 
        {
            s->i2c_cr1 &= ~STM_I2C_POS_BIT;
        }
    }
    
    /**
     * TODO: Handle the bits: 12 and 13
     */

    // SWRST bit handling (Bit 15) - self-clearing
    if (value & STM_I2C_SWRST_BIT) 
    {
        printf("STM32 I2C: Software Reset\n");
        
        stm32f4xx_i2c_reset_reg(s);

        return;
    }

    s->i2c_cr1 = (uint32_t)value;

}


static void __i2c_write_cr2(STM32F4XXI2CState *s, uint64_t value)
{
    s->i2c_cr2 = value;
    stm32f4xx_i2c_update_irq(s);
}


static uint64_t __i2c_read_dr(STM32F4XXI2CState *s)
{
    // Handling BTF Bit Cleaning
    stm32f4xx_i2c_clear_status_flag(s, STM_I2C_BTF_BIT);

    // Handling RXNE Bit Cleaning
    stm32f4xx_i2c_clear_status_flag(s, STM_I2C_RXNE_BIT);

    // Handling Data Reception
    if (i2c_reciving_condition(s))
        stm32f4xx_i2c_data_recive(s);
    
    return (uint64_t)(s->i2c_dr);
}

static void __i2c_write_dr(STM32F4XXI2CState *s, uint64_t value)
{
    s->i2c_dr = (uint32_t)value & 0xFF;

    /* Clear flags when DR is written */
    stm32f4xx_i2c_clear_status_flag(s, STM_I2C_BTF_BIT);
    stm32f4xx_i2c_clear_status_flag(s, STM_I2C_TXE_BIT);
    stm32f4xx_i2c_clear_status_flag(s, STM_I2C_RXNE_BIT);

    /* Handle SB flag clearing */
    if ((s->i2c_sr1 & STM_I2C_SB_BIT) && s->flags.flg_sb == true) 
    {
        printf("STM32: SB flag cleared after DR write\n");
        s->flags.flg_sb = false;
        stm32f4xx_i2c_clear_status_flag(s, STM_I2C_SB_BIT);
    }

    // Handeling Data Transmision
    if (s->state == STM32F4xx_I2C_STATE_START_SENT) 
    {
        stm32f4xx_i2c_begin_communication(s);
    } 
    else if (i2c_transmitting_condition(s))
    {   
        stm32f4xx_i2c_transferring_data(s);
    }
}


static uint64_t __i2c_read_sr1(STM32F4XXI2CState *s)
{    
    s->flags.flg_sb     = true;
    s->flags.flg_addr   = true;

    return (uint64_t)(s->i2c_sr1);
}


static uint64_t __i2c_read_sr2(STM32F4XXI2CState *s)
{
    // Handling ADDR Bit Clearing 
    if ( s->flags.flg_addr )
    {
        s->flags.flg_addr = false;
        s->i2c_sr1 &= ~STM_I2C_ADDR_BIT;
    }

    return (uint64_t)(s->i2c_sr2);

}

/**
 * ============================================================================
 *              QEMU REGISTER ACCESS HANDLERS
 * ============================================================================ 
 */

static uint64_t stm32f4xx_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    STM32F4XXI2CState *s = opaque;
    uint64_t result = 0x00UL;

    if ( s->flags.flg_addr && addr != STM_I2C_REG_SR2)
        s->flags.flg_addr = false;

    if ( s->flags.flg_sb )
        s->flags.flg_sb = false;

    switch (addr) 
    {
        case STM_I2C_REG_CR1:   result = s->i2c_cr1;        break;
        case STM_I2C_REG_CR2:   result = s->i2c_cr2;        break;
        case STM_I2C_REG_OAR1:  result = s->i2c_oar1;       break;
        case STM_I2C_REG_OAR2:  result = s->i2c_oar2;       break;
        case STM_I2C_REG_DR:    result = __i2c_read_dr(s);  break;
        case STM_I2C_REG_SR1:   result = __i2c_read_sr1(s); break;
        case STM_I2C_REG_SR2:   result = __i2c_read_sr2(s); break;
        case STM_I2C_REG_CCR:   result = s->i2c_ccr;        break;
        case STM_I2C_REG_TRISE: result = s->i2c_trise;      break;
        case STM_I2C_REG_FLTR:  result = s->i2c_fltr;       break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: Bad read offset 0x%lx\n", addr);
    }

    return result;
}

static void stm32f4xx_i2c_write(void *opaque, hwaddr addr, uint64_t value, uint32_t size)
{
    STM32F4XXI2CState *s = opaque;

    switch (addr) 
    {
        case STM_I2C_REG_CR1:   __i2c_write_cr1(s, value);        break;
        case STM_I2C_REG_CR2:   __i2c_write_cr2(s, value);        break;
        case STM_I2C_REG_OAR1:  s->i2c_oar1 = (uint32_t)value;  break;
        case STM_I2C_REG_OAR2:  s->i2c_oar2 = (uint32_t)value;  break;
        case STM_I2C_REG_DR:    __i2c_write_dr(s, value);         break;
        case STM_I2C_REG_SR1:   s->i2c_sr1  = (uint32_t)value;  break;
        case STM_I2C_REG_SR2:
            /* SR2 is generally read-only */
            break;
        case STM_I2C_REG_CCR:   s->i2c_ccr      = (uint32_t)value;  break;
        case STM_I2C_REG_TRISE: s->i2c_trise    = (uint32_t)value;  break;
        case STM_I2C_REG_FLTR:  s->i2c_fltr     = (uint32_t)value;  break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: Bad write offset 0x%lx\n", addr);
    }
}


/**
 * ============================================================================
 *              DEVICE LIFE FUNCTIONS
 * ============================================================================ 
 */

static void stm32f4xx_i2c_reset_reg(STM32F4XXI2CState* stm32f4xx)
{
    stm32f4xx->state            = STM32F4xx_I2C_STATE_DISABLED;
    stm32f4xx->slv_address      = 0xFF;
    stm32f4xx->flags.flg_sb     = false;
    stm32f4xx->flags.flg_stop   = false;
    stm32f4xx->flags.flg_addr   = false;

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
