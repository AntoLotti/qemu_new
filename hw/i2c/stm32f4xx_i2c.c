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

/** ============================================================================
 *                              DEFINITIONS
 * =============================================================================
 */

/* Debug control */
#define DEBUG_I2C 1

#ifdef DEBUG_I2C

#define STM32_DEBUG(text, ...) \
    printf("STM32 I2C: " text "\n", ## __VA_ARGS__ )
#else
#define DPRINTF_BUFFER(fmt, ...) do {} while(0)

#endif

static void stm32f4xx_i2c_reset_reg(STM32F4XXI2CState* stm32f4xx);

enum operation_e
{
    ops_na = 0, // No operation
    ops_w = 1,  // Write
    ops_r = 2,  // Read    
};

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
 *                              I2C FSM FUNCTIONS
 * =============================================================================
 */

static stm32f4xx_i2c_fsm_t *stm32f4xx_i2c_fsm_new(stm32f4xx_i2c_fsm_trans_t *const src)
{
	stm32f4xx_i2c_fsm_t *ret = (stm32f4xx_i2c_fsm_t*)g_malloc(sizeof(stm32f4xx_i2c_fsm_t));

	if(ret != NULL)
    {
		ret->trans_table = src;
	    ret->act_st = src[0].org_st;
        STM32_DEBUG("FSM created, initial state: %d", ret->act_st);
    } else 
    {
        STM32_DEBUG("ERROR: Failed to allocate FSM!");
    }

	return ret;
}

static void stm32f4xx_i2c_fsm_fire(STM32F4XXI2CState *src)
{
    STM32_DEBUG("Inside fsm fire");

    if (src->fsm->trans_table == NULL)
    {
        STM32_DEBUG("ERROR: FSM table is NULL!\n");
        return;
    }
    
    STM32_DEBUG("Inside fsm fire, state: %d", src->fsm->act_st);
    stm32f4xx_i2c_fsm_trans_t *t = src->fsm->trans_table;
    
    while (t->org_st >= 0 && t->condition != NULL)
    {
        if (t->condition == NULL)
        {
            STM32_DEBUG("Condition NUL");
            continue;  // Skip invalid entries
        }
        
        //STM32_DEBUG("Inside fsm fire for loop, t->org_st = %d", t->org_st);

        if ((src->fsm->act_st == t->org_st) && t->condition(src))
        {
            STM32_DEBUG("FSM Transition: %d -> %d\n", t->org_st, t->dst_st);
            src->fsm->act_st = t->dst_st;

            if (t->output)
                t->output(src);

            break;
        }

        t++;
    }
}

/** ============================================================================
 *                              I2C FSM CONDITIONS
 * =============================================================================
 */

static bool stm32f4xx_i2c_fsm_condition_DISABLED_to_IDLE(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_CR1) &&
        ((src->i2c_cr1 & STM_I2C_PE_BIT) != 0 )
    );
}

static bool stm32f4xx_i2c_fsm_condition_IDLE_to_START(STM32F4XXI2CState  *src)
{
    bool ret = (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_CR1) &&
        (src->flags.flg_start);
        //(src->flags.flg_start) &&
        //((src->i2c_cr1 & STM_I2C_ACK_BIT) != 0 );

    STM32_DEBUG("IDLE to START condition: %s", ret ? "TRUE" : "FALSE");

    return
    (
        ret
    );
}

static bool stm32f4xx_i2c_fsm_condition_START_to_S_WRITE(STM32F4XXI2CState  *src)
{
    uint8_t is_recv = (uint8_t)(extract32(src->i2c_dr, 0, 1));

    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_DR) &&
        (is_recv == 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_S_WRITE_to_TRANS(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_DR)
    );
}

static bool stm32f4xx_i2c_fsm_condition_TRANS_to_START(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_CR1) &&
        (src->flags.flg_start)
    );
}

static bool stm32f4xx_i2c_fsm_condition_TRANS_to_TRANS(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_DR)
    );
}

static bool stm32f4xx_i2c_fsm_condition_TRANS_to_IDLE(STM32F4XXI2CState *src)
{
    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_CR1) &&
        (src->flags.flg_stop)
    );
}

static bool stm32f4xx_i2c_fsm_condition_START_to_S_READ(STM32F4XXI2CState  *src)
{
    uint8_t is_recv = (uint8_t)(extract32(src->i2c_dr, 0, 1));

    return
    (
        (src->ops == ops_w) &&
        (src->addr == STM_I2C_REG_DR) &&
        (is_recv == 1)
    );
}

static bool stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_1B(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        (src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0) &&
        ((src->i2c_sr1 & STM_I2C_ADDR_BIT) == 0) &&
        ((src->i2c_cr1 & STM_I2C_POS_BIT) == 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_2BF(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        (src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0) &&
        ((src->i2c_sr1 & STM_I2C_ADDR_BIT) == 0) &&
        ((src->i2c_cr1 & STM_I2C_POS_BIT) != 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_S_READ_to_R_3B(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        !(src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0) &&
        ((src->i2c_sr1 & STM_I2C_ADDR_BIT) == 0) &&
        ((src->i2c_cr1 & STM_I2C_POS_BIT) == 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_NB(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        !(src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) != 0) &&
        ((src->i2c_sr1 & STM_I2C_ADDR_BIT) == 0) &&
        ((src->i2c_cr1 & STM_I2C_POS_BIT) == 0)    
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_2BF_to_RECV_2BS(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        (src->flags.flg_stop)
    );
}

static bool stm32f4xx_i2c_fsm_condition_R_3B_to_RECV_2BF(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        (src->flags.flg_stop)
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_NB_to_RECV_NB(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        !(src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) != 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_NB_to_R_3B(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        !(src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_NB_to_RECV_2BF(STM32F4XXI2CState  *src)
{
    return
    (
        (src->ops == ops_r) &&
        (src->addr == STM_I2C_REG_DR) &&
        (src->flags.flg_stop) &&
        ((src->i2c_cr1 & STM_I2C_ACK_BIT) == 0)
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_1B_to_IDLE(STM32F4XXI2CState *src)
{
    return
    (
        (src->flags.flg_stop)
    );
}

static bool stm32f4xx_i2c_fsm_condition_RECV_2BS_to_IDLE(STM32F4XXI2CState *src)
{
    return
    (
        (src->flags.flg_stop)
    );
}


/** ============================================================================
 *                              I2C FSM OUTPUTS
 * =============================================================================
 */
static void stm32f4xx_i2c_begin_communication(STM32F4XXI2CState *src)
{
    src->ops    = ops_na;
    src->addr   = 0xFF;

    uint8_t address = (uint8_t)(extract32(src->i2c_dr, 1, 7));
    uint8_t is_recv = (uint8_t)(extract32(src->i2c_dr, 0, 1));

    STM32_DEBUG(": Address: 0x%02x, Mode=%s\n",
                  address, is_recv ? "READ" : "WRITE");
 
    if ( i2c_start_transfer( src->bus, address, is_recv ) )
    {
        STM32_DEBUG("Error during i2c_start_transfer \n");
        src->i2c_sr1 |= STM_I2C_AF_BIT;
        return;
    }

    src->slv_address = address;

    STM32_DEBUG("All ok during i2c_start_transfer \n");

    src->slv_address = address;

    src->i2c_sr2 = 
    ( (is_recv == 1) ? (src->i2c_sr2 & ~STM_I2C_TRA_BIT) : (src->i2c_sr2 | STM_I2C_TRA_BIT) );

    src->i2c_sr1 |= STM_I2C_ADDR_BIT;
    src->i2c_dr   = 0U;

    if ( is_recv == 0 )
    {
        STM32_DEBUG("\n STM32F4XX_I2C_SENT_WRITE \n");
        src->i2c_sr1 |= STM_I2C_TXE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
    }
    else if ( is_recv == 1 )    
    {
        STM32_DEBUG("\n STM32F4XX_I2C_SENT_READ \n");        
        src->i2c_sr1 |= STM_I2C_RXNE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
    }
}

static void stm32f4xx_i2c_data_transfer(STM32F4XXI2CState *src)
{
    src->ops    = ops_na;
    src->addr   = 0xFF;

    if ( i2c_send(src->bus, src->i2c_dr) )
    {
        STM32_DEBUG("\n Error during i2c_send \n");
    }
    else
    {
        STM32_DEBUG("\n All ok during i2c_send \n");
        src->i2c_sr1 |= STM_I2C_TXE_BIT;
        src->i2c_sr1 |= STM_I2C_BTF_BIT;
        src->i2c_dr   = 0U;
    }
}

static void stm32f4xx_i2c_data_reception(STM32F4XXI2CState *src)
{
    src->i2c_dr = (uint32_t)i2c_recv(src->bus);

    STM32_DEBUG("Data recived: 0x%02x", src->i2c_dr );
        
    src->i2c_sr1 |= STM_I2C_RXNE_BIT;
    src->i2c_sr1 |= STM_I2C_BTF_BIT;
}

static void stm32f4xx_i2c_stop_generation(STM32F4XXI2CState *src)
{
    src->ops    = ops_na;
    src->addr   = 0xFF;

    src->i2c_sr2 &= ~STM_I2C_MSL_BIT;
    src->i2c_sr2 &= ~STM_I2C_BUSY_BIT;

    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_BTF_BIT);
    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_TXE_BIT);
        
    src->flags.flg_sb   = false;
    src->flags.flg_stop = false;
    src->flags.flg_addr = false;

    i2c_nack(src->bus);
    i2c_end_transfer(src->bus);
}

static void stm32f4xx_i2c_last_data_reception(STM32F4XXI2CState *src)
{
    stm32f4xx_i2c_data_reception(src);
    stm32f4xx_i2c_fsm_fire(src);
}

static void stm32f4xx_i2c_fsm_output_DISABLED_to_IDLE(STM32F4XXI2CState  *src)
{
    (void)src;
}

static void stm32f4xx_i2c_fsm_output_IDLE_to_START(STM32F4XXI2CState  *src)
{
    STM32_DEBUG("\n OUTPUT IDLE to START \n");

    src->ops    = ops_na;
    src->addr   = 0xFF;

    src->flags.flg_start = false;

    src->i2c_sr2 |= STM_I2C_MSL_BIT;
    src->i2c_sr2 |= STM_I2C_BUSY_BIT;
    
    stm32f4xx_i2c_set_status_flag(src, STM_I2C_SB_BIT);
    stm32f4xx_i2c_set_status_flag(src, STM_I2C_BTF_BIT);
    
    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_TXE_BIT);
    
    qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: START condition sent\n");
}

static void stm32f4xx_i2c_fsm_output_TRANS_to_START(STM32F4XXI2CState  *src)
{
    src->ops    = ops_na;
    src->addr   = 0xFF;

    src->flags.flg_start = false;

    src->i2c_sr2 |= STM_I2C_MSL_BIT;
    src->i2c_sr2 |= STM_I2C_BUSY_BIT;
    
    stm32f4xx_i2c_set_status_flag(src, STM_I2C_SB_BIT);
    stm32f4xx_i2c_set_status_flag(src, STM_I2C_BTF_BIT);
    
    stm32f4xx_i2c_clear_status_flag(src, STM_I2C_TXE_BIT);
    
    qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: START condition sent\n");
}


stm32f4xx_i2c_fsm_trans_t stm32f4xx_i2c_fsm_transition_table[TRANSITIONS]= 
{
    { STM32F4XX_I2C_DISABLED,               stm32f4xx_i2c_fsm_condition_DISABLED_to_IDLE,       STM32F4XX_I2C_IDLE,                 stm32f4xx_i2c_fsm_output_DISABLED_to_IDLE   },
    { STM32F4XX_I2C_IDLE,                   stm32f4xx_i2c_fsm_condition_IDLE_to_START,          STM32F4XX_I2C_START,                stm32f4xx_i2c_fsm_output_IDLE_to_START      },
    { STM32F4XX_I2C_START,                  stm32f4xx_i2c_fsm_condition_START_to_S_WRITE,       STM32F4XX_I2C_SENT_WRITE,           stm32f4xx_i2c_begin_communication           },
    { STM32F4XX_I2C_SENT_WRITE,             stm32f4xx_i2c_fsm_condition_S_WRITE_to_TRANS,       STM32F4XX_I2C_TRANSMITTING,         stm32f4xx_i2c_data_transfer                 },
    { STM32F4XX_I2C_TRANSMITTING,           stm32f4xx_i2c_fsm_condition_TRANS_to_TRANS,         STM32F4XX_I2C_TRANSMITTING,         stm32f4xx_i2c_data_transfer                 },
    { STM32F4XX_I2C_TRANSMITTING,           stm32f4xx_i2c_fsm_condition_TRANS_to_START,         STM32F4XX_I2C_START,                stm32f4xx_i2c_fsm_output_TRANS_to_START     },
    { STM32F4XX_I2C_TRANSMITTING,           stm32f4xx_i2c_fsm_condition_TRANS_to_IDLE,          STM32F4XX_I2C_IDLE,                 stm32f4xx_i2c_stop_generation               },
    { STM32F4XX_I2C_START,                  stm32f4xx_i2c_fsm_condition_START_to_S_READ,        STM32F4XX_I2C_SENT_READ,            stm32f4xx_i2c_begin_communication           },
    { STM32F4XX_I2C_SENT_READ,              stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_1B,      STM32F4XX_I2C_RECV_1BYTE,           stm32f4xx_i2c_last_data_reception           },
    { STM32F4XX_I2C_SENT_READ,              stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_2BF,     STM32F4XX_I2C_RECV_2BYTES_FIRST,    stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_SENT_READ,              stm32f4xx_i2c_fsm_condition_S_READ_to_R_3B,         STM32F4XX_I2C_RECV_3BYTES,          stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_SENT_READ,              stm32f4xx_i2c_fsm_condition_S_READ_to_RECV_NB,      STM32F4XX_I2C_RECV_NBYTES,          stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_RECV_2BYTES_FIRST,      stm32f4xx_i2c_fsm_condition_RECV_2BF_to_RECV_2BS,   STM32F4XX_I2C_RECV_2BYTES_SECOND,   stm32f4xx_i2c_last_data_reception           },    
    { STM32F4XX_I2C_RECV_3BYTES,            stm32f4xx_i2c_fsm_condition_R_3B_to_RECV_2BF,       STM32F4XX_I2C_RECV_2BYTES_FIRST,    stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_RECV_NBYTES,            stm32f4xx_i2c_fsm_condition_RECV_NB_to_RECV_NB,     STM32F4XX_I2C_RECV_NBYTES,          stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_RECV_NBYTES,            stm32f4xx_i2c_fsm_condition_RECV_NB_to_R_3B,        STM32F4XX_I2C_RECV_3BYTES,          stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_RECV_NBYTES,            stm32f4xx_i2c_fsm_condition_RECV_NB_to_RECV_2BF,    STM32F4XX_I2C_RECV_2BYTES_FIRST,    stm32f4xx_i2c_data_reception                },
    { STM32F4XX_I2C_RECV_1BYTE,             stm32f4xx_i2c_fsm_condition_RECV_1B_to_IDLE,        STM32F4XX_I2C_IDLE,                 stm32f4xx_i2c_stop_generation               },
    { STM32F4XX_I2C_RECV_2BYTES_SECOND,     stm32f4xx_i2c_fsm_condition_RECV_2BS_to_IDLE,       STM32F4XX_I2C_IDLE,                 stm32f4xx_i2c_stop_generation               },

    { -1,   NULL,   -1, NULL},
};


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
            s->fsm->act_st = STM32F4XX_I2C_IDLE;
        } 
        else 
        {
            s->i2c_cr1 &= ~STM_I2C_PE_BIT;
            s->i2c_sr1 = 0;
            s->i2c_sr2 &= ~STM_I2C_MSL_BIT;
            s->fsm->act_st = STM32F4XX_I2C_DISABLED;
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
            STM32_DEBUG("ERROR: START requested but peripheral not enabled\n");
            return;
        }

        s->flags.flg_start = true;

        value &= ~STM_I2C_START_BIT;
    }

    // STOP bit handling (Bit 9) - self-clearing
    if ( value & STM_I2C_STOP_BIT )
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT)) 
        {
            STM32_DEBUG("ERROR: STOP requested but peripheral not enabled\n");
            return;
        }

        STM32_DEBUG("Stop condition requested\n");
        s->flags.flg_stop = true;

        value &= ~STM_I2C_STOP_BIT;
    }

    // ACK bit handling (Bit 10)
    if ( (s->i2c_cr1 & STM_I2C_ACK_BIT) != (value & STM_I2C_ACK_BIT) ) 
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT))
        {
            STM32_DEBUG("ERROR: ACK bit seted but peripheral not enabled\n");
            return;
        }

        if (value & STM_I2C_ACK_BIT) 
        {
            s->i2c_cr1 |= STM_I2C_ACK_BIT;
            STM32_DEBUG("\nSTM32 I2C: ACK enabled\n");
        }
        else 
        {
            s->i2c_cr1 &= ~STM_I2C_ACK_BIT;
            STM32_DEBUG("\nSTM32 I2C: NACK enabled\n");            
        }
    }
    
    // POS bit handling (Bit 11)
    if ( (s->i2c_cr1 & STM_I2C_POS_BIT) != (value & STM_I2C_POS_BIT) ) 
    {
        if (!(s->i2c_cr1 & STM_I2C_PE_BIT))
        {
            STM32_DEBUG("ERROR: POS bit seted but peripheral not enabled\n");
            return;
        }
        
        STM32_DEBUG(" POS bit set\n");

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
        STM32_DEBUG(" Software Reset\n");
        
        stm32f4xx_i2c_reset_reg(s);

        return;
    }

    s->i2c_cr1 = (uint32_t)value;

    /* Launch FSM */
    stm32f4xx_i2c_fsm_fire(s);
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
    
    /* Launch FSM */
    stm32f4xx_i2c_fsm_fire(s);

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
        STM32_DEBUG(" SB flag cleared after DR write\n");
        s->flags.flg_sb = false;
        stm32f4xx_i2c_clear_status_flag(s, STM_I2C_SB_BIT);
    }

    /* Launch FSM */
    stm32f4xx_i2c_fsm_fire(s);
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
    
    STM32_DEBUG("READ addres: 0x%02x", (uint32_t)addr);

    s->ops = ops_r;
    s->addr = addr;

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

    STM32_DEBUG("Value readed: 0x%02x", (uint32_t)result);

    s->ops = ops_na;
    s->addr = 0xFF;

    return result;
}

static void stm32f4xx_i2c_write(void *opaque, hwaddr addr, uint64_t value, uint32_t size)
{
    STM32F4XXI2CState *s = opaque;

    STM32_DEBUG("WRITE addres: 0x%02x", (uint32_t)addr);

    s->ops = ops_w;
    s->addr = addr;

    switch (addr) 
    {
        case STM_I2C_REG_CR1:   __i2c_write_cr1(s, value);      break;
        case STM_I2C_REG_CR2:   __i2c_write_cr2(s, value);      break;
        case STM_I2C_REG_OAR1:  s->i2c_oar1 = (uint32_t)value;  break;
        case STM_I2C_REG_OAR2:  s->i2c_oar2 = (uint32_t)value;  break;
        case STM_I2C_REG_DR:    __i2c_write_dr(s, value);       break;
        case STM_I2C_REG_SR1:   s->i2c_sr1  = (uint32_t)value;  break;
        case STM_I2C_REG_SR2:                                   break;
        case STM_I2C_REG_CCR:   s->i2c_ccr  = (uint32_t)value;  break;
        case STM_I2C_REG_TRISE: s->i2c_trise = (uint32_t)value; break;
        case STM_I2C_REG_FLTR:  s->i2c_fltr = (uint32_t)value;  break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "STM32 I2C: Bad write offset 0x%lx\n", addr);
    }

    s->ops = ops_na;
    s->addr = 0xFF;

}


/**
 * ============================================================================
 *              DEVICE LIFE FUNCTIONS
 * ============================================================================ 
 */

static void stm32f4xx_i2c_reset_reg(STM32F4XXI2CState* stm32f4xx)
{
     STM32_DEBUG("Resetting I2C controller");
    
    // Free existing FSM if it exists
    if (stm32f4xx->fsm) 
    {
        g_free(stm32f4xx->fsm);
        stm32f4xx->fsm = NULL;
    }

    // Create new FSM
    stm32f4xx->fsm = stm32f4xx_i2c_fsm_new(stm32f4xx_i2c_fsm_transition_table);
    if (!stm32f4xx->fsm)
        STM32_DEBUG("ERROR: Failed to create FSM on reset!");

    stm32f4xx->ops              = ops_na;
    stm32f4xx->addr             = 0xFF;
    stm32f4xx->slv_address      = 0xFF;

    stm32f4xx->flags.flg_sb     = false;
    stm32f4xx->flags.flg_start  = false;
    stm32f4xx->flags.flg_stop   = false;
    stm32f4xx->flags.flg_swrst  = false;
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
    STM32F4XXI2CState  *src = STM32F4XX_I2C(dev);
    stm32f4xx_i2c_reset_reg(src);
}

static const VMStateDescription vmstate_stm32f4xx_i2c = 
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
    STM32_DEBUG("Realizing STM32F4XX I2C controller\n");

    STM32F4XXI2CState   *stm32  = STM32F4XX_I2C(dev);
    SysBusDevice        *sbd    = SYS_BUS_DEVICE(dev);

    if (!stm32->bus_name) 
    {
        stm32->bus_name = g_strdup("i2c");
    }

    STM32_DEBUG("stm32->bus_name = %s \n", stm32->bus_name );
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


    if (!stm32->bus) 
    {
        error_setg(errp, "stm32f4xx_i2c: I2C bus not initialized");
        return;
    }


    STM32_DEBUG(" I2C realized successfully\n");
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
    dc->vmsd    = &vmstate_stm32f4xx_i2c;
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
