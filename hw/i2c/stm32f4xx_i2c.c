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
#include "hw/irq.h"
#include "hw/i2c/stm32f4xx_i2c.h"
#include "migration/vmstate.h"

/**************************************************************************
    I2C FUNCTIONS
**************************************************************************/
static uint64_t stm32f4xx_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    STM32F4XXI2CState *s = opaque;
    uint32_t readval = 0x00U;

    STM32F4XXI2CState *s = opaque;
    
    switch (addr) {
    case STM_I2C_REG_CR1:       return (uint64_t)s->cr1;
    case STM_I2C_REG_CR2:       return (uint64_t)s->cr2;
    case STM_I2C_REG_OAR1:      return (uint64_t)s->oar1;
    case STM_I2C_REG_OAR2:      return (uint64_t)s->oar2;
    case STM_I2C_REG_DR:        return (uint64_t)s->dr;
    case STM_I2C_REG_SR1:       return (uint64_t)s->sr1;
    case STM_I2C_REG_SR2:       return (uint64_t)s->sr2;
    case STM_I2C_REG_CCR:       return (uint64_t)s->ccr;
    case STM_I2C_REG_TRISE:     return (uint64_t)s->trise;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "stm32f4xx_i2c: read at 0x%" HWADDR_PRIx "\n", addr);
        return 0x00UL;
    }

    return readval;
}

static void stm32f4xx_i2c_write(void *opaque, hwaddr addr, uint64_t value, uint32_t size)
{
    STM32F4XXI2CState *s = opaque;
    
    switch (addr) 
    {
        case I2C_CR1:   s->cr1  = value;    break;
        case I2C_CR2:   s->cr2  = value;    break;
        case I2C_OAR1:  s->oar1 = value;    break;
        case I2C_OAR2:  s->oar2 = value;    break;
        case I2C_DR:    s->dr   = value;    break;
        case I2C_SR1:   /* Read-only */     break;
        case I2C_SR2:   /* Read-only */     break;
        case I2C_CCR:   s->ccr  = value;    break;
        case I2C_TRISE: s->trise = value;   break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "stm32f4xx_i2c: write at 0x%" HWADDR_PRIx "\n", addr);
            break;
    }
}

/**************************************************************************
    DEVICE LIFE FUNCTIONS
**************************************************************************/
static void stm32f4xx_i2c_reset(DeviceState *dev)
{
    STM32F4XXI2CState *stm32f4xx = STM32F4XX_I2C(dev);
  
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

static const VMStateDescription vmstate_stm32f2xx_i2c = {
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
/**************************************************************************
    LIS3DH REGISTRATION IN QEMU 
**************************************************************************/
static void stm32f4xx_i2c_init(Object *obj)
{
    STM32F4XXI2CState *stm32 = STM32F4XX_I2C(obj);
    SysBusDevice    *sbd    = SYS_BUS_DEVICE(obj);
    DeviceState     *dev    = DEVICE(obj);

    /* Initialize memory region */
    memory_region_init_io
    (
        &stm32->mmio, obj, 
        &stm32f4xx_i2c_ops, stm32,
        TYPE_STM32F4XX_I2C, 0x400
    );
    sysbus_init_mmio(sbd, &stm32->mmio);
    
    /* Initialize interrupt lines */
    sysbus_init_irq(sbd, &stm32->irq);
    //sysbus_init_irq(sbd, &s->irq_event);   // Event interrupt
    //sysbus_init_irq(sbd, &s->irq_error);    // Error interrupt
    
    /* Create I2C bus */
    s->bus = i2c_init_bus(dev, "i2c");
}

static void stm32f4xx_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    device_class_set_legacy_reset(dc, stm32f4xx_i2c_reset);
    dc->vmsd    = &vmstate_stm32f4xx_i2c;
    //dc->realize = stm32f4xx_i2c_realize;
}

static const TypeInfo stm32f4xx_i2c_info = 
{
    .name           = TYPE_STM32F4XX_I2C,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(STM32F4XXI2CState),
    .instance_init  = stm32f4xx_i2c_init,
    .class_init     = stm32f4xx_i2c_class_init,
};

static void stm32f4xx_i2c_register_types(void)
{
    type_register_static(&stm32f4xx_i2c);
}

type_init(stm32f4xx_i2c_register_types)
