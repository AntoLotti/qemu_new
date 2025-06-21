#include "qemu/log.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

#include "qemu/osdep.h"
#include "hw/i2c/lis3dh_i2c.h"
#include "hw/qdev-properties.h"

/**********************/
/*  Basic Functions   */
/**********************/
static uint8_t lis3dh_read_byte( LIS3DHState *src )
{

}

static int lis3dh_i2c_realize(DeviceState *dev, Error **errp)
{
    LIS3DHState *lis3dh = 
}

static int lis3dh_i2c_unrealize()
{

}

static int lis3dh_i2c_reset()
{

}


/********************/
/*  I2C Functions   */
/********************/
static int lis3dh_i2c_event(I2CSlave *i2c)
{

}

static int lis3dh_i2c_send(I2CSlave *i2c)
{

}

static int lis3dh_i2c_recv(I2CSlave *i2c)
{

}

/*********************/
/* Type Registration */
/*********************/

/* LIS3DH class initialization */
void lis3dh_i2c_class_init( ObjectClass *kclass, const void *data )
{
    DeviceClass *dc     = DEVICE_CLASS(klass);      // The generic device class operations
    I2CSlaveClass *k    = I2C_SLAVE_CLASS(klass);   // The I2C-specific interface implementation
    
    /* Device lifecycle */
    dc->desc        = "I2C accelerometer: LIS3DH"; 
    dc->realize     = lis3dh_i2c_realize;       // Called when device created
    dc->unrealize   = lis3dh_i2c_unrealize;     // Cleanup
    dc->reset       = lis3dh_i2c_reset;         // On system reset
    
    /* I2C protocol implementation */
    k->send     = lis3dh_i2c_send;              // Master writes to device
    k->recv     = lis3dh_i2c_recv;              // Master reads from device
    k->event    = lis3dh_i2c_event;             // Bus events: START/STOP/NACK
}

/* Tells QEMU’s type system how to create and wire the LIS3DH object class. */
static const TypeInfo lis3dh_i2c_info = {
    .name           = TYPE_LIS3DH_I2C, 
    .parent         = TYPE_I2C_SLAVE,
    .instance_size  = sizeof(LIS3DHState),
    .class_init     = lis3dh_i2c_class_init,
}

/* Add LIS3DH object class into Qemu core */
static void lis3dh_i2c_regster_types(void)
{
    type_register_static(&lis3dh_i2c_info);
}

type_init(lis3dh_i2c_regster_types);