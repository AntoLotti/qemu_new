#include "qemu/log.h"
#include "hw/irq.h"
#include "migration/vmstate.h"

#include "qemu/osdep.h"
#include "hw/i2c/lis3dh_i2c.h"
#include "hw/qdev-properties.h"

/**********************/
/*  Basic Functions   */
/**********************/
static void lis3dh_write( LIS3DHState *dst, uint8_t addr, uint8_t src )
{
    
}

static uint8_t lis3dh_read( LIS3DHState *src, uint8_t addr )
{
    
}

static void lis3dh_i2c_realize(DeviceState *dev, Error **errp)
{
    LIS3DHState *s = LIS3DH_I2C(dev);
    
    /* Initialize I2C state */
    s->pointer = 0xFF;  // Invalid initial pointer
    s->command_phase = true;
    
    /* Create data update timer */
    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, lis3dh_update_data, s);
    timer_mod(s->timer, 
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100); // 100Hz update
    
    /* Enable hotplug */
    DeviceClass *dc = DEVICE_GET_CLASS(dev);
    dc->hotpluggable = true;
}

static void lis3dh_i2c_unrealize(DeviceState *dev)
{
    LIS3DHState *s = LIS3DH_I2C(dev);
    
    timer_del(s->timer);
    timer_free(s->timer);
}

static void lis3dh_i2c_reset(DeviceState *dev)
{
    LIS3DHState *lis3dh     = LIS3DH_I2C(dev);

    lis3dh->status_reg_aux  = LIS3DH_STATUS_REG_DEFAULT;        //Default: Output
    
    memset( lis3dh->adc_reg, 0, 0x07 );                         //Default: Output
    
    uint8_t who_am_i = LIS3DH_WHO_AM_I_DEFAULT;                 //Default: 00110011 (default in write)
    
    uint8_t ctrl_reg[0x00]  = LIS3DH_CTRL_REG0_DEFAULT;         //Default: 00010000
    uint8_t ctrl_reg[0x01]  = LIS3DH_TEMP_CFG_REG_DEFAULT;      //Default: 0
    uint8_t ctrl_reg[0x02]  = LIS3DH_CTRL_REG1_DEFAULT;         //Default: 00000111
    
    memset( ((lis3dh->ctrl_reg)+2), 0, 0x06 );                  //Default: 0

    lis3dh->reference       = LIS3DH_REFERENCE_DEFAULT;         //Default: 0
    lis3dh->status_reg      = LIS3DH_STATUS_REG_DEFAULT;        //Default: Output
    
    memset( lis3dh->out_x_reg,  0, 0x02 );                      //Default: Output
    memset( lis3dh->out_y_reg,  0, 0x02 );                      //Default: Output
    memset( lis3dh->out_z_reg,  0, 0x02 );                      //Default: Output
    
    lis3dh->fifo_ctrl_reg   = LIS3DH_FIFO_CTRL_REG_DEFAULT;     //Default: 0
    lis3dh->fifo_src_reg    = LIS3DH_FIFO_SRC_REG_DEFAULT;      //Default: Output
    
    memset( lis3dh->int1_reg,   0, 0x04 );                      //Default:  
    memset( lis3dh->int2_reg,   0, 0x04 );                      //Default:
   
    lis3dh->click_cfg       = LIS3DH_CLICK_CFG_DEFAULT;         //Default: 0
    lis3dh->click_src       = LIS3DH_CLICK_SRC_DEFAULT;         //Default: Output
    lis3dh->click_ths       = LIS3DH_CLICK_CFG_DEFAULT;         //Default: 0

    lis3dh->time_limit      = LIS3DH_TIME_LIMIT_DEFAULT;        //Default: 0
    lis3dh->time_latency    = LIS3DH_TIME_LATENCY_DEFAULT;      //Default: 0
    lis3dh->time_window     = LIS3DH_TIME_WINDOW_DEFAULT;       //Default: 0

    memset( lis3dh->act_reg, 0, 0x02 );                         //Default: 0

}


/********************/
/*  I2C Functions   */
/********************/
static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    printf("I2C LIS3DH event: %d \n", event );
    
    LIS3DHState *lis3dh = LIS3DH_I2C(dev);
    
    switch (event) 
    {
        case I2C_START_SEND:
            printf("I2C start send");    
            break;
        case I2C_START_RECV:
            printf("I2C start reciv");    
            break;
        case I2C_FINISH:
            printf("I2C finish");    
            break;
        case I2C_NACK:
            printf("I2C nack");    
            break;
        default:
            return -1;
    }
    return 0;
}

static int lis3dh_i2c_send(I2CSlave *i2c, uint8_t data)
{
    printf("I2C LIS3DH sent:");
}

static int lis3dh_i2c_recv(I2CSlave *i2c)
{
    printf("I2C LIS3DH recived:");
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
    dc->realize     = lis3dh_i2c_realize;           // Called when device created
    dc->unrealize   = lis3dh_i2c_unrealize;         // Cleanup
    dc->reset       = lis3dh_i2c_reset;             // On system reset
    dc->desc        = "I2C accelerometer: LIS3DH"; 
    
    /* I2C protocol implementation */
    k->send     = lis3dh_i2c_send;                  // Master writes to device
    k->recv     = lis3dh_i2c_recv;                  // Master reads from device
    k->event    = lis3dh_i2c_event;                 // Bus events: START/STOP/NACK
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