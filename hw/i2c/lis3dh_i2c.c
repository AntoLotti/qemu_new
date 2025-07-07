#include "qemu/osdep.h"
#include "hw/i2c/lis3dh_i2c.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

/**********************/
/*  Basic Functions   */
/**********************/
static void lis3dh_update_data(void *opaque)
{
    LIS3DHState *s = opaque;
    
    /* Generate new accelerometer values */
    // Replace with actual sensor model or test data
    uint16_t x = rand() % 0xFFFF;
    uint16_t y = rand() % 0xFFFF;
    uint16_t z = rand() % 0xFFFF;
    
    /* Update registers */
    s->out_x_reg[LIS3DH_OUT_X_H - LIS3DH_OUT_X_L] = x & 0xFF;
    s->out_x_reg[LIS3DH_OUT_X_L - LIS3DH_OUT_X_L] = (x >> 8) & 0xFF;
    s->out_y_reg[LIS3DH_OUT_Y_H - LIS3DH_OUT_Y_L] = y & 0xFF;
    s->out_y_reg[LIS3DH_OUT_Y_L - LIS3DH_OUT_Y_L] = (y >> 8) & 0xFF;
    s->out_z_reg[LIS3DH_OUT_Z_H - LIS3DH_OUT_Y_L] = z & 0xFF;
    s->out_z_reg[LIS3DH_OUT_Z_H - LIS3DH_OUT_Y_L] = (z >> 8) & 0xFF;
    
    /* Set data ready flag */
    //s->status_reg |= 0x08;  // Set DRDY bit
    
    /* Reschedule timer */
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 
             NANOSECONDS_PER_SECOND / 100);
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

/*
static void lis3dh_i2c_reset(DeviceState *dev)
{
    LIS3DHState *lis3dh     = LIS3DH_I2C(dev);

    lis3dh->status_reg_aux  = LIS3DH_STATUS_REG_DEFAULT;        //Default: Output
    
    //memset( lis3dh->adc_reg, 0, 0x07 );                         //Default: Output
    
    lis3dh->who_am_i        = LIS3DH_WHO_AM_I_DEFAULT;          //Default: 00110011 (default in write)
    
    //lis3dh->ctrl_reg[0x00]  = LIS3DH_CTRL_REG0_DEFAULT;         //Default: 00010000
    //lis3dh->ctrl_reg[0x01]  = LIS3DH_TEMP_CFG_REG_DEFAULT;      //Default: 0
    //lis3dh->ctrl_reg[0x02]  = LIS3DH_CTRL_REG1_DEFAULT;         //Default: 00000111
    //memset( ((lis3dh->ctrl_reg)+2), 0, 0x06 );                  //Default: 0

    lis3dh->reference_reg       = LIS3DH_REFERENCE_DEFAULT;         //Default: 0
    //lis3dh->status_reg      = LIS3DH_STATUS_REG_DEFAULT;        //Default: Output
    
    memset( lis3dh->out_x_reg,  0, 0x02 );                      //Default: Output
    memset( lis3dh->out_y_reg,  0, 0x02 );                      //Default: Output
    memset( lis3dh->out_z_reg,  0, 0x02 );                      //Default: Output
    
    //lis3dh->fifo_ctrl_reg   = LIS3DH_FIFO_CTRL_REG_DEFAULT;     //Default: 0
    //lis3dh->fifo_src_reg    = LIS3DH_FIFO_SRC_REG_DEFAULT;      //Default: Output
    
    //memset( lis3dh->int1_reg,   0, 0x04 );                      //Default:  
    //memset( lis3dh->int2_reg,   0, 0x04 );                      //Default:
   
    lis3dh->click_cfg       = LIS3DH_CLICK_CFG_DEFAULT;         //Default: 0
    lis3dh->click_src       = LIS3DH_CLICK_SRC_DEFAULT;         //Default: Output
    lis3dh->click_ths       = LIS3DH_CLICK_CFG_DEFAULT;         //Default: 0

    lis3dh->time_limit      = LIS3DH_TIME_LIMIT_DEFAULT;        //Default: 0
    lis3dh->time_latency    = LIS3DH_TIME_LATENCY_DEFAULT;      //Default: 0
    lis3dh->time_window     = LIS3DH_TIME_WINDOW_DEFAULT;       //Default: 0

    //memset( lis3dh->act_reg, 0, 0x02 );                         //Default: 0
}
*/

/********************/
/*  I2C Functions   */
/********************/
static void set_register_value(LIS3DHState *src, uint8_t pointer, uint8_t data)
{
    printf("\n hola \n");
}

static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS3DHState *s = LIS3DH_I2C(i2c);
    
    switch (event) 
    {
        case I2C_START_SEND:  // Start of write operation
            s->command_phase = true;
            s->pointer = 0xFF;  // Reset pointer
            break;
            
        case I2C_START_RECV:  // Start of read operation
            // If no pointer set, default to first register
            if (s->pointer == 0xFF)
            {
                s->pointer = LIS3DH_STATUS_REG_AUX;
            }
            break;
            
        case I2C_FINISH:      // Stop condition
            s->command_phase = false;
            break;
            
        case I2C_NACK:        // NACK received
            break;
        
        default:
            break;
    }
    return 0;
}

static int lis3dh_i2c_send(I2CSlave *i2c, uint8_t data)
{
    LIS3DHState *s = LIS3DH_I2C(i2c);
    
    if (s->command_phase) 
    {
        // First byte is register address
        s->pointer = data;
        s->command_phase = false;
    } else 
    {
        // Write to register
        set_register_value(s, s->pointer, data);
        s->pointer++;
    }
    return 0;
}

static uint8_t lis3dh_i2c_recv(I2CSlave *i2c)
{
    LIS3DHState *s = LIS3DH_I2C(i2c);
    uint8_t val = 0x00;
    
    switch (s->pointer)
    {
        case LIS3DH_STATUS_REG    :

            break;

        /*
        case LIS3DH_ADC_1_L       :
            val = s->adc_reg[LIS3DH_ADC_1_L - LIS3DH_ADC_1_L];
            break;

        case LIS3DH_ADC_1_H       :
            val = s->adc_reg[LIS3DH_ADC_1_H - LIS3DH_ADC_1_L];
            break;

        case LIS3DH_ADC_2_L       :
            val = s->adc_reg[LIS3DH_ADC_2_L - LIS3DH_ADC_1_L];
            break;

        case LIS3DH_ADC_2_H       :
            val = s->adc_reg[LIS3DH_ADC_2_H- LIS3DH_ADC_1_L];
            break;

        case LIS3DH_ADC_3_L       :
            val = s->adc_reg[LIS3DH_ADC_3_L - LIS3DH_ADC_1_L];
            break;

        case LIS3DH_ADC_3_H       :
            val = s->adc_reg[LIS3DH_ADC_3_H - LIS3DH_ADC_1_L];
            break;

        case LIS3DH_WHO_AM_I      :
            val = LIS3DH_WHO_AM_I_DEFAULT;
            break;

        case LIS3DH_CTRL_REG0     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG0 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_TEMP_CFG_REG  :
            val = s->ctrl_reg[LIS3DH_TEMP_CFG_REG - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG1     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG1 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG2     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG2 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG3     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG3 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG4     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG4 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG5     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG5 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_CTRL_REG6     :
            val = s->ctrl_reg[LIS3DH_CTRL_REG6 - LIS3DH_CTRL_REG0];
            break;

        case LIS3DH_REFERENCE     :
            val = LIS3DH_REFERENCE
            break;

        case LIS3DH_STATUS_REG    :
            val = LIS3DH_STATUS_REG
            break;

        case LIS3DH_OUT_X_L       :
            break;

        case LIS3DH_OUT_X_H       :
            break;

        case LIS3DH_OUT_Y_L       :
            break;

        case LIS3DH_OUT_Y_H       :
            break;

        case LIS3DH_OUT_Z_L       :
            break;

        case LIS3DH_OUT_Z_H       :
            break;

        case LIS3DH_FIFO_CTRL_REG :
            break;

        case LIS3DH_FIFO_SRC_REG  :
            break;

        case LIS3DH_INT1_CFG      :
            break;

        case LIS3DH_INT1_SRC      :
            break;

        case LIS3DH_INT1_THS      :
            break;

        case LIS3DH_INT1_DURATION :
            break;

        case LIS3DH_INT2_CFG      :
            break;

        case LIS3DH_INT2_SRC      :
            break;

        case LIS3DH_INT2_THS      :
            break;

        case LIS3DH_INT2_DURATION :
            break;
            
        case LIS3DH_CLICK_CFG     :
            break;

        case LIS3DH_CLICK_SRC     :
            break;

        case LIS3DH_CLICK_THS     :
            break;
        
        case LIS3DH_TIME_LIMIT    :
            break;
        
        case LIS3DH_TIME_LATENCY  :
            break;

        case LIS3DH_TIME_WINDOW   :
            break;

        case LIS3DH_ACT_THS       :
            break;

        case LIS3DH_ACT_DUR       :
            break;
        */
        default                   :
            break;
    }
    
    return val;
}

/*********************/
/* Type Registration */
/*********************/

/* LIS3DH class initialization */
static void lis3dh_i2c_class_init( ObjectClass *kclass, void *data )
{
    DeviceClass *dc     = DEVICE_CLASS(kclass);      // The generic device class operations
    I2CSlaveClass *k    = I2C_SLAVE_CLASS(kclass);   // The I2C-specific interface implementation
    
    /* Device lifecycle */
    dc->realize     = lis3dh_i2c_realize;           // Called when device created
    dc->unrealize   = lis3dh_i2c_unrealize;         // Cleanup
    //dc->reset       = lis3dh_i2c_reset;             // On system reset
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
};

/* Add LIS3DH object class into Qemu core */
static void lis3dh_i2c_regster_types(void)
{
    type_register_static(&lis3dh_i2c_info);
}

type_init(lis3dh_i2c_regster_types);