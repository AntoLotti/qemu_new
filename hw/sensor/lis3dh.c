#include "qemu/osdep.h"
#include "hw/sensor/lis3dh.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include <math.h>

static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event);
static int lis3dh_i2c_send(I2CSlave *i2c, uint8_t data);
static uint8_t lis3dh_i2c_recv(I2CSlave *i2c);

static LIS3DH_Mode_t __lis3dh_get_current_mode(LIS3DHState* src);
static LIS3DH_FullScale_t __lis3dh_get_current_fs(LIS3DHState* src);

static bool __reserved_address( uint8_t src);
static bool __write_in_register( LIS3DHState *dst, uint8_t dir, uint8_t src );
static uint8_t __read_register( LIS3DHState *src );

static void __lis3dh_update_data(void *src);
static void __lis3dh_reset(LIS3DHState *lis3dh);
static void lis3dh_realize(DeviceState *dev, Error **errp);
static void lis3dh_unrealize(DeviceState *dev);

static int16_t __float_to_int16(float src);
static void __data_transformation( float src, uint8_t* out_axis_h, uint8_t* out_axis_l );

static void lis3dh_set_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);
static void lis3dh_set_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);
static void lis3dh_set_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);

static void lis3dh_get_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);
static void lis3dh_get_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);
static void lis3dh_get_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);

//static void lis3dh_get_temp(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);
//static void lis3dh_set_temp(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp);

/**************************************************************************
    ACCELEROMETER DATA GENERATION
**************************************************************************/
static int16_t __float_to_int16(float src)
{
    bool is_pos = true;

    if ( src < 0 )
        is_pos = false;

    int16_t dst = 0x00;

    if ( fabsf(src - (int)src) >= 0.5f )
        dst = (int16_t)src + (is_pos ? 1 : -1); 
    else
        dst = (int16_t)src;

    return dst;
}

static void __data_transformation( float src, uint8_t* out_axis_h, uint8_t* out_axis_l )
{
    /**
    *   TODO: develop each lis3dh mode
    *   - NOW only high resolution mode with a FS of +- 2g
    */

    float data_with_So = src / ( 0.001f );   // -1,023(g) / 0.001(g/LSB) = -1023 LSB 

    /**
     * TODO: check the max value of each modes
     */

    int16_t data_in_16b = __float_to_int16( data_with_So ); // data_in_16b = 1111 1100 0000 0001
    
    /**
    * TODO: change the shift depending of the mode
    * - now High resolution mode (12 bits) --> 4 bits shift
    */
    
    int16_t x_raw_data = data_in_16b << 4; // 1111 1100 0000 0001 --> 1100 0000 0001 0000
    printf("\n\n x_raw_data: 0x%x \n\n", x_raw_data);
    
    // Split Data Into Registers //
    *out_axis_h = (uint8_t)( (x_raw_data & 0xFF00) >> 8 ); // 1100 0000 = 0xc0
    printf("\n\n out_x_h: 0x%x \n\n",*out_axis_h);
    *out_axis_l = (uint8_t)(x_raw_data & 0x00FF);          // 0001 0000 = 0x10
    printf("\n\n out_x_l: 0x%x \n\n",*out_axis_l);

}


static void lis3dh_set_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{

    LIS3DHState *s = LIS3DH(obj);
    int64_t value;

    // Data Generation In g //
    visit_type_int(v, name, &value, errp);

    __data_transformation( (value * 1.0), &s->out_x_h, &s->out_x_l );
}

static void lis3dh_set_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{

    LIS3DHState *s = LIS3DH(obj);
    int64_t value;

    // Data Generation In g //
    visit_type_int(v, name, &value, errp);

    __data_transformation( (value * 1.0), &s->out_y_h, &s->out_y_l );
}

static void lis3dh_set_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{

    LIS3DHState *s = LIS3DH(obj);
    int64_t value;

    // Data Generation In g //
    visit_type_int(v, name, &value, errp);

    __data_transformation( (value * 1.0), &s->out_z_h, &s->out_z_l );
}


static void lis3dh_get_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    int16_t raw = ((int16_t)s->out_x_h << 8) | s->out_x_l;
    int64_t value = raw >> 4;

    visit_type_int(v, name, &value, errp);
}

static void lis3dh_get_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    int16_t raw = ((int16_t)s->out_y_h << 8) | s->out_y_l;
    int64_t value = raw >> 4;

    visit_type_int(v, name, &value, errp);
}

static void lis3dh_get_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    int16_t raw = ((int16_t)s->out_z_h << 8) | s->out_z_l;
    int64_t value = raw >> 4;

    visit_type_int(v, name, &value, errp);
}

/**************************************************************************
    DEVICE LIFE FUNCTIONS
**************************************************************************/
static void __lis3dh_update_data(void *src)
{
    (void)src;
    //LIS3DHState *lis3dh = src;
    //
    ///* Generate new accelerometer values */
    //__acc_x_axis_data_generation( lis3dh->out_x_h, lis3dh->out_x_l );
    //__acc_y_axis_data_generation( lis3dh->out_y_h, lis3dh->out_y_l );
    //__acc_z_axis_data_generation( lis3dh->out_z_h, lis3dh->out_z_l );
    //    
    ///* Set data ready flag */
    ////s->status_reg |= 0x08;  // Set DRDY bit
    //
    ///* Reschedule timer */
    //timer_mod
    //(
    //    lis3dh->timer, 
    //    qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100
    //);
}

static void __lis3dh_reset(LIS3DHState *lis3dh)
{
    // directions [0x00-0x06] reserved
    lis3dh->status_reg_aux  = LIS3DH_STATUS_REG_AUX_DEF;    // Status Register
    lis3dh->adc_1_l         = LIS3DH_ADC_1_L_DEF;           // 1-Axis Acceleration Data Low Register
    lis3dh->adc_1_h         = LIS3DH_ADC_1_H_DEF;           // 1-Axis Acceleration Data High Register
    lis3dh->adc_2_l         = LIS3DH_ADC_2_L_DEF;           // 2-Axis Acceleration Data Low Register
    lis3dh->adc_2_h         = LIS3DH_ADC_2_H_DEF;           // 2-Axis Acceleration Data High Register
    lis3dh->adc_3_l         = LIS3DH_ADC_3_L_DEF;           // 3-Axis Acceleration Data Low Register
    lis3dh->adc_3_h         = LIS3DH_ADC_3_H_DEF;           // 3-Axis Acceleration Data High Register
    // direction 0x0E reserved
    lis3dh->who_am_i        = LIS3DH_WHO_AM_I_DEF;          // Device identification Register 
    // directions [0x10-0x1D] reserved
    lis3dh->ctrl_reg0       = LIS3DH_CTRL_REG0_DEF;         //
    lis3dh->temp_cfg_reg    = LIS3DH_TEMP_CFG_REG_DEF;      // Temperature Sensor Register
    lis3dh->ctrl_reg1       = LIS3DH_CTRL_REG1_DEF;         // Accelerometer Control Register 1
    lis3dh->ctrl_reg2       = LIS3DH_CTRL_REG2_DEF;         // Accelerometer Control Register 2
    lis3dh->ctrl_reg3       = LIS3DH_CTRL_REG3_DEF;         // Accelerometer Control Register 3
    lis3dh->ctrl_reg4       = LIS3DH_CTRL_REG4_DEF;         // Accelerometer Control Register 4
    lis3dh->ctrl_reg5       = LIS3DH_CTRL_REG5_DEF;         // Accelerometer Control Register 5
    lis3dh->ctrl_reg6       = LIS3DH_CTRL_REG6_DEF;         // Accelerometer Control Register 6
    lis3dh->reference       = LIS3DH_REFERENCE_DEF;         // Reference/Datacapture Register
    lis3dh->status_reg      = LIS3DH_STATUS_REG_DEF;        // Status Register 2

    lis3dh->out_x_l         = 0x10;//LIS3DH_OUT_X_L_DEF;           // X-Axis Acceleration Data Low Register
    lis3dh->out_x_h         = 0xc0;//LIS3DH_OUT_X_H_DEF;           // X-Axis Acceleration Data High Register
    lis3dh->out_y_l         = 0x10;//LIS3DH_OUT_Y_L_DEF;           // Y-Axis Acceleration Data Low Register
    lis3dh->out_y_h         = 0xc0;//LIS3DH_OUT_Y_H_DEF;           // Y-Axis Acceleration Data High Register
    lis3dh->out_z_l         = 0x10;//LIS3DH_OUT_Z_L_DEF;           // Z-Axis Acceleration Data Low Register
    lis3dh->out_z_h         = 0xc0;//LIS3DH_OUT_Z_H_DEF;           // Z-Axis Acceleration Data High Register
    
    lis3dh->fifo_ctrl_reg   = LIS3DH_FIFO_CTRL_REG_DEF;     // FIFO Control Register
    lis3dh->fifo_src_reg    = LIS3DH_FIFO_SRC_REG_DEF;      // FIFO Source Register
    lis3dh->int1_cfg        = LIS3DH_INT1_CFG_DEF;          // Interrupt Configuration Register
    lis3dh->int1_src        = LIS3DH_INT1_SRC_DEF;          // Interrupt Source Register
    lis3dh->int1_ths        = LIS3DH_INT1_THS_DEF;          // Interrupt Threshold Register
    lis3dh->int1_duration   = LIS3DH_INT1_DURATION_DEF;     // Interrupt Duration Register
    lis3dh->int2_cfg        = LIS3DH_INT2_CFG_DEF;          //
    lis3dh->int2_src        = LIS3DH_INT2_SRC_DEF;          //
    lis3dh->int2_ths        = LIS3DH_INT2_THS_DEF;          //
    lis3dh->int2_duration   = LIS3DH_INT2_DURATION_DEF;     //       
    lis3dh->click_cfg       = LIS3DH_CLICK_CFG_DEF;         // Interrupt Click Recognition Register
    lis3dh->click_src       = LIS3DH_CLICK_SRC_DEF;         // Interrupt Click Source Register
    lis3dh->click_ths       = LIS3DH_CLICK_THS_DEF;         // Interrupt Click Threshold Register
    lis3dh->time_limit      = LIS3DH_TIME_LIMIT_DEF;        // Click Time Limit Register
    lis3dh->time_latency    = LIS3DH_TIME_LATENCY_DEF;      // Click Time Latency Register
    lis3dh->time_window     = LIS3DH_TIME_WINDOW_DEF;       // Click Time Window Register
    lis3dh->act_ths         = LIS3DH_ACT_THS_DEF;           //
    lis3dh->act_dur         = LIS3DH_ACT_DUR_DEF;           //
}

static void lis3dh_realize(DeviceState *dev, Error **errp)
{
    printf("QEMU LIS3DH realize\n");
    LIS3DHState *lis3dh = LIS3DH(dev);
    
    /* Initialize I2C state */
    lis3dh->address         = LIS3DH_DEFAULT_ADDRESS;
	lis3dh->ptr             = 0xFF;
	lis3dh->auto_increment  = false;
	//lis3dh->data_ready      = false;
	lis3dh->address_phase   = false;

    /* Create data update timer */
    lis3dh->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, __lis3dh_update_data, lis3dh);
    timer_mod
    (
        lis3dh->timer, 
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100
    ); // 100Hz update
    
    /* Reset registers */
    __lis3dh_reset(lis3dh);
        
    /* Enable hotplug */
    /*DeviceClass *dc = DEVICE_GET_CLASS(dev);
    dc->hotpluggable = true;*/
}

static void lis3dh_unrealize(DeviceState *dev)
{
    LIS3DHState *lis3dh = LIS3DH(dev);

    timer_del(lis3dh->timer);
    timer_free(lis3dh->timer);
}

static LIS3DH_Mode_t __lis3dh_get_current_mode(LIS3DHState* src)
{
    LIS3DH_Mode_t mode = LIS3DH_MODE_HIGH_RES;

    uint8_t temp =  
        ( (src->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_LPEN) >> 2 )       // 0000 X000 -> 0000 00X0
        | ( (src->ctrl_reg4 & LIS3DH_CTRL_REG4_BIT_HR) >> 3 );      // 0000 X000 -> 0000 000X

    if ( !( temp >= LIS3DH_MODE_NOT_ALLOWED) )
        mode = temp;

    return mode;
}

static LIS3DH_FullScale_t __lis3dh_get_current_fs(LIS3DHState* src)
{
    LIS3DH_FullScale_t fs = LIS3DH_FS_2G;

    uint8_t temp = (src->ctrl_reg4 & LIS3DH_CTRL_REG4_BITS_FS) >> 4;    // 00XX 0000 -> 0000 00XX

    if ( !( temp > LIS3DH_FS_16G) )
        fs = temp;

    return fs;
}

static bool __reserved_address( uint8_t src)
{
    if ( src == 0x0E )
            return true;

    for ( uint8_t i = 0x00; i < 0x07; i++ )
    {
        if ( src == i )
            return true;
    }

    for ( uint8_t j = 0x10; j < 0x1; j++ )
    {
        if ( src == j )
            return true;
    }

    return false;
}

static bool __write_in_register( LIS3DHState *dst, uint8_t dir, uint8_t src )
{
    if ( __reserved_address(dir) ) //#TODO print error message
        return false;
    
    switch (dir)
    {
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_STATUS_REG_AUX, status_reg_aux)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC1_L, adc_1_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC1_H, adc_1_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC2_L, adc_2_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC2_H, adc_2_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC3_L, adc_3_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_ADC3_H, adc_3_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_WHO_AM_I, who_am_i)
        CASE_WRITE_RETURN(LIS3DH_REG_CTRL_REG0, ctrl_reg0)
        CASE_WRITE_RETURN(LIS3DH_REG_TEMP_CFG_REG, temp_cfg_reg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG1, ctrl_reg1)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG2, ctrl_reg2)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG3, ctrl_reg3)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG4, ctrl_reg4)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG5, ctrl_reg5)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CTRL_REG6, ctrl_reg6)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_REFERENCE, reference)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_STATUS_REG, status_reg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_X_L, out_x_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_X_H, out_x_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_Y_L, out_y_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_Y_H, out_y_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_Z_L, out_z_l)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_OUT_Z_H, out_z_h)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_FIFO_CTRL, fifo_ctrl_reg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_FIFO_SRC, fifo_src_reg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT1_CFG, int1_cfg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT1_SRC, int1_src)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT1_THS, int2_ths)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT1_DURATION, int1_duration)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT2_CFG, int2_cfg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT2_SRC, int2_src)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT2_THS, int2_ths)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_INT2_DURATION, int2_duration)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CLICK_CFG, click_cfg)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CLICK_SRC, click_src)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_CLICK_THS, click_ths)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_TIME_LIMIT, time_limit)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_TIME_LATENCY, time_latency)
        CASE_WRITE_RETURN(LIS3DH_REG_ACCEL_TIME_WINDOW, time_window)
        CASE_WRITE_RETURN(LIS3DH_ACT_THS, act_ths)
        CASE_WRITE_RETURN(LIS3DH_ACT_DUR, act_dur)

        default:
            //#TODO print error message
            return false;
            break;
    }

    return false;

}

static uint8_t __read_register( LIS3DHState *src )
{
    
    printf("\n\n LIS3DH read ptr: 0x%x", src->ptr);
    
    uint8_t ret = 0x00;

    switch (src->ptr)
    {             
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_STATUS_REG_AUX, status_reg_aux)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC1_L, adc_1_l)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC1_H, adc_1_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC2_L, adc_2_l)        
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC2_H, adc_2_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC3_L, adc_3_l)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_ADC3_H, adc_3_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_WHO_AM_I, who_am_i)
        CASE_READ_RETURN(ret, LIS3DH_REG_CTRL_REG0, ctrl_reg0)
        CASE_READ_RETURN(ret, LIS3DH_REG_TEMP_CFG_REG, temp_cfg_reg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG1, ctrl_reg1)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG2, ctrl_reg2)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG3, ctrl_reg3)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG4, ctrl_reg4)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG5, ctrl_reg5)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CTRL_REG6, ctrl_reg6)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_REFERENCE, reference)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_STATUS_REG, status_reg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_X_L, out_x_l)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_X_H, out_x_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_Y_L, out_y_l)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_Y_H, out_y_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_Z_L, out_z_l)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_OUT_Z_H, out_z_h)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_FIFO_CTRL, fifo_ctrl_reg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_FIFO_SRC, fifo_src_reg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT1_CFG, int1_cfg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT1_SRC, int1_src)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT1_THS, int1_ths)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT1_DURATION, int1_duration)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT2_CFG, int2_cfg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT2_SRC, int2_src)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT2_THS, int2_ths)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_INT2_DURATION, int2_duration)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CLICK_CFG, click_cfg)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CLICK_SRC, click_src)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_CLICK_THS, click_ths)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_TIME_LIMIT, time_limit)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_TIME_LATENCY, time_latency)
        CASE_READ_RETURN(ret, LIS3DH_REG_ACCEL_TIME_WINDOW, time_window)
        CASE_READ_RETURN(ret, LIS3DH_ACT_THS, act_ths)
        CASE_READ_RETURN(ret, LIS3DH_ACT_DUR, act_dur)

        default:
            //#TODO print error message
            return false;
            break;
    }

    return ret;
}

/**************************************************************************
    I2C FUNCTIONS
**************************************************************************/
static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS3DHState *lis3dh = LIS3DH(i2c);
    
    switch (event) 
    {
        case I2C_START_SEND:    // Start of write operation
            /* Master is starting a WRITE operation 
            (sending data to the device) */
            printf("\n\n LIS3DH SEND");
            lis3dh->ptr             = 0xFF;
			lis3dh->auto_increment  = false;
			//lis3dh->data_ready      = false;
			lis3dh->address_phase   = true;    // next data is register addres
            break;
            
        case I2C_START_RECV:    // Start of read operation
            /* Master is starting a READ operation 
            (requesting data from the device) */
            printf("\n\n LIS3DH RECV");
			if (lis3dh->ptr == 0xFF)
				lis3dh->ptr = LIS3DH_REG_ACCEL_WHO_AM_I;
			lis3dh->address_phase = false;
            break;
            
        case I2C_FINISH:        // Stop condition
            /* Master ends the transaction 
            (STOP condition) */
            printf("\n\n LIS3DH STOP");
            break;
            
        case I2C_NACK:          // NACK received
            /* Master didn't acknowledge the data */
            break;
        
        default:               // Should never happen
            return -1;
    }

    return 0;
}

static int lis3dh_i2c_send(I2CSlave *i2c, uint8_t data)
{
    LIS3DHState *lis3dh = LIS3DH(i2c);

    printf("\n\n LIS3DH received: 0x%02x\n", data);

	if (lis3dh->ptr == 0xFF && lis3dh->address_phase)
	{
		lis3dh->ptr = data & LIS3DH_SUB_REG_MASK;
		lis3dh->auto_increment =  (data & LIS3DH_SUB_AUTO_INC_MASK) != 0x00 ? true : false;
		lis3dh->address_phase = false;
	}else
	{
		__write_in_register( lis3dh, lis3dh->ptr, data);
		if (lis3dh->auto_increment)
			lis3dh->ptr++; //#TODO  
	}
	return 0;
}

static uint8_t lis3dh_i2c_recv(I2CSlave *i2c)
{
    LIS3DHState *lis3dh = LIS3DH(i2c);

    uint8_t value = __read_register( lis3dh );

    printf("\nLIS3DH autoincrement: %s\n", lis3dh->auto_increment ? "true" : "false" );

    if (lis3dh->auto_increment)
        lis3dh->ptr++;

    printf("\nLIS3DH sending: 0x%02x\n", value);
    
	return value;
}

/**************************************************************************
    LIS3DH REGISTRATION IN QEMU 
**************************************************************************/

// Example for your LIS3DH
static void lis3dh_initfn(Object *obj)
{
    object_property_add(obj, "accel-x", "int",
                        lis3dh_get_accel_x,
                        lis3dh_set_accel_x, NULL, NULL);
    object_property_add(obj, "accel-y", "int",
                        lis3dh_get_accel_y, 
                        lis3dh_set_accel_y, NULL, NULL);
    object_property_add(obj, "accel-z", "int",
                        lis3dh_get_accel_z,
                        lis3dh_set_accel_z, NULL, NULL);
}

/* LIS3DH class initialization */
static void lis3dh_class_init( ObjectClass *kclass, void *data )
{
    printf("\n Qemu LIS3DH class init \n");
    DeviceClass *dc     = DEVICE_CLASS(kclass);     // The generic device class operations
    I2CSlaveClass *k    = I2C_SLAVE_CLASS(kclass);  // The I2C-specific interface implementation
    
    /* Device lifecycle */
    dc->realize         = lis3dh_realize;           // Called when device created
    dc->unrealize       = lis3dh_unrealize;         // Clean up
    dc->hotpluggable    = false;
    dc->desc            = "I2C accelerometer: LIS3DH"; 
    
    /* I2C protocol implementation */
    k->event    = lis3dh_i2c_event;                 // Bus events: START/STOP/NACK
    k->send     = lis3dh_i2c_send;                  // Master writes to device
    k->recv     = lis3dh_i2c_recv;                  // Master reads from device
}

/* Tells QEMU’s type system how to create and wire the LIS3DH object class. */
static const TypeInfo lis3dh_i2c_info = 
{
    .name           = TYPE_LIS3DH, 
    .parent         = TYPE_I2C_SLAVE,
    .instance_size  = sizeof(LIS3DHState),
    .instance_init  = lis3dh_initfn,
    .class_init     = lis3dh_class_init,
};

/* Add LIS3DH object class into Qemu core */
static void lis3dh_i2c_register_types(void)
{
    type_register_static(&lis3dh_i2c_info);
}

type_init(lis3dh_i2c_register_types);