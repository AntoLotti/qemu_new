#include "qemu/osdep.h"
#include "hw/i2c/lis3dh_i2c.h"
#include "qom/object.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include <math.h>

static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event);
static int lis3dh_i2c_send(I2CSlave *i2c, uint8_t data);
static uint8_t lis3dh_i2c_recv(I2CSlave *i2c);

static bool __reserved_address( uint8_t src);
static bool __write_in_register( LIS3DHState *dst, uint8_t dir, uint8_t src );
static uint8_t __read_register( LIS3DHState *src );

static void __lis3dh_i2c_update_data(void *src);
static void __lis3dh_i2c_reset(LIS3DHState *lis3dh);
static void lis3dh_i2c_realize(DeviceState *dev, Error **errp);
static void lis3dh_i2c_unrealize(DeviceState *dev);

static int16_t __round_and_transform(float src);
static void __acc_x_axis_data_generation(uint8_t out_x_h, uint8_t out_x_l);
static void __acc_y_axis_data_generation(uint8_t out_y_h, uint8_t out_y_l);
static void __acc_z_axis_data_generation(uint8_t out_z_h, uint8_t out_z_l);

/**************************************************************************
    ACCELEROMETER DATA GENERATION
**************************************************************************/
static int16_t __round_and_transform(float src)
{
    float decimal = fabsf(src - (int)src);

    if (decimal >= 0.5f)
    {
        return (int16_t)(src) + src <= 0 ? -1 : 1 ;   
    }    

    return (int16_t)src;
}

static void __acc_x_axis_data_generation(uint8_t out_x_h, uint8_t out_x_l)
{
    // Data Generation In m/s² //
    float x_axis_data = -10.7;

    /**
     * TODO
     * Ensure that the raw data fit into
     * a int16_t with out overflow
     * 
     * limit the range to ensure it 
     */

    // Transform In Raw Data //
    float temp = x_axis_data / ( 0.001f * LIS3DH_ACCELERATION_CONST);   // -1090.7
    int16_t x_raw_data = __round_and_transform(temp);   // -1091 = 1111 1011 1011 1101

    /**
     * TODO
     * 
     * 1. Round The x_raw_data Up If Decimal Part Equal Or Bigger 
     * That 0.5
     * 
     * 2. add the calculations with the others modes
     * Now only avilable the high-resolution mode
     */

    // Split Data Into Registers //
    out_x_h = (uint8_t)( (x_raw_data & 0xFF00) >> 8 );  // 1111 1011
    out_x_l = (uint8_t)(x_raw_data & 0x00FF);           // 1011 1101

}

static void __acc_y_axis_data_generation(uint8_t out_y_h, uint8_t out_y_l)
{
    // Data Generation In m/s² //
    float x_axis_data = -10.7;

    /**
     * TODO
     * Ensure that the raw data fit into
     * a int16_t with out overflow
     * 
     * limit the range to ensure it 
     */

    // Transform In Raw Data //
    float temp = x_axis_data / ( 0.001f * LIS3DH_ACCELERATION_CONST);   // -1090.7
    int16_t x_raw_data = __round_and_transform(temp);   // -1091 = 1111 1011 1011 1101

    /**
     * TODO
     * 
     * 1. Round The x_raw_data Up If Decimal Part Equal Or Bigger 
     * That 0.5
     * 
     * 2. add the calculations with the others modes
     * Now only avilable the high-resolution mode
     */

    // Split Data Into Registers //
    out_y_h = (uint8_t)( (x_raw_data & 0xFF00) >> 8 );  // 1111 1011
    out_y_l = (uint8_t)(x_raw_data & 0x00FF);           // 1011 1101
}

static void __acc_z_axis_data_generation(uint8_t out_z_h, uint8_t out_z_l)
{
    // Data Generation In m/s² //
    float x_axis_data = -10.7;

    /**
     * TODO
     * Ensure that the raw data fit into
     * a int16_t with out overflow
     * 
     * limit the range to ensure it 
     */

    // Transform In Raw Data //
    float temp = x_axis_data / ( 0.001f * LIS3DH_ACCELERATION_CONST);   // -1090.7
    int16_t x_raw_data = __round_and_transform(temp);   // -1091 = 1111 1011 1011 1101

    /**
     * TODO
     * 
     * 1. Round The x_raw_data Up If Decimal Part Equal Or Bigger 
     * That 0.5
     * 
     * 2. add the calculations with the others modes
     * Now only avilable the high-resolution mode
     */

    // Split Data Into Registers //
    out_z_h = (uint8_t)( (x_raw_data & 0xFF00) >> 8 );  // 1111 1011
    out_z_l = (uint8_t)(x_raw_data & 0x00FF);           // 1011 1101
}

/**************************************************************************
    DEVICE LIFE FUNCTIONS
**************************************************************************/
static void __lis3dh_i2c_update_data(void *src)
{
    LIS3DHState *lis3dh = src;
    
    /* Generate new accelerometer values */
    __acc_x_axis_data_generation( lis3dh->out_x_h, lis3dh->out_x_l );
    __acc_y_axis_data_generation( lis3dh->out_y_h, lis3dh->out_y_l );
    __acc_z_axis_data_generation( lis3dh->out_z_h, lis3dh->out_z_l );
        
    /* Set data ready flag */
    //s->status_reg |= 0x08;  // Set DRDY bit
    
    /* Reschedule timer */
    timer_mod
    (
        lis3dh->timer, 
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100
    );
}

static void __lis3dh_i2c_reset(LIS3DHState *lis3dh)
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
    lis3dh->out_x_l         = LIS3DH_OUT_X_L_DEF;           // X-Axis Acceleration Data Low Register
    lis3dh->out_x_h         = LIS3DH_OUT_X_H_DEF;           // X-Axis Acceleration Data High Register
    lis3dh->out_y_l         = LIS3DH_OUT_Y_L_DEF;           // Y-Axis Acceleration Data Low Register
    lis3dh->out_y_h         = LIS3DH_OUT_Y_H_DEF;           // Y-Axis Acceleration Data High Register
    lis3dh->out_z_l         = LIS3DH_OUT_Z_L_DEF;           // Z-Axis Acceleration Data Low Register
    lis3dh->out_z_h         = LIS3DH_OUT_Z_H_DEF;           // Z-Axis Acceleration Data High Register
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

static void lis3dh_i2c_realize(DeviceState *dev, Error **errp)
{
    LIS3DHState *lis3dh = LIS3DH_I2C(dev);
    
    /* Initialize I2C state */
    lis3dh->address         = LIS3DH_DEFAULT_ADDRESS;
	lis3dh->ptr             = 0xFF;
	lis3dh->auto_increment  = false;
	//lis3dh->data_ready      = false;
	lis3dh->address_phase   = false;

    /* Create data update timer */
    lis3dh->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, __lis3dh_i2c_update_data, lis3dh);
    timer_mod
    (
        lis3dh->timer, 
        qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + NANOSECONDS_PER_SECOND / 100
    ); // 100Hz update
    
    /* Reset registers */
    __lis3dh_i2c_reset(lis3dh);
        
    /* Enable hotplug */
    DeviceClass *dc = DEVICE_GET_CLASS(dev);
    dc->hotpluggable = true;
}

static void lis3dh_i2c_unrealize(DeviceState *dev)
{
    LIS3DHState *lis3dh = LIS3DH_I2C(dev);

    timer_del(lis3dh->timer);
    timer_free(lis3dh->timer);
}

/**************************************************************************
    I2C FUNCTIONS
**************************************************************************/
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
        CASE_WRITE_RETURN(LIS3DHTR_REG_ACCEL_WHO_AM_I, who_am_i)
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
    if (__reserved_address(src->ptr))
        return 0;
    
    switch (src->ptr)
    {             
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_STATUS_REG_AUX, status_reg_aux)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC1_L, adc_1_l)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC1_H, adc_1_h)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC2_L, adc_2_l)        
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC2_H, adc_2_h)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC3_L, adc_3_l)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_ADC3_H, adc_3_h)
        CASE_READ_RETURN(LIS3DHTR_REG_ACCEL_WHO_AM_I, who_am_i)
        CASE_READ_RETURN(LIS3DH_REG_CTRL_REG0, ctrl_reg0)
        CASE_READ_RETURN(LIS3DH_REG_TEMP_CFG_REG, temp_cfg_reg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG1, ctrl_reg1)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG2, ctrl_reg2)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG3, ctrl_reg3)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG4, ctrl_reg4)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG5, ctrl_reg5)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CTRL_REG6, ctrl_reg6)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_REFERENCE, reference)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_STATUS_REG, status_reg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_X_L, out_x_l)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_X_H, out_x_h)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_Y_L, out_y_l)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_Y_H, out_y_h)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_Z_L, out_z_l)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_OUT_Z_H, out_z_h)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_FIFO_CTRL, fifo_ctrl_reg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_FIFO_SRC, fifo_src_reg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT1_CFG, int1_cfg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT1_SRC, int1_src)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT1_THS, int1_ths)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT1_DURATION, int1_duration)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT2_CFG, int2_cfg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT2_SRC, int2_src)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT2_THS, int2_ths)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_INT2_DURATION, int2_duration)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CLICK_CFG, click_cfg)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CLICK_SRC, click_src)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_CLICK_THS, click_ths)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_TIME_LIMIT, time_limit)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_TIME_LATENCY, time_latency)
        CASE_READ_RETURN(LIS3DH_REG_ACCEL_TIME_WINDOW, time_window)
        CASE_READ_RETURN(LIS3DH_ACT_THS, act_ths)
        CASE_READ_RETURN(LIS3DH_ACT_DUR, act_dur)

        default:
            //#TODO print error message
            return false;
            break;
    }

    return 0x00;
}

static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS3DHState *lis3dh = LIS3DH_I2C(i2c);
    
    switch (event) 
    {
        case I2C_START_SEND:    // Start of write operation
            /* Master is starting a WRITE operation 
            (sending data to the device) */
            lis3dh->ptr             = 0xFF;
			lis3dh->auto_increment  = false;
			//lis3dh->data_ready      = false;
			lis3dh->address_phase   = true;    // next data register addres
            break;
            
        case I2C_START_RECV:    // Start of read operation
            /* Master is starting a READ operation 
            (requesting data from the device) */
			if (lis3dh->ptr == 0xFF)
				lis3dh->ptr = LIS3DHTR_REG_ACCEL_WHO_AM_I;
			lis3dh->address_phase = false;
            break;
            
        case I2C_FINISH:        // Stop condition
            /* Master ends the transaction 
            (STOP condition) */			
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
    LIS3DHState *lis3dh = LIS3DH_I2C(i2c);

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
    LIS3DHState *lis3dh = LIS3DH_I2C(i2c);
	return __read_register( lis3dh );
}

/**************************************************************************
    LIS3DH REGISTRATION IN QEMU 
**************************************************************************/
/* LIS3DH class initialization */
static void lis3dh_i2c_class_init( ObjectClass *kclass, const void *data )
{
    DeviceClass *dc     = DEVICE_CLASS(kclass);     // The generic device class operations
    I2CSlaveClass *k    = I2C_SLAVE_CLASS(kclass);  // The I2C-specific interface implementation
    
    /* Device lifecycle */
    dc->realize     = lis3dh_i2c_realize;           // Called when device created
    dc->unrealize   = lis3dh_i2c_unrealize;         // Clean up
    dc->desc        = "I2C accelerometer: LIS3DH"; 
    
    /* I2C protocol implementation */
    k->event    = lis3dh_i2c_event;                 // Bus events: START/STOP/NACK
    k->send     = lis3dh_i2c_send;                  // Master writes to device
    k->recv     = lis3dh_i2c_recv;                  // Master reads from device
}

/* Tells QEMU’s type system how to create and wire the LIS3DH object class. */
static const TypeInfo lis3dh_i2c_info = 
{
    .name           = TYPE_LIS3DH_I2C, 
    .parent         = TYPE_I2C_SLAVE,
    .instance_size  = sizeof(LIS3DHState),
    .class_init     = lis3dh_i2c_class_init,
};

/* Add LIS3DH object class into Qemu core */
static void lis3dh_i2c_register_types(void)
{
    type_register_static(&lis3dh_i2c_info);
}

type_init(lis3dh_i2c_register_types);