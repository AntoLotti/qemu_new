
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"

/* Directions */
// directions [0x00-0x06] reserved
#define LIS3DH_STATUS_REG_AUX       0x07    // Status Register
#define LIS3DH_ADC_1_L              0x08    // 1-Axis Acceleration Data Low Register
#define LIS3DH_ADC_1_H              0x09    // 1-Axis Acceleration Data High Register
#define LIS3DH_ADC_2_L              0x0A    // 2-Axis Acceleration Data Low Register
#define LIS3DH_ADC_2_H              0x0B    // 2-Axis Acceleration Data High Register
#define LIS3DH_ADC_3_L              0x0C    // 3-Axis Acceleration Data Low Register
#define LIS3DH_ADC_3_H              0x0D    // 3-Axis Acceleration Data High Register
// direction 0x0E reserved
#define LIS3DH_WHO_AM_I             0x0F    //Device identification Register 
// directions [0x10-0x1D] reserved
#define LIS3DH_CTRL_REG0            0x1E
#define LIS3DH_TEMP_CFG_REG         0x1F    // Temperature Sensor Register
#define LIS3DH_CTRL_REG1            0x20    // Accelerometer Control Register 1
#define LIS3DH_CTRL_REG2            0x21    // Accelerometer Control Register 2
#define LIS3DH_CTRL_REG3            0x22    // Accelerometer Control Register 3
#define LIS3DH_CTRL_REG4            0x23    // Accelerometer Control Register 4
#define LIS3DH_CTRL_REG5            0x24    // Accelerometer Control Register 5
#define LIS3DH_CTRL_REG6            0x25    // Accelerometer Control Register 6
#define LIS3DH_REFERENCE            0x26    // Reference/Datacapture Register
#define LIS3DH_STATUS_REG           0x27    // Status Register 2
#define LIS3DH_OUT_X_L              0x28    // X-Axis Acceleration Data Low Register
#define LIS3DH_OUT_X_H              0x29    // X-Axis Acceleration Data High Register
#define LIS3DH_OUT_Y_L              0x2A    // Y-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Y_H              0x2B    // Y-Axis Acceleration Data High Register
#define LIS3DH_OUT_Z_L              0x2C    // Z-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Z_H              0x2D    // Z-Axis Acceleration Data High Register
#define LIS3DH_FIFO_CTRL_REG        0x2E    // FIFO Control Register
#define LIS3DH_FIFO_SRC_REG         0x2F    // FIFO Source Register
#define LIS3DH_INT1_CFG             0x30    // Interrupt 1 Configuration Register
#define LIS3DH_INT1_SRC             0x31    // Interrupt 1 Source Register
#define LIS3DH_INT1_THS             0x32    // Interrupt 1 Threshold Register
#define LIS3DH_INT1_DURATION        0x33    // Interrupt 1 Duration Register
#define LIS3DH_INT2_CFG             0x34    // Interrupt 2 Configuration Register
#define LIS3DH_INT2_SRC             0x35    // Interrupt 2 Source Register
#define LIS3DH_INT2_THS             0x36    // Interrupt 2 Threshold Register
#define LIS3DH_INT2_DURATION        0x37    // Interrupt 2 Duration Register
#define LIS3DH_CLICK_CFG            0x38    // Interrupt Click Recognition Register
#define LIS3DH_CLICK_SRC            0x39    // Interrupt Click Source Register
#define LIS3DH_CLICK_THS            0x3A    // Interrupt Click Threshold Register
#define LIS3DH_TIME_LIMIT           0x3B    // Click Time Limit Register
#define LIS3DH_TIME_LATENCY         0x3C    // Click Time Latency Register
#define LIS3DH_TIME_WINDOW          0x3D    // Click Time Window Register
#define LIS3DH_ACT_THS              0x3E
#define LIS3DH_ACT_DUR              0x3F

/* Default */
#define LIS3DH_REGS_DEF         0x00
#define LIS3DH_OUTPUTS_DEF      0x00

// directions [0x00-0x06] reserved
#define LIS3DH_STATUS_REG_AUX_DEF       LIS3DH_OUTPUTS_DEF      // Status Register
#define LIS3DH_ADC_1_L_DEF              LIS3DH_OUTPUTS_DEF      // 1-Axis Acceleration Data Low Register
#define LIS3DH_ADC_1_H_DEF              LIS3DH_OUTPUTS_DEF      // 1-Axis Acceleration Data High Register
#define LIS3DH_ADC_2_L_DEF              LIS3DH_OUTPUTS_DEF      // 2-Axis Acceleration Data Low Register
#define LIS3DH_ADC_2_H_DEF              LIS3DH_OUTPUTS_DEF      // 2-Axis Acceleration Data High Register
#define LIS3DH_ADC_3_L_DEF              LIS3DH_OUTPUTS_DEF      // 3-Axis Acceleration Data Low Register
#define LIS3DH_ADC_3_H_DEF              LIS3DH_OUTPUTS_DEF      // 3-Axis Acceleration Data High Register
// direction 0x0E reserved
#define LIS3DH_WHO_AM_I_DEF             0x33                    //Device identification Register 00110011 (default in write)
// directions [0x10-0x1D] reserved
#define LIS3DH_CTRL_REG0_DEF            0x10
#define LIS3DH_TEMP_CFG_REG_DEF         LIS3DH_REGS_DEF         // Temperature Sensor Register
#define LIS3DH_CTRL_REG1_DEF            0x07                    // Accelerometer Control Register 1
#define LIS3DH_CTRL_REG2_DEF            LIS3DH_REGS_DEF         // Accelerometer Control Register 2
#define LIS3DH_CTRL_REG3_DEF            LIS3DH_REGS_DEF         // Accelerometer Control Register 3
#define LIS3DH_CTRL_REG4_DEF            LIS3DH_REGS_DEF         // Accelerometer Control Register 4
#define LIS3DH_CTRL_REG5_DEF            LIS3DH_REGS_DEF         // Accelerometer Control Register 5
#define LIS3DH_CTRL_REG6_DEF            LIS3DH_REGS_DEF         // Accelerometer Control Register 6
#define LIS3DH_REFERENCE_DEF            LIS3DH_REGS_DEF         // Reference/Datacapture Register
#define LIS3DH_STATUS_REG_DEF           LIS3DH_OUTPUTS_DEF      // Status Register 2
#define LIS3DH_OUT_X_L_DEF              LIS3DH_OUTPUTS_DEF      // X-Axis Acceleration Data Low Register
#define LIS3DH_OUT_X_H_DEF              LIS3DH_OUTPUTS_DEF      // X-Axis Acceleration Data High Register
#define LIS3DH_OUT_Y_L_DEF              LIS3DH_OUTPUTS_DEF      // Y-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Y_H_DEF              LIS3DH_OUTPUTS_DEF      // Y-Axis Acceleration Data High Register
#define LIS3DH_OUT_Z_L_DEF              LIS3DH_OUTPUTS_DEF      // Z-Axis Acceleration Data Low Register
#define LIS3DH_OUT_Z_H_DEF              LIS3DH_OUTPUTS_DEF      // Z-Axis Acceleration Data High Register
#define LIS3DH_FIFO_CTRL_REG_DEF        LIS3DH_REGS_DEF         // FIFO Control Register
#define LIS3DH_FIFO_SRC_REG_DEF         LIS3DH_REGS_DEF         // FIFO Source Register
#define LIS3DH_INT1_CFG_DEF             LIS3DH_REGS_DEF         // Interrupt 1 Configuration Register
#define LIS3DH_INT1_SRC_DEF             LIS3DH_OUTPUTS_DEF      // Interrupt 1 Source Register
#define LIS3DH_INT1_THS_DEF             LIS3DH_REGS_DEF         // Interrupt 1 Threshold Register
#define LIS3DH_INT1_DURATION_DEF        LIS3DH_REGS_DEF         // Interrupt 1 Duration Register
#define LIS3DH_INT2_CFG_DEF             LIS3DH_REGS_DEF         // Interrupt 2 Configuration Register
#define LIS3DH_INT2_SRC_DEF             LIS3DH_OUTPUTS_DEF      // Interrupt 2 Source Register
#define LIS3DH_INT2_THS_DEF             LIS3DH_REGS_DEF         // Interrupt 2 Threshold Register
#define LIS3DH_INT2_DURATION_DEF        LIS3DH_REGS_DEF         // Interrupt 2 Duration Register
#define LIS3DH_CLICK_CFG_DEF            LIS3DH_REGS_DEF         // Interrupt Click Recognition Register
#define LIS3DH_CLICK_SRC_DEF            LIS3DH_OUTPUTS_DEF      // Interrupt Click Source Register
#define LIS3DH_CLICK_THS_DEF            LIS3DH_REGS_DEF         // Interrupt Click Threshold Register
#define LIS3DH_TIME_LIMIT_DEF           LIS3DH_REGS_DEF         // Click Time Limit Register
#define LIS3DH_TIME_LATENCY_DEF         LIS3DH_REGS_DEF         // Click Time Latency Register
#define LIS3DH_TIME_WINDOW_DEF          LIS3DH_REGS_DEF         // Click Time Window Register
#define LIS3DH_ACT_THS_DEF              LIS3DH_REGS_DEF
#define LIS3DH_ACT_DUR_DEF              LIS3DH_REGS_DEF

/************************/
/*  Device Structure    */
/*  and Initialization  */
/************************/

/* Declaration of the QOM for the LIS3DH */
#define TYPE_LIS3DH_I2C "lis3dh-i2c"
OBJECT_DECLARE_SIMPLE_TYPE(LIS3DHState, LIS3DH_I2C);

/* LIS3DH State struct requirements (the hardware) */
typedef struct LIS3DHState
{
    /* Parent object */
    I2CSlave i2c;

    /* Critical Fields */
    uint8_t address;       // 7-bit I2C address
	uint8_t ptr;           // Current register pointer
	bool auto_increment;   // Auto-advance pointer after access
	bool data_ready;       // Data ready flag
	bool command_phase;    // I2C command phase tracker

    /* Registers */
    // directions [0x00-0x06] reserved
    uint8_t status_reg_aux;     // Status Register
    uint8_t adc_1_l;            // 1-Axis Acceleration Data Low Register
    uint8_t adc_1_h;            // 1-Axis Acceleration Data High Register
    uint8_t adc_2_l;            // 2-Axis Acceleration Data Low Register
    uint8_t adc_2_h;            // 2-Axis Acceleration Data High Register
    uint8_t adc_3_l;            // 3-Axis Acceleration Data Low Register
    uint8_t adc_3_h;            // 3-Axis Acceleration Data High Register
    // direction 0x0E reserved
    uint8_t who_am_i;           // Device identification Register 
    // directions [0x10-0x1D] reserved
    uint8_t ctrl_reg0;
    uint8_t temp_cfg_reg;       // Temperature Sensor Register
    uint8_t ctrl_reg1;          // Accelerometer Control Register 1
    uint8_t ctrl_reg2;          // Accelerometer Control Register 2
    uint8_t ctrl_reg3;          // Accelerometer Control Register 3
    uint8_t ctrl_reg4;          // Accelerometer Control Register 4
    uint8_t ctrl_reg5;          // Accelerometer Control Register 5
    uint8_t ctrl_reg6;          // Accelerometer Control Register 6
    uint8_t reference;          // Reference/Datacapture Register
    uint8_t status_reg;         // Status Register 2
    uint8_t out_x_l;            // X-Axis Acceleration Data Low Register
    uint8_t out_x_h;            // X-Axis Acceleration Data High Register
    uint8_t out_y_l;            // Y-Axis Acceleration Data Low Register
    uint8_t out_y_h;            // Y-Axis Acceleration Data High Register
    uint8_t out_z_l;            // Z-Axis Acceleration Data Low Register
    uint8_t out_z_h;            // Z-Axis Acceleration Data High Register
    uint8_t fifo_ctrl_reg;      // FIFO Control Register
    uint8_t fifo_src_reg;       // FIFO Source Register
    uint8_t int1_cfg;           // Interrupt 1 Configuration Register
    uint8_t int1_src;           // Interrupt 1 Source Register
    uint8_t int1_ths;           // Interrupt 1 Threshold Register
    uint8_t int1_duration;      // Interrupt 1 Duration Register
    uint8_t int2_cfg;           // Interrupt 2 Configuration Register
    uint8_t int2_src;           // Interrupt 2 Source Register
    uint8_t int2_ths;           // Interrupt 2 Threshold Register
    uint8_t int2_duration;      // Interrupt 2 Duration Register
    uint8_t click_cfg;          // Interrupt Click Recognition Register
    uint8_t click_src;          // Interrupt Click Source Register
    uint8_t click_ths;          // Interrupt Click Threshold Register
    uint8_t time_limit;         // Click Time Limit Register
    uint8_t time_latency;       // Click Time Latency Register
    uint8_t time_window;        // Click Time Window Register
    uint8_t act_ths;            //
    uint8_t act_dur;            //
} LIS3DHState;