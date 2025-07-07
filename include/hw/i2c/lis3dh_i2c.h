
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"
#include "qom/object.h"

/* Directions */
#define LIS3DH_STATUS_REG_AUX   0x07
#define LIS3DH_ADC_1_L          0x08
#define LIS3DH_ADC_1_H          0x09
#define LIS3DH_ADC_2_L          0x0A
#define LIS3DH_ADC_2_H          0x0B
#define LIS3DH_ADC_3_L          0x0C
#define LIS3DH_ADC_3_H          0x0D
// direction 0x0E reserved
#define LIS3DH_WHO_AM_I         0x0F
// directions [0x10-0x1D] reserved
#define LIS3DH_CTRL_REG0        0x1E
#define LIS3DH_TEMP_CFG_REG     0x1F
#define LIS3DH_CTRL_REG1        0x20
#define LIS3DH_CTRL_REG2        0x21
#define LIS3DH_CTRL_REG3        0x22
#define LIS3DH_CTRL_REG4        0x23
#define LIS3DH_CTRL_REG5        0x24
#define LIS3DH_CTRL_REG6        0x25
#define LIS3DH_REFERENCE        0x26
#define LIS3DH_STATUS_REG       0x27
#define LIS3DH_OUT_X_L          0x28
#define LIS3DH_OUT_X_H          0x29
#define LIS3DH_OUT_Y_L          0x2A
#define LIS3DH_OUT_Y_H          0x2B
#define LIS3DH_OUT_Z_L          0x2C
#define LIS3DH_OUT_Z_H          0x2D
#define LIS3DH_FIFO_CTRL_REG    0x2E
#define LIS3DH_FIFO_SRC_REG     0x2F
#define LIS3DH_INT1_CFG         0x30
#define LIS3DH_INT1_SRC         0x31
#define LIS3DH_INT1_THS         0x32
#define LIS3DH_INT1_DURATION    0x33
#define LIS3DH_INT2_CFG         0x34
#define LIS3DH_INT2_SRC         0x35
#define LIS3DH_INT2_THS         0x36
#define LIS3DH_INT2_DURATION    0x37
#define LIS3DH_CLICK_CFG        0x38
#define LIS3DH_CLICK_SRC        0x39
#define LIS3DH_CLICK_THS        0x3A
#define LIS3DH_TIME_LIMIT       0x3B
#define LIS3DH_TIME_LATENCY     0x3C
#define LIS3DH_TIME_WINDOW      0x3D
#define LIS3DH_ACT_THS          0x3E
#define LIS3DH_ACT_DUR          0x3F

/* Default */
#define LIS3DH_WHO_AM_I_DEFAULT         0x33    //00110011 (default in write)
#define LIS3DH_CTRL_REG0_DEFAULT        0x10
#define LIS3DH_TEMP_CFG_REG_DEFAULT     0x00
#define LIS3DH_CTRL_REG1_DEFAULT        0x07
#define LIS3DH_CTRL_REG2_DEFAULT        0x00
#define LIS3DH_CTRL_REG3_DEFAULT        0x00
#define LIS3DH_CTRL_REG4_DEFAULT        0x00
#define LIS3DH_CTRL_REG5_DEFAULT        0x00
#define LIS3DH_CTRL_REG6_DEFAULT        0x00
#define LIS3DH_REFERENCE_DEFAULT        0x00
#define LIS3DH_STATUS_REG_DEFAULT       0x00
#define LIS3DH_FIFO_CTRL_REG_DEFAULT    0x00
#define LIS3DH_FIFO_SRC_REG_DEFAULT     0x00
#define LIS3DH_INT1_CFG_DEFAULT         0x00
#define LIS3DH_INT1_THS_DEFAULT         0x00
#define LIS3DH_INT1_DURATION_DEFAULT    0x00
#define LIS3DH_TIME_LIMIT_DEFAULT       0x00
#define LIS3DH_TIME_LATENCY_DEFAULT     0x00
#define LIS3DH_TIME_WINDOW_DEFAULT      0x00
#define LIS3DH_ACT_THS_DEFAULT          0x00
#define LIS3DH_ACT_DUR_DEFAULT          0x00
#define LIS3DH_INT2_CFG_DEFAULT         0x00
#define LIS3DH_INT2_THS_DEFAULT         0x00
#define LIS3DH_INT2_DURATION_DEFAULT    0x00
#define LIS3DH_CLICK_CFG_DEFAULT        0x00
#define LIS3DH_CLICK_SRC_DEFAULT        0x00
#define LIS3DH_CLICK_THS_DEFAULT        0x00
#define LIS3DH_TIME_LIMIT_DEFAULT       0x00
#define LIS3DH_TIME_LATENCY_DEFAULT     0x00
#define LIS3DH_TIME_WINDOW_DEFAULT      0x00
#define LIS3DH_ACT_THS_DEFAULT          0x00
#define LIS3DH_ACT_DUR_DEFAULT          0x00

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
    /* My parent object */
    I2CSlave parent_obj;        //

    /* Critical Fields */
    uint8_t pointer;           // Current register pointer
    bool data_ready;           // Data ready flag
    bool command_phase;        // I2C command phase tracker
    QEMUTimer *timer;          // Data update timer

    /* Registers */
    uint8_t status_reg_aux;     //

    //uint8_t adc_reg[0x07];      //
    
    uint8_t who_am_i;           //
    
    //uint8_t ctrl_reg[0x08]      //
    
    uint8_t reference_reg;      //
    
    //uint8_t status_reg;         //

    uint8_t out_x_reg[0x02];    //
    uint8_t out_y_reg[0x02];    //
    uint8_t out_z_reg[0x02];    //

    //uint8_t fifo_ctrl_reg;      //
    //uint8_t fifo_src_reg;       //

    //uint8_t int1_reg[0x04]      //
    //uint8_t int2_reg[0x04]      //
   
    uint8_t click_cfg;          //
    uint8_t click_src;          //
    uint8_t click_ths;          //

    uint8_t time_limit;         //
    uint8_t time_latency;       //
    uint8_t time_window;        //

    //uint8_t act_reg[0x02];      //

} LIS3DHState;