#include "qemu/osdep.h"
#include "hw/sensor/lis3dh.h"
#include "hw/i2c/i2c.h"
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
#include <float.h>

#define DEBUG_LIS3DH 1

#ifdef DEBUG_LIS3DH

#define SENSOR_LIS3DH(text, ...) \
    printf("LIS3DH: " text "\n", ## __VA_ARGS__ )
#else
#define DPRINTF_BUFFER(fmt, ...) do {} while(0)

#endif

static void lis3dh_set_So(LIS3DHState *src)
{
    switch (src->config->mode)
    {
        case LIS3DH_MODE_HIGH_RES:  // 12-bit
            switch (src->config->fscale)
            {
                case LIS3DH_FS_2G:  src->params->acc_So = LIS3DH_So_HIG_RES_2G;	 return; break;
                case LIS3DH_FS_4G:  src->params->acc_So = LIS3DH_So_HIG_RES_4G;	 return; break;
                case LIS3DH_FS_8G:  src->params->acc_So = LIS3DH_So_HIG_RES_8G;	 return; break;
                case LIS3DH_FS_16G: src->params->acc_So = LIS3DH_So_HIG_RES_16G; return; break;
                default:
                    src->params->acc_So = 4.0f; // Default: normal ±2g
                    qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: The FS: 0x%02x does not exist for this operating mode, So setted default value ±2g, So: %.2f\n",
                        __func__, src->config->fscale, src->params->acc_So);
                break;
            }
            break;

        case LIS3DH_MODE_NORMAL:  // 10-bit
            switch (src->config->fscale)
            {
                case LIS3DH_FS_2G:  src->params->acc_So = LIS3DH_So_NORMAL_2G;  return; break;
                case LIS3DH_FS_4G:  src->params->acc_So = LIS3DH_So_NORMAL_4G;  return; break;
                case LIS3DH_FS_8G:  src->params->acc_So = LIS3DH_So_NORMAL_8G;  return; break;
                case LIS3DH_FS_16G: src->params->acc_So = LIS3DH_So_NORMAL_16G; return; break;
                default:
                    src->params->acc_So = 4.0f; // Default: normal ±2g
                    qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: The FS: 0x%02x does not exist for this operating mode, So setted default value ±2g, So: %.2f\n",
                        __func__, src->config->fscale, src->params->acc_So);
                break;
            }
            break;

        case LIS3DH_MODE_LOW_POWER:  // 8-bit
            switch (src->config->fscale)
            {
                case LIS3DH_FS_2G:  src->params->acc_So = LIS3DH_So_LOW_POWER_2G;  return; break;
                case LIS3DH_FS_4G:  src->params->acc_So = LIS3DH_So_LOW_POWER_4G;  return; break;
                case LIS3DH_FS_8G:  src->params->acc_So = LIS3DH_So_LOW_POWER_8G;  return; break;
                case LIS3DH_FS_16G: src->params->acc_So = LIS3DH_So_LOW_POWER_16G; return; break;
                default:
                    src->params->acc_So = 4.0f; // Default: normal ±2g
                    qemu_log_mask(LOG_GUEST_ERROR,
                        "%s: The FS: 0x%02x does not exist for this operating mode, So setted default value ±2g, So: %.2f\n",
                        __func__, src->config->fscale, src->params->acc_So);
                break;
            }
            break;

        default:
            src->params->acc_So = 4.0f; // Default: normal ±2g
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: The operating mode 0x%02x does not exist, So setted to the default value ±2g, So: %.2f\n",
                      __func__, src->config->mode, src->params->acc_So);
            break;
    }
}

static void lis3dh_set_acc_range(LIS3DHState *src)
{
    switch (src->config->fscale)
    {
        case LIS3DH_FS_2G:
            src->params->acc_max_range = LIS3DH_MAX_2G_RANGE;
            src->params->acc_min_range = LIS3DH_MIN_2G_RANGE;
        break;
    
        case LIS3DH_FS_4G:
            src->params->acc_max_range = LIS3DH_MAX_4G_RANGE;
            src->params->acc_min_range = LIS3DH_MIN_4G_RANGE;
        break;

        case LIS3DH_FS_8G:
            src->params->acc_max_range = LIS3DH_MAX_8G_RANGE;
            src->params->acc_min_range = LIS3DH_MIN_8G_RANGE;
        break;

        case LIS3DH_FS_16G:
            src->params->acc_max_range = LIS3DH_MAX_16G_RANGE;
            src->params->acc_min_range = LIS3DH_MIN_16G_RANGE;
        break;

        default:
            src->params->acc_max_range = LIS3DH_MAX_2G_RANGE;
            src->params->acc_min_range = LIS3DH_MIN_2G_RANGE;
        break;
    }
}

static void lis3dh_set_acc_shifts(LIS3DHState *src)
{
    switch (src->config->mode)
    {        
        case LIS3DH_MODE_HIGH_RES:  src->params->acc_shifts = 4;    break;
        case LIS3DH_MODE_LOW_POWER: src->params->acc_shifts = 8;    break;
        case LIS3DH_MODE_NORMAL:    src->params->acc_shifts = 6;    break;
        default:
            src->params->acc_shifts = 6;
        break;
    }
}

static void lis3dh_set_operating_mode(LIS3DHState *src)
{
    lis3dh_mode_t mode = LIS3DH_MODE_NORMAL;

    if((src->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_LPEN) != 0)
        mode = LIS3DH_MODE_LOW_POWER;
    else
    {
        mode = (uint8_t)(((src->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_LPEN) >> 2 )
            | ((src->ctrl_reg4 & LIS3DH_CTRL_REG4_BIT_HR) >> 3 ));
        
        if (mode > LIS3DH_MODE_NOT_ALLOWED)
        {
            src->ctrl_reg1 &= ~LIS3DH_CTRL_REG1_BIT_LPEN;
            src->ctrl_reg4 &= ~LIS3DH_CTRL_REG4_BIT_HR;
            qemu_log_mask(LOG_GUEST_ERROR,
                "%s: The operating mode 0x%02x does not exist, setted to normal mode: 0x%02x\n",
                      __func__, mode, LIS3DH_MODE_NORMAL);
            mode = LIS3DH_MODE_NORMAL;
        }
    }
    
    src->config->mode = mode;
    lis3dh_set_So(src);
    lis3dh_set_acc_shifts(src);
}

static void lis3dh_set_odr(LIS3DHState *src)
{
    lis3dh_odr_t odr = (src->ctrl_reg1 & LIS3DH_CTRL_REG1_BITS_ODR) >> 4;
    
    if( odr < LIS3DH_ODR_POWER_DOWN || odr > LIS3DH_ODR_1250 )
    {
        qemu_log_mask(LOG_GUEST_ERROR,
            "%s: The ODR: 0x%02x does not exist, setted to LIS3DH_ODR_POWER_DOWN: %d\n",
            __func__, odr, LIS3DH_ODR_POWER_DOWN);
    
        odr = LIS3DH_ODR_POWER_DOWN;
        
        uint8_t ctrl_reg1 = src->ctrl_reg1;
        ctrl_reg1 &= ~LIS3DH_CTRL_REG1_BITS_ODR;
        src->ctrl_reg1 = ctrl_reg1 | (((uint8_t)(LIS3DH_ODR_POWER_DOWN)) << 4);
    }

    src->config->odr = odr;
}

static void lis3dh_set_fscale(LIS3DHState *src)
{
    lis3dh_fscale_t fscale = (src->ctrl_reg4 & LIS3DH_CTRL_REG4_BITS_FS) >> 4;

    if (fscale > LIS3DH_FS_16G)
        fscale = LIS3DH_FS_2G;

    src->config->fscale = fscale;
    lis3dh_set_acc_range(src);
    lis3dh_set_So(src);
}

/* ==================== DATA GENERATION ============================================== */

static int16_t lis3dh_get_acc_raw_data(LIS3DHState *src, float data)
{
    int16_t raw_data = 0x0000;

    /* Convert acceleration to sensor LSB format */
    float data_with_So = data / src->params->acc_So;

    /* Handle rounding */
    bool is_pos = (data_with_So >= 0);
    int16_t data_in_16b = 0x00;

    if ( fabsf(data_with_So - (int)data_with_So) >= 0.5f )
        data_in_16b = (int16_t)data_with_So + (is_pos ? 1 : -1); 
    else
        data_in_16b = (int16_t)data_with_So;

    /* Shift the data depending of the mode */
    raw_data = data_in_16b << src->params->acc_shifts;
    
    return raw_data;
}

static void lis3dh_set_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{ 
    LIS3DHState *s = LIS3DH(obj);

    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    /* Data Input In mg */
    int64_t value = 0;
    visit_type_int(v, name, &value, errp);

    if(value < s->params->acc_min_range || value > s->params->acc_max_range)
       value = 0;

    /* int64_t to float safe cast */
    float flt_value = 0.0;
    if ((value > (int64_t)FLT_MAX) || (value < (int64_t)-FLT_MAX)) 
        flt_value = 0.0;
    else
        flt_value = (float)value;

    int16_t raw_data = lis3dh_get_acc_raw_data(s, flt_value);

    s->out_x_h = (uint8_t)((raw_data & 0xFF00) >> 8);
    s->out_x_l = (uint8_t)(raw_data & 0x00FF);

    SENSOR_LIS3DH("out_x_h: 0x%x \n\n", s->out_x_h);
    SENSOR_LIS3DH("out_x_l: 0x%x \n\n", s->out_x_l);
}

static void lis3dh_set_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    /* Data Input In mg */
    int64_t value = 0;
    visit_type_int(v, name, &value, errp);

    if(value < s->params->acc_min_range || value > s->params->acc_max_range)
       value = 0;

    /* int64_t to float safe cast */
    float flt_value = 0.0;
    if ((value > (int64_t)FLT_MAX) || (value < (int64_t)-FLT_MAX)) 
        flt_value = 0.0;
    else
        flt_value = (float)value;

    int16_t raw_data = lis3dh_get_acc_raw_data(s, flt_value);

    s->out_y_h = (uint8_t)((raw_data & 0xFF00) >> 8);
    s->out_y_l = (uint8_t)(raw_data & 0x00FF);

    SENSOR_LIS3DH("out_y_h: 0x%x", s->out_y_h);
    SENSOR_LIS3DH("out_y_l: 0x%x", s->out_y_l);
}

static void lis3dh_set_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    /* Data Input In mg */
    int64_t value = 0;
    visit_type_int(v, name, &value, errp);

    if(value < s->params->acc_min_range || value > s->params->acc_max_range)
    {
       /** TODO: qemu error line */
       value = 0;
    }

    /* int64_t to float safe cast */
    float flt_value = 0.0;
    if ((value > (int64_t)FLT_MAX) || (value < (int64_t)-FLT_MAX)) 
        flt_value = 0.0;
    else
        flt_value = (float)value;

    /* get raw data */
    int16_t raw_data = lis3dh_get_acc_raw_data(s, flt_value);

    s->out_z_h = (uint8_t)((raw_data & 0xFF00) >> 8);
    s->out_z_l = (uint8_t)(raw_data & 0x00FF);

    SENSOR_LIS3DH("out_z_h: 0x%x", s->out_z_h);
    SENSOR_LIS3DH("out_z_l: 0x%x", s->out_z_l);
}

static void lis3dh_get_accel_x(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);
    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    int16_t raw = ((int16_t)s->out_x_h << 8) | s->out_x_l;
    int64_t value = (int64_t)((raw >> s->params->acc_shifts) * (s->params->acc_So));

    visit_type_int(v, name, &value, errp);
}

static void lis3dh_get_accel_y(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);
    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    int16_t raw = ((int16_t)s->out_y_h << 8) | s->out_y_l;
    int64_t value = (int64_t)((raw >> s->params->acc_shifts) * (s->params->acc_So));

    visit_type_int(v, name, &value, errp);
}

static void lis3dh_get_accel_z(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);
    if((s->ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_XEN) == 0 )
        return;

    int16_t raw = ((int16_t)s->out_z_h << 8) | s->out_z_l;
    int64_t value = (int64_t)((raw >> s->params->acc_shifts) * (s->params->acc_So));

    visit_type_int(v, name, &value, errp);
}


static bool lis3dh_temp_enable(LIS3DHState *src)
{
    return
    (
        (src->ctrl_reg4 & LIS3DH_CTRL_REG4_BIT_BDU) != 0 
        && (src->temp_cfg_reg & LIS3DH_TEMP_CFG_BIT_REG_ADC_EN) != 0
        && (src->temp_cfg_reg & LIS3DH_TEMP_CFG_BIT_REG_TEMP_EN) != 0
    );
}

static int64_t lis3dh_temp_get_data(LIS3DHState *src)
{
    uint8_t shifts = 0x00;
    if(src->config->mode == LIS3DH_MODE_LOW_POWER)
        shifts = 0x08;
    else
        shifts = 0x06;

    int16_t raw = ((int16_t)( ( (int16_t)src->adc_3_h << 8 ) | src->adc_3_l ) >> shifts );

    /* I assume a factory calibration point of 25ºC for an output of 0 */
    /* The TSDr is 1 digit/ºC = 1 LSB/ºC */
    int64_t temp = 25 + (int64_t)((raw*1.0f)/1);

    return temp;
}

static void lis3dh_set_temp(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);
    
    int64_t value = 0L;
    visit_type_int(v, name, &value, errp);

    if ( (value < LIS3DH_TEMP_MIN || value > LIS3DH_TEMP_MAX)
        &&  lis3dh_temp_enable(s))
    {
        /* I assume a factory calibration point of 25ºC for an output of 0 */
        /* The TSDr is 1 digit/ºC = 1 LSB/ºC (datasheet page 12/54)*/
        
        /**
         * T = raw * (1/TSDr) + 25
         * ºC = LSB * (ºC/LSB) + ºC
         *
         * raw = (T - 25.0) * TSDr
         * LSB = (ºC - ºC) * (LSB/ºC)
         */

        /**
         * - Output left justified
         * - Output in normal/high mode 10bit long
         * - Output in low power mode 8bit long
         */
        uint8_t shifts = 0x00;
        if(s->config->mode == LIS3DH_MODE_LOW_POWER)
            shifts = 0x08;
        else
            shifts = 0x06;

        int16_t raw = ((int16_t)(value - 25)) << shifts;

        s->adc_3_h = (uint8_t)((raw & 0xFF00) >> 8);
        s->adc_3_l = (uint8_t)(raw & 0x00FF); 
    }
    else if (lis3dh_temp_enable(s))
    {
        s->adc_3_h = 0x00;
        s->adc_3_l = 0x00;
    }

}

static void lis3dh_get_temp(Object *obj, Visitor *v, const char *name, void *opaque, Error **errp)
{
    LIS3DHState *s = LIS3DH(obj);

    int64_t value = 0x00;
    if(lis3dh_temp_enable(s))
        value = lis3dh_temp_get_data(s);  

    visit_type_int(v, name, &value, errp);
}

/* ==================== I2C FUNCTIONS ================================================ */

static void lis3dh_write_ctr_reg0(LIS3DHState *dst, uint8_t data)
{
    dst->ctrl_reg0 = data;
}

static void lis3dh_write_ctr_reg1(LIS3DHState *dst, uint8_t data)
{
    uint8_t ctrl_reg1 = dst->ctrl_reg1;
    dst->ctrl_reg1 = data;

    if ((ctrl_reg1 & LIS3DH_CTRL_REG1_BITS_ODR) != (data & LIS3DH_CTRL_REG1_BITS_ODR))
        lis3dh_set_odr(dst);

    if ((ctrl_reg1 & LIS3DH_CTRL_REG1_BIT_LPEN) != (data & LIS3DH_CTRL_REG1_BIT_LPEN))
        lis3dh_set_operating_mode(dst);
}

static void lis3dh_write_ctr_reg2(LIS3DHState *dst, uint8_t data)
{
    dst->ctrl_reg2 = data;
    /**
     * TODO:
     * High-pass filter logic
     */
}

static void lis3dh_write_ctr_reg3(LIS3DHState *dst, uint8_t data)
{
    dst->ctrl_reg3 = data;
    /**
     * TODO:
     * Interruptions logic
     */
}

static void lis3dh_write_ctr_reg4(LIS3DHState *dst, uint8_t data)
{
    uint8_t ctrl_reg4 = dst->ctrl_reg4;
    dst->ctrl_reg4 = data;

    if ((ctrl_reg4 & LIS3DH_CTRL_REG4_BITS_FS) != (data & LIS3DH_CTRL_REG4_BITS_FS))
        lis3dh_set_fscale(dst);

    if ((ctrl_reg4 & LIS3DH_CTRL_REG4_BIT_HR) != (data & LIS3DH_CTRL_REG4_BIT_HR))
        lis3dh_set_operating_mode(dst);

    /**
     * TODO:
     * - Big and Little endian logic
     */
}

static void lis3dh_write_ctr_reg5(LIS3DHState *dst, uint8_t data)
{
    dst->ctrl_reg5 = data;
    /**
     * TODO:
     * - FIFO logic
     * - Interruption Logic
     * - Latch logic
     */
}

static void lis3dh_write_ctr_reg6(LIS3DHState *dst, uint8_t data)
{
    dst->ctrl_reg6 = data;
    /**
     * TODO: 
     * - Interruptions logic
     * - 
     */
}

static bool lis3dh_write_register(LIS3DHState *dst, uint8_t dir, uint8_t data)
{ 
    switch (dir)
    {
        case LIS3DH_ADDR_CTRL_REG0:     lis3dh_write_ctr_reg0(dst, data); return true; break;
        case LIS3DH_ADDR_TEMP_CFG_REG:  dst->temp_cfg_reg = data; return true; break;
        case LIS3DH_ADDR_CTRL_REG1:     lis3dh_write_ctr_reg1(dst, data);  return true; break;
        case LIS3DH_ADDR_CTRL_REG2:     lis3dh_write_ctr_reg2(dst, data);  return true; break;
        case LIS3DH_ADDR_CTRL_REG3:     lis3dh_write_ctr_reg3(dst, data);  return true; break;
        case LIS3DH_ADDR_CTRL_REG4:     lis3dh_write_ctr_reg4(dst, data);  return true; break;
        case LIS3DH_ADDR_CTRL_REG5:     lis3dh_write_ctr_reg5(dst, data);  return true; break;
        case LIS3DH_ADDR_CTRL_REG6:     lis3dh_write_ctr_reg6(dst, data);  return true; break;

        case LIS3DH_ADDR_REFERENCE:     dst->reference = data;      return true; break;
        case LIS3DH_ADDR_FIFO_CTRL:     dst->fifo_ctrl_reg = data;  return true; break;
        case LIS3DH_ADDR_INT1_CFG:      dst->int1_cfg = data;       return true; break;
        case LIS3DH_ADDR_INT1_THS:      dst->int1_ths = data;       return true; break;
        case LIS3DH_ADDR_INT1_DURATION: dst->int1_duration = data;  return true; break;

        case LIS3DH_ADDR_INT2_CFG:      dst->int2_cfg = data;       return true; break;
        case LIS3DH_ADDR_INT2_THS:      dst->int2_ths = data;       return true; break;
        case LIS3DH_ADDR_INT2_DURATION: dst->int2_duration = data;  return true; break;

        case LIS3DH_ADDR_CLICK_CFG:     dst->click_cfg = data;      return true; break;
        case LIS3DH_ADDR_CLICK_THS:     dst->click_ths = data;      return true; break;
        case LIS3DH_ADDR_TIME_LIMIT:    dst->time_limit = data;     return true; break;
        case LIS3DH_ADDR_TIME_LATENCY:  dst->time_latency = data;   return true; break;
        case LIS3DH_ADDR_TIME_WINDOW:   dst->time_window = data;    return true; break;

        case LIS3DH_ADDR_ACT_THS:       dst->act_ths = data;        return true; break;
        case LIS3DH_ADDR_ACT_DUR:       dst->act_dur = data;        return true; break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: writing to unsupported register: 0x%02x\n",
                      __func__, dir);
            return false;
            break;
    }

    return false;

}

static uint8_t lis3dh_read_register( LIS3DHState *src )
{
    uint8_t ret = 0x00;

    switch (src->i2c_params.ptr)
    {             
        CASE_READ_RETURN(ret, LIS3DH_ADDR_STATUS_REG_AUX, status_reg_aux)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC1_L, adc_1_l)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC1_H, adc_1_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC2_L, adc_2_l)        
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC2_H, adc_2_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC3_L, adc_3_l)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_ADC3_H, adc_3_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_WHO_AM_I, who_am_i)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG0 , ctrl_reg0)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_TEMP_CFG_REG, temp_cfg_reg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG1, ctrl_reg1)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG2, ctrl_reg2)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG3, ctrl_reg3)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG4, ctrl_reg4)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG5, ctrl_reg5)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CTRL_REG6, ctrl_reg6)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_REFERENCE, reference)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_STATUS_REG, status_reg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_X_L, out_x_l)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_X_H, out_x_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_Y_L, out_y_l)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_Y_H, out_y_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_Z_L, out_z_l)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_OUT_Z_H, out_z_h)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_FIFO_CTRL, fifo_ctrl_reg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_FIFO_SRC, fifo_src_reg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT1_CFG, int1_cfg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT1_SRC, int1_src)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT1_THS, int1_ths)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT1_DURATION, int1_duration)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT2_CFG, int2_cfg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT2_SRC, int2_src)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT2_THS, int2_ths)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_INT2_DURATION, int2_duration)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CLICK_CFG, click_cfg)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CLICK_SRC, click_src)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_CLICK_THS, click_ths)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_TIME_LIMIT, time_limit)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_TIME_LATENCY, time_latency)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_TIME_WINDOW, time_window)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_ACT_THS, act_ths)
        CASE_READ_RETURN(ret, LIS3DH_ADDR_ACT_DUR, act_dur)

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: reading from unsupported register: 0x%02x\n",
                      __func__, src->i2c_params.ptr);
            return false;
        break;
    }

    return ret;
}

static int lis3dh_i2c_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS3DHState *lis3dh = LIS3DH(i2c);
    
    switch (event) 
    {
        case I2C_START_SEND:    // Start of write operation
            /* Master is starting a WRITE operation 
            (sending data to the device) */
            lis3dh->i2c_params.ptr             = 0xFF;
			lis3dh->i2c_params.auto_increment  = false;
			lis3dh->i2c_params.address_phase   = true;
            break;
            
        case I2C_START_RECV:    // Start of read operation
            /* Master is starting a READ operation 
            (requesting data from the device) */
			if (lis3dh->i2c_params.ptr == 0xFF)
				lis3dh->i2c_params.ptr = LIS3DH_ADDR_WHO_AM_I;

			lis3dh->i2c_params.address_phase = false;
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
    LIS3DHState *lis3dh = LIS3DH(i2c);

	if (lis3dh->i2c_params.ptr == 0xFF && lis3dh->i2c_params.address_phase)
	{
		lis3dh->i2c_params.ptr = data & LIS3DH_SUB_REG_MASK;
		lis3dh->i2c_params.auto_increment =  (data & LIS3DH_SUB_AUTO_INC_MASK) != 0x00 ? true : false;
		lis3dh->i2c_params.address_phase = false;
	}else
	{
		lis3dh_write_register( lis3dh, lis3dh->i2c_params.ptr, data);
		if (lis3dh->i2c_params.auto_increment)
			lis3dh->i2c_params.ptr++;
	}
	return 0;
}

static uint8_t lis3dh_i2c_recv(I2CSlave *i2c)
{
    LIS3DHState *lis3dh = LIS3DH(i2c);

    uint8_t value = lis3dh_read_register( lis3dh );

    if (lis3dh->i2c_params.auto_increment)
    {
        lis3dh->i2c_params.ptr++;
    }

	return value;
}

/* ==================== LIS3DH IN QEMU =============================================== */

static void lis3dh_reset_registers(LIS3DHState *lis3dh)
{
    lis3dh->status_reg_aux  = LIS3DH_STATUS_REG_AUX_DEF;    // Status Register

    lis3dh->adc_1_l         = LIS3DH_ADC_1_L_DEF;           // 1-Axis Acceleration Data Low Register
    lis3dh->adc_1_h         = LIS3DH_ADC_1_H_DEF;           // 1-Axis Acceleration Data High Register
    lis3dh->adc_2_l         = LIS3DH_ADC_2_L_DEF;           // 2-Axis Acceleration Data Low Register
    lis3dh->adc_2_h         = LIS3DH_ADC_2_H_DEF;           // 2-Axis Acceleration Data High Register
    lis3dh->adc_3_l         = LIS3DH_ADC_3_L_DEF;           // 3-Axis Acceleration Data Low Register
    lis3dh->adc_3_h         = LIS3DH_ADC_3_H_DEF;           // 3-Axis Acceleration Data High Register

    lis3dh->who_am_i        = LIS3DH_WHO_AM_I_DEF;          // Device identification Register 

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

static void lis3dh_realize(DeviceState *dev, Error **errp)
{
    LIS3DHState *lis3dh = LIS3DH(dev);
    
    /* Initialize I2C state */
    lis3dh->i2c_params.address         = LIS3DH_DEFAULT_ADDRESS;
	lis3dh->i2c_params.ptr             = 0xFF;
	lis3dh->i2c_params.auto_increment  = false;
	lis3dh->i2c_params.address_phase   = false;

    lis3dh->config = (lis3dh_config_t*)g_malloc(sizeof(lis3dh_config_t));

    lis3dh->config->fscale  = LIS3DH_FS_2G;
    lis3dh->config->mode    = LIS3DH_MODE_NORMAL;
    lis3dh->config->odr     = LIS3DH_ODR_100;
    lis3dh->config->temp_enable         = false;
    lis3dh->config->high_pass_filter    = false;
    lis3dh->config->fifo_enabled        = false;

    lis3dh->params = (lis3dh_params_t*)g_malloc(sizeof(lis3dh_params_t));

    lis3dh_set_So(lis3dh);
    lis3dh_set_acc_range(lis3dh);
    lis3dh_set_acc_shifts(lis3dh);

    /* Reset registers */
    lis3dh_reset_registers(lis3dh);

}

static void lis3dh_unrealize(DeviceState *dev)
{
    LIS3DHState *lis3dh = LIS3DH(dev);
    
    g_free(lis3dh->config);
    g_free(lis3dh->params);
}

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
    object_property_add(obj, "temp", "int",
                        lis3dh_get_temp,
                        lis3dh_set_temp, NULL, NULL);
}

/* LIS3DH class initialization */
static void lis3dh_class_init( ObjectClass *kclass, void *data )
{
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

/* Creation the LIS3DH object class. */
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