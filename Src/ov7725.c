/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7725 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
//#include STM32_HAL_H
//#include "cambus.h"
#include "ov7725.h"
#include "ov7725_regs.h"
//#include "systick.h"
#include "omv_boardconfig.h"

const uint8_t default_regs[][2] = {
		//extra�do de Application Note, para XCLK = 24MHz
		//OV7720, YCbCr, VGA
		//15fps at 24MHz input clock, 4x maximum gain
		//2/27/2007
		{COM3, 0x10},//@p�coli
		{COM7, COM7_RES_QVGA|COM7_FMT_YUV},//@p�coli
		{COM12, 0x03},
		{HSTART, 0x49},
		{HSIZE, 0x28},
		{VSTART, 0x03},
		{VSIZE, 0x3c},
		{HREF, 0x00},
		{HOUTSIZE, 0x28},//muda para QQVGA
		{VOUTSIZE, 0x3c},//muda para QQVGA
		{EXHCH, 0x00},

		// Disable auto-scaling/zooming factors
		{DSPAUTO,0xF3},//@p�coli, conforme projeto openmv, ov725.set_framesize
		{ADVFL, 0x00},//@p�coli
		{ADVFH, 0x00},//@p�coli
		{CLKRC, 0x00},//@p�coli
		{0x42, 0x7f},
		{0x4d, 0x09},
		{0x63, 0xe0},
		{0x64, 0xff},
		//{DSP_CTRL2, DSP_CTRL2_VZOOM_EN|DSP_CTRL2_HZOOM_EN},//habilita downsampling a partir de VGA
		//{SCAL0, 0x05},//@P�coli: 1/2 downsampling vertical e horizontal
		//{SCAL1,0x10},
		//{SCAL2,0x10},
		{0x66, 0x00},
		{DSP_CTRL4, DSP_CTRL4_YUV_RGB},//{0x67, 0x48},
		{0x13, 0xf0},
		{0x0d, 0x41}, // 0x51/0x61/0x71 for different AEC/AGC window
		{0x0f, 0xc5},
		{0x14, 0x11},
		{0x22, 0x7f},
		{0x23, 0x03},
		{0x24, 0x40},
		{0x25, 0x30},
		{0x26, 0xa1},
		{0x2b, 0x00},
		{0x6b, 0xaa},
		{0x13, 0xff},
		{0x90, 0x05},
		{0x91, 0x01},
		{0x92, 0x03},
		{0x93, 0x00},
		{0x94, 0xb0},
		{0x95, 0x9d},
		{0x96, 0x13},
		{0x97, 0x16},
		{0x98, 0x7b},
		{0x99, 0x91},
		{0x9a, 0x1e},
		{0x9b, 0x08},
		{0x9c, 0x20},
		{0x9e, 0x81},
		{0xa6, 0x06},
		//Gamma
		{0x7e, 0x0c},
		{0x7f, 0x16},
		{0x80, 0x2a},
		{0x81, 0x4e},
		{0x82, 0x61},
		{0x83, 0x6f},
		{0x84, 0x7b},
		{0x85, 0x86},
		{0x86, 0x8e},
		{0x87, 0x97},
		{0x88, 0xa4},
		{0x89, 0xaf},
		{0x8a, 0xc5},
		{0x8b, 0xd7},
		{0x8c, 0xe8},
		{0x8d, 0x20},

		{0x33, 0x2b},//@p�coli
		{0x34, 0x00},//@p�coli
		{0x22, 0x7f},
		{0x23, 0x03},
		// for 25 fps, 50Hz
		//{0x33, 0x66),
		//{0x22, 0x99},

		// Frame reduction in night mode.
		//write_SCCB(0x23, 0x03);
		// Lens Correction, should be tuned with real camera module
		{0x4a, 0x10},
		{0x49, 0x10},
		{0x4b, 0x14},
		{0x4c, 0x17},
		{0x46, 0x05},
		{0x0e, 0x65},

    {0x00,          0x00},
};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
    {0x38, 0x0e}, /* -4 */
    {0x28, 0x0e}, /* -3 */
    {0x18, 0x0e}, /* -2 */
    {0x08, 0x0e}, /* -1 */
    {0x08, 0x06}, /*  0 */
    {0x18, 0x06}, /* +1 */
    {0x28, 0x06}, /* +2 */
    {0x38, 0x06}, /* +3 */
    {0x48, 0x06}, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][1] = {
    {0x10}, /* -4 */
    {0x14}, /* -3 */
    {0x18}, /* -2 */
    {0x1C}, /* -1 */
    {0x20}, /*  0 */
    {0x24}, /* +1 */
    {0x28}, /* +2 */
    {0x2C}, /* +3 */
    {0x30}, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
    {0x00, 0x00}, /* -4 */
    {0x10, 0x10}, /* -3 */
    {0x20, 0x20}, /* -2 */
    {0x30, 0x30}, /* -1 */
    {0x40, 0x40}, /*  0 */
    {0x50, 0x50}, /* +1 */
    {0x60, 0x60}, /* +2 */
    {0x70, 0x70}, /* +3 */
    {0x80, 0x80}, /* +4 */
};
/*
static int reset(sensor_t *sensor)
{
    int i=0;
    const uint8_t (*regs)[2];

    // Reset all registers
    cambus_writeb(sensor->slv_addr, COM7, COM7_RESET);

    // Delay 10 ms
    HAL_Delay(10);

    // Write default registers
    for (i=0, regs = default_regs; regs[i][0]; i++) {
        cambus_writeb(sensor->slv_addr, regs[i][0], regs[i][1]);
    }

    // Delay
    HAL_Delay(30);

    return 0;
}

static int sleep(sensor_t *sensor, int enable)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM2, &reg);

    if (enable) {
        reg |= COM2_SOFT_SLEEP;
    } else {
        reg &= ~COM2_SOFT_SLEEP;
    }

    // Write back register
    return cambus_writeb(sensor->slv_addr, COM2, reg) | ret;
}

static int read_reg(sensor_t *sensor, uint8_t reg_addr)
{
    uint8_t reg_data;
    if (cambus_readb(sensor->slv_addr, reg_addr, &reg_data) != 0) {
        return -1;
    }
    return reg_data;
}

static int write_reg(sensor_t *sensor, uint8_t reg_addr, uint16_t reg_data)
{
    return cambus_writeb(sensor->slv_addr, reg_addr, reg_data);
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM7, &reg);

    switch (pixformat) {
        case PIXFORMAT_RGB565:
            reg = COM7_SET_FMT(reg, COM7_FMT_RGB);
            ret = cambus_writeb(sensor->slv_addr, DSP_CTRL4, 0);
            break;
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
            reg = COM7_SET_FMT(reg, COM7_FMT_YUV);
            ret = cambus_writeb(sensor->slv_addr, DSP_CTRL4, 0);
            break;
        case PIXFORMAT_BAYER:
            reg = COM7_SET_FMT(reg, COM7_FMT_P_BAYER);
            ret = cambus_writeb(sensor->slv_addr, DSP_CTRL4, DSP_CTRL4_RAW8);
            break;

        default:
            return -1;
    }

    // Write back register
    return cambus_writeb(sensor->slv_addr, COM7, reg) | ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
    int ret=0;
    uint16_t w = resolution[framesize][0];
    uint16_t h = resolution[framesize][1];

    // Write MSBs
    ret |= cambus_writeb(sensor->slv_addr, HOUTSIZE, w>>2);
    ret |= cambus_writeb(sensor->slv_addr, VOUTSIZE, h>>1);

    // Write LSBs
    ret |= cambus_writeb(sensor->slv_addr, EXHCH, ((w&0x3) | ((h&0x1) << 2)));

    if ((w <= 320) && (h <= 240)) {
        // Set QVGA Resolution
        uint8_t reg;
        int ret = cambus_readb(sensor->slv_addr, COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_QVGA);
        ret |= cambus_writeb(sensor->slv_addr, COM7, reg);

        // Set QVGA Window Size
        ret |= cambus_writeb(sensor->slv_addr, HSTART, 0x3F);
        ret |= cambus_writeb(sensor->slv_addr, HSIZE,  0x50);
        ret |= cambus_writeb(sensor->slv_addr, VSTART, 0x03);
        ret |= cambus_writeb(sensor->slv_addr, VSIZE,  0x78);
        ret |= cambus_writeb(sensor->slv_addr, HREF,   0x00);

        // Enable auto-scaling/zooming factors
        ret |= cambus_writeb(sensor->slv_addr, DSPAUTO, 0xFF);
    } else {
        // Set VGA Resolution
        uint8_t reg;
        int ret = cambus_readb(sensor->slv_addr, COM7, &reg);
        reg = COM7_SET_RES(reg, COM7_RES_VGA);
        ret |= cambus_writeb(sensor->slv_addr, COM7, reg);

        // Set VGA Window Size
        ret |= cambus_writeb(sensor->slv_addr, HSTART, 0x23);
        ret |= cambus_writeb(sensor->slv_addr, HSIZE,  0xA0);
        ret |= cambus_writeb(sensor->slv_addr, VSTART, 0x07);
        ret |= cambus_writeb(sensor->slv_addr, VSIZE,  0xF0);
        ret |= cambus_writeb(sensor->slv_addr, HREF,   0x00);

        // Disable auto-scaling/zooming factors
        ret |= cambus_writeb(sensor->slv_addr, DSPAUTO, 0xF3);

        // Clear auto-scaling/zooming factors
        ret |= cambus_writeb(sensor->slv_addr, SCAL0, 0x00);
        ret |= cambus_writeb(sensor->slv_addr, SCAL1, 0x00);
        ret |= cambus_writeb(sensor->slv_addr, SCAL2, 0x00);
    }

    return ret;
}

static int set_framerate(sensor_t *sensor, framerate_t framerate)
{
    return 0;
}

static int set_contrast(sensor_t *sensor, int level)
{
    level += (NUM_CONTRAST_LEVELS / 2);
    if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
        return -1;
    }

    return cambus_writeb(sensor->slv_addr, CONTRAST, contrast_regs[level][0]);
}

static int set_brightness(sensor_t *sensor, int level)
{
    int ret=0;
    level += (NUM_BRIGHTNESS_LEVELS / 2);
    if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
        return -1;
    }

    ret |= cambus_writeb(sensor->slv_addr, BRIGHTNESS, brightness_regs[level][0]);
    ret |= cambus_writeb(sensor->slv_addr, SIGN_BIT,   brightness_regs[level][1]);
    return ret;
}

static int set_saturation(sensor_t *sensor, int level)
{
    int ret=0;

    level += (NUM_SATURATION_LEVELS / 2 );
    if (level < 0 || level >= NUM_SATURATION_LEVELS) {
        return -1;
    }

    ret |= cambus_writeb(sensor->slv_addr, USAT, saturation_regs[level][0]);
    ret |= cambus_writeb(sensor->slv_addr, VSAT, saturation_regs[level][1]);
    return ret;
}

static int set_gainceiling(sensor_t *sensor, gainceiling_t gainceiling)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM9, &reg);

    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    return cambus_writeb(sensor->slv_addr, COM9, reg) | ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM3, &reg);
    
    // Enable colorbar test pattern output
    reg = COM3_SET_CBAR(reg, enable);
    ret |= cambus_writeb(sensor->slv_addr, COM3, reg);

    // Enable DSP colorbar output
    ret |= cambus_readb(sensor->slv_addr, DSP_CTRL3, &reg);
    reg = DSP_CTRL3_SET_CBAR(reg, enable);
    return cambus_writeb(sensor->slv_addr, DSP_CTRL3, reg) | ret;
}

static int set_auto_gain(sensor_t *sensor, int enable, float gain_db, float gain_db_ceiling)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);
    ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AGC(reg, (enable != 0)));

    if ((enable == 0) && (!isnanf(gain_db)) && (!isinff(gain_db))) {
        float gain = IM_MAX(IM_MIN(fast_expf((gain_db / 20.0) * fast_log(10.0)), 32.0), 1.0);

        int gain_temp = fast_roundf(fast_log2(IM_MAX(gain / 2.0, 1.0)));
        int gain_hi = 0xF >> (4 - gain_temp);
        int gain_lo = IM_MIN(fast_roundf(((gain / (1 << gain_temp)) - 1.0) * 16.0), 15);

        ret |= cambus_writeb(sensor->slv_addr, GAIN, (gain_hi << 4) | (gain_lo << 0));
    } else if ((enable != 0) && (!isnanf(gain_db_ceiling)) && (!isinff(gain_db_ceiling))) {
        float gain_ceiling = IM_MAX(IM_MIN(fast_expf((gain_db_ceiling / 20.0) * fast_log(10.0)), 32.0), 2.0);

        ret |= cambus_readb(sensor->slv_addr, COM9, &reg);
        ret |= cambus_writeb(sensor->slv_addr, COM9, (reg & 0x8F) | ((fast_ceilf(fast_log2(gain_ceiling)) - 1) << 4));
    }

    return ret;
}

static int get_gain_db(sensor_t *sensor, float *gain_db)
{
    uint8_t reg, gain;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AGC(reg, 0));
    // }
    // DISABLED

    ret |= cambus_readb(sensor->slv_addr, GAIN, &gain);

    // DISABLED
    // if (reg & COM8_AGC_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AGC(reg, 1));
    // }
    // DISABLED

    int hi_gain = 1 << (((gain >> 7) & 1) + ((gain >> 6) & 1) + ((gain >> 5) & 1) + ((gain >> 4) & 1));
    float lo_gain = 1.0 + (((gain >> 0) & 0xF) / 16.0);
    *gain_db = 20.0 * (fast_log(hi_gain * lo_gain) / fast_log(10.0));

    return ret;
}

static int set_auto_exposure(sensor_t *sensor, int enable, int exposure_us)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);
    ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AEC(reg, (enable != 0)));

    if ((enable == 0) && (exposure_us >= 0)) {
        ret |= cambus_readb(sensor->slv_addr, COM7, &reg);

        int t_line = (reg & COM7_RES_QVGA) ? (320 + 256) : (640 + 144);
        int t_pclk = (COM7_GET_FMT(reg) == COM7_FMT_P_BAYER) ? 1 : 2;

        ret |= cambus_readb(sensor->slv_addr, COM4, &reg);
        int pll_mult = 0;

        if (COM4_GET_PLL(reg) == COM4_PLL_BYPASS) pll_mult = 1;
        if (COM4_GET_PLL(reg) == COM4_PLL_4x) pll_mult = 4;
        if (COM4_GET_PLL(reg) == COM4_PLL_6x) pll_mult = 6;
        if (COM4_GET_PLL(reg) == COM4_PLL_8x) pll_mult = 8;

        ret |= cambus_readb(sensor->slv_addr, CLKRC, &reg);
        int clk_rc = 0;

        if (reg & CLKRC_NO_PRESCALE) {
            clk_rc = 1;
        } else {
            clk_rc = ((reg & CLKRC_PRESCALER) + 1) * 2;
        }

        int exposure = IM_MAX(IM_MIN(((exposure_us*(((OMV_XCLK_FREQUENCY/clk_rc)*pll_mult)/1000000))/t_pclk)/t_line,0xFFFF),0x0000);

        ret |= cambus_writeb(sensor->slv_addr, AEC, ((exposure >> 0) & 0xFF));
        ret |= cambus_writeb(sensor->slv_addr, AECH, ((exposure >> 8) & 0xFF));
    }

    return ret;
}

static int get_exposure_us(sensor_t *sensor, int *exposure_us)
{
    uint8_t reg, aec_l, aec_h;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AEC(reg, 0));
    // }
    // DISABLED

    ret |= cambus_readb(sensor->slv_addr, AEC, &aec_l);
    ret |= cambus_readb(sensor->slv_addr, AECH, &aec_h);

    // DISABLED
    // if (reg & COM8_AEC_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AEC(reg, 1));
    // }
    // DISABLED

    ret |= cambus_readb(sensor->slv_addr, COM7, &reg);

    int t_line = (reg & COM7_RES_QVGA) ? (320 + 256) : (640 + 144);
    int t_pclk = (COM7_GET_FMT(reg) == COM7_FMT_P_BAYER) ? 1 : 2;

    ret |= cambus_readb(sensor->slv_addr, COM4, &reg);
    int pll_mult = 0;

    if (COM4_GET_PLL(reg) == COM4_PLL_BYPASS) pll_mult = 1;
    if (COM4_GET_PLL(reg) == COM4_PLL_4x) pll_mult = 4;
    if (COM4_GET_PLL(reg) == COM4_PLL_6x) pll_mult = 6;
    if (COM4_GET_PLL(reg) == COM4_PLL_8x) pll_mult = 8;

    ret |= cambus_readb(sensor->slv_addr, CLKRC, &reg);
    int clk_rc = 0;

    if (reg & CLKRC_NO_PRESCALE) {
        clk_rc = 1;
    } else {
        clk_rc = ((reg & CLKRC_PRESCALER) + 1) * 2;
    }

    *exposure_us = (((aec_h<<8)+(aec_l<<0))*t_line*t_pclk)/(((OMV_XCLK_FREQUENCY/clk_rc)*pll_mult)/1000000);

    return ret;
}

static int set_auto_whitebal(sensor_t *sensor, int enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);
    ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AWB(reg, (enable != 0)));

    if ((enable == 0) && (!isnanf(r_gain_db)) && (!isnanf(g_gain_db)) && (!isnanf(b_gain_db))
                      && (!isinff(r_gain_db)) && (!isinff(g_gain_db)) && (!isinff(b_gain_db))) {
        ret |= cambus_readb(sensor->slv_addr, AWB_CTRL1, &reg);
        float gain_div = (reg & 0x2) ? 64.0 : 128.0;

        int r_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((r_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);
        int g_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((g_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);
        int b_gain = IM_MAX(IM_MIN(fast_roundf(fast_expf((b_gain_db / 20.0) * fast_log(10.0)) * gain_div), 255), 0);

        ret |= cambus_writeb(sensor->slv_addr, BLUE, b_gain);
        ret |= cambus_writeb(sensor->slv_addr, RED, r_gain);
        ret |= cambus_writeb(sensor->slv_addr, GREEN, g_gain);
    }

    return ret;
}

static int get_rgb_gain_db(sensor_t *sensor, float *r_gain_db, float *g_gain_db, float *b_gain_db)
{
    uint8_t reg, blue, red, green;
    int ret = cambus_readb(sensor->slv_addr, COM8, &reg);

    // DISABLED
    // if (reg & COM8_AWB_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AWB(reg, 0));
    // }
    // DISABLED

    ret |= cambus_readb(sensor->slv_addr, BLUE, &blue);
    ret |= cambus_readb(sensor->slv_addr, RED, &red);
    ret |= cambus_readb(sensor->slv_addr, GREEN, &green);

    // DISABLED
    // if (reg & COM8_AWB_EN) {
    //     ret |= cambus_writeb(sensor->slv_addr, COM8, COM8_SET_AWB(reg, 1));
    // }
    // DISABLED

    ret |= cambus_readb(sensor->slv_addr, AWB_CTRL1, &reg);
    float gain_div = (reg & 0x2) ? 64.0 : 128.0;

    *r_gain_db = 20.0 * (fast_log(red / gain_div) / fast_log(10.0));
    *g_gain_db = 20.0 * (fast_log(green / gain_div) / fast_log(10.0));
    *b_gain_db = 20.0 * (fast_log(blue / gain_div) / fast_log(10.0));

    return ret;
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM3, &reg);
    ret |= cambus_writeb(sensor->slv_addr, COM3, COM3_SET_MIRROR(reg, enable)) ;

    return ret;
}

static int set_vflip(sensor_t *sensor, int enable)
{
    uint8_t reg;
    int ret = cambus_readb(sensor->slv_addr, COM3, &reg);
    ret |= cambus_writeb(sensor->slv_addr, COM3, COM3_SET_FLIP(reg, enable));

    return ret;
}

static int set_special_effect(sensor_t *sensor, sde_t sde)
{
    int ret=0;

    switch (sde) {
        case SDE_NEGATIVE:
            ret |= cambus_writeb(sensor->slv_addr, SDE, 0x46);
            break;
        case SDE_NORMAL:
            ret |= cambus_writeb(sensor->slv_addr, SDE, 0x06);
            ret |= cambus_writeb(sensor->slv_addr, UFIX, 0x80);
            ret |= cambus_writeb(sensor->slv_addr, UFIX, 0x80);
            break;
        default:
            return -1;
    }

    return ret;
}

static int set_lens_correction(sensor_t *sensor, int enable, int radi, int coef)
{
    int ret=0;

    ret |= cambus_writeb(sensor->slv_addr, LC_CTR, (enable&0x01));
    ret |= cambus_writeb(sensor->slv_addr, LC_RADI, radi);
    ret |= cambus_writeb(sensor->slv_addr, LC_COEF, coef);
    return ret;
}

int ov7725_init(sensor_t *sensor)
{
    // Initialize sensor structure.
    sensor->gs_bpp              = 2;
    sensor->reset               = reset;
    sensor->sleep               = sleep;
    sensor->read_reg            = read_reg;
    sensor->write_reg           = write_reg;
    sensor->set_pixformat       = set_pixformat;
    sensor->set_framesize       = set_framesize;
    sensor->set_framerate       = set_framerate;
    sensor->set_contrast        = set_contrast;
    sensor->set_brightness      = set_brightness;
    sensor->set_saturation      = set_saturation;
    sensor->set_gainceiling     = set_gainceiling;
    sensor->set_colorbar        = set_colorbar;
    sensor->set_auto_gain       = set_auto_gain;
    sensor->get_gain_db         = get_gain_db;
    sensor->set_auto_exposure   = set_auto_exposure;
    sensor->get_exposure_us     = get_exposure_us;
    sensor->set_auto_whitebal   = set_auto_whitebal;
    sensor->get_rgb_gain_db     = get_rgb_gain_db;
    sensor->set_hmirror         = set_hmirror;
    sensor->set_vflip           = set_vflip;
    sensor->set_special_effect  = set_special_effect;
    sensor->set_lens_correction = set_lens_correction;

    // Set sensor flags
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_VSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_HSYNC, 0);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_PIXCK, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_FSYNC, 1);
    SENSOR_HW_FLAGS_SET(sensor, SENSOR_HW_FLAGS_JPEGE, 0);

    return 0;
}
*/
