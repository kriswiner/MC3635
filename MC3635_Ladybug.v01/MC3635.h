/******************************************************************************
 *
 * Copyright (c) 2018 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *****************************************************************************/

/**
 * @file    MC3635.h
 * @author  mCube
 * @date    10 May 2018
 * @brief   Driver interface header file for accelerometer MC3635 series.
 * @see     http://www.mcubemems.com
 */

#ifndef MC3635_h
#define MC3635_h

/******************************************************************************
 *** INFORMATION
 *****************************************************************************/
#define M_DRV_MC3635_VERSION    "2.0.0"

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC3635_RETCODE_SUCCESS                 (0)
#define MC3635_RETCODE_ERROR_BUS               (-1)
#define MC3635_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC3635_RETCODE_ERROR_STATUS            (-3)
#define MC3635_RETCODE_ERROR_SETUP             (-4)
#define MC3635_RETCODE_ERROR_GET_DATA          (-5)
#define MC3635_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC3635_RETCODE_ERROR_NO_DATA           (-7)
#define MC3635_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC3635_FIFO_DEPTH                        32
#define MC3635_REG_MAP_SIZE                      64

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC3635_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC3635_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC3635_INTR_C_IAH_ACTIVE_LOW         (0x00)
#define MC3635_INTR_C_IAH_ACTIVE_HIGH        (0x02)

/******************************************************************************
 *** Register Map
 * https://mcubemems.com/wp-content/uploads/2020/10/AN-012-SNIFF-Mode-for-MC3600-Series-v1.1.pdf
 * 
 * https://mcubemems.com/wp-content/uploads/2019/11/MC3635-Datasheet-APS-048-0044v1.7.pdf
 *****************************************************************************/
#define MC3635_REG_EXT_STAT_1       (0x00)
#define MC3635_REG_EXT_STAT_2       (0x01)
#define MC3635_REG_XOUT_LSB         (0x02)
#define MC3635_REG_XOUT_MSB         (0x03)
#define MC3635_REG_YOUT_LSB         (0x04)
#define MC3635_REG_YOUT_MSB         (0x05)
#define MC3635_REG_ZOUT_LSB         (0x06)
#define MC3635_REG_ZOUT_MSB         (0x07)
#define MC3635_REG_STATUS_1         (0x08)
#define MC3635_REG_STATUS_2         (0x09)
#define MC3635_REG_FEATURE_CTL      (0x0D)
#define MC3635_REG_PWR_CONTROL      (0X0F)
#define MC3635_REG_MODE_C           (0x10)
#define MC3635_REG_WAKE_C           (0x11)
#define MC3635_REG_SNIFF_C          (0x12)
#define MC3635_REG_SNIFFTH_C        (0x13)
#define MC3635_REG_SNIFF_CONF_C     (0x14)
#define MC3635_REG_RANGE_C          (0x15)
#define MC3635_REG_FIFO_C           (0x16)
#define MC3635_REG_INTR_C           (0x17)
#define MC3635_REG_PROD             (0x18)
#define MC3635_REG_PMCR             (0x1C)
#define MC3635_REG_DMX              (0x20)
#define MC3635_REG_DMY              (0x21)
#define MC3635_REG_GAIN             (0x21)
#define MC3635_REG_DMZ              (0x22)
#define MC3635_REG_RESET            (0x24)
#define MC3635_REG_XOFFL            (0x2A)
#define MC3635_REG_XOFFH            (0x2B)
#define MC3635_REG_YOFFL            (0x2C)
#define MC3635_REG_YOFFH            (0x2D)
#define MC3635_REG_ZOFFL            (0x2E)
#define MC3635_REG_ZOFFH            (0x2F)
#define MC3635_REG_XGAIN            (0x30)
#define MC3635_REG_YGAIN            (0x31)
#define MC3635_REG_ZGAIN            (0x32)
#define MC3635_REG_OPT              (0x3B)
#define MC3635_REG_LOC_X            (0x3C)
#define MC3635_REG_LOC_Y            (0x3D)
#define MC3635_REG_LOT_dAOFSZ       (0x3E)
#define MC3635_REG_WAF_LOT          (0x3F)

#define MC3635_NULL_ADDR            (0)

struct MC3635_acc_t
{
    short XAxis;
    short YAxis;
    short ZAxis;
    float XAxis_g;
    float YAxis_g;
    float ZAxis_g;
} ;

typedef enum
{
    MC3635_GAIN_DEFAULT    = 0b00,
    MC3635_GAIN_4X         = 0b01,
    MC3635_GAIN_1X         = 0b10,
    MC3635_GAIN_NOT_USED   = 0b11,
}   MC3635_gain_t;

typedef enum
{
    MC3635_MODE_SLEEP      = 0b000,
    MC3635_MODE_STANDBY    = 0b001,
    MC3635_MODE_SNIFF      = 0b010,
    MC3635_MODE_CWAKE      = 0b101,
    MC3635_MODE_TRIG       = 0b111,
}   MC3635_mode_t;

typedef enum
{
    MC3635_RANGE_2G    = 0b000,
    MC3635_RANGE_4G    = 0b001,
    MC3635_RANGE_8G    = 0b010,
    MC3635_RANGE_16G   = 0b011,
    MC3635_RANGE_12G   = 0b100,
    MC3635_RANGE_END,
}   MC3635_range_t;

typedef enum
{
    MC3635_RESOLUTION_6BIT    = 0b000,
    MC3635_RESOLUTION_7BIT    = 0b001,
    MC3635_RESOLUTION_8BIT    = 0b010,
    MC3635_RESOLUTION_10BIT   = 0b011,
    MC3635_RESOLUTION_12BIT   = 0b100,
    MC3635_RESOLUTION_14BIT   = 0b101,  //(Do not select if FIFO enabled)
    MC3635_RESOLUTION_END,
}   MC3635_resolution_t;

typedef enum
{
    MC3635_CWAKE_SR_DEFAULT_54Hz = 0b0000,
    MC3635_CWAKE_SR_14Hz         = 0b0101,
    MC3635_CWAKE_SR_28Hz         = 0b0110,
    MC3635_CWAKE_SR_54Hz         = 0b0111,
    MC3635_CWAKE_SR_105Hz        = 0b1000,
    MC3635_CWAKE_SR_210Hz        = 0b1001,
    MC3635_CWAKE_SR_400Hz        = 0b1010,
    MC3635_CWAKE_SR_600Hz        = 0b1011,
    MC3635_CWAKE_SR_END,
}   MC3635_cwake_sr_t;

typedef enum
{
    MC3635_SNIFF_SR_DEFAULT_7Hz = 0b0000,
    MC3635_SNIFF_SR_0p4Hz       = 0b0001,
    MC3635_SNIFF_SR_0p8Hz       = 0b0010,
    MC3635_SNIFF_SR_1p5Hz       = 0b0011,
    MC3635_SNIFF_SR_7Hz         = 0b0100,
    MC3635_SNIFF_SR_14Hz        = 0b0101,
    MC3635_SNIFF_SR_28Hz        = 0b0110,
    MC3635_SNIFF_SR_54Hz        = 0b0111,
    MC3635_SNIFF_SR_105Hz       = 0b1000,
    MC3635_SNIFF_SR_210Hz       = 0b1001,
    MC3635_SNIFF_SR_400Hz       = 0b1010,
    MC3635_SNIFF_SR_600Hz       = 0b1011,
    MC3635_SNIFF_SR_END,
}   MC3635_sniff_sr_t;

typedef enum
{
    MC3635_FIFO_CTL_DISABLE = 0,
    MC3635_FIFO_CTL_ENABLE,
    MC3635_FIFO_CTL_END,
}   MC3635_fifo_ctl_t;

typedef enum
{
    MC3635_FIFO_MODE_NORMAL = 0,
    MC3635_FIFO_MODE_WATERMARK,
    MC3635_FIFO_MODE_END,
}   MC3635_fifo_mode_t;

typedef enum
{
    MC3635_ANDORN_OR = 0,
    MC3635_ANDORN_AND,
    MC3635_ANDORN_END,
}   MC3635_andorn_t;

typedef enum
{
  //Compare to previous
    MC3635_DELTA_MODE_C2P = 0,
  //Compare to baseline
    MC3635_DELTA_MODE_C2B,
    MC3635_DELTA_MODE_END,
}   MC3635_delta_mode_t;

typedef struct
{
    unsigned char    bWAKE;
    unsigned char    bACQ;
    unsigned char    bFIFO_EMPTY;
    unsigned char    bFIFO_FULL;
    unsigned char    bFIFO_THRESHOLD;
    unsigned char    bRESV;
    unsigned char    bSWAKE_SNIFF;
    unsigned char    baPadding[2];
}   MC3635_interrupt_event_t;

typedef enum
{
    MC3635_AXIS_X = 0,
    MC3635_AXIS_Y,
    MC3635_AXIS_Z,
    MC3635_AXIS_END,
}   MC3635_axis_t;

typedef struct
{
    // Sensor wakes from sniff mode.
    unsigned char    bWAKE;
    // New sample is ready and acquired.
    unsigned char    bACQ;
    // FIFO is empty.
    unsigned char    bFIFO_EMPTY;
    // FIFO is full.
    unsigned char    bFIFO_FULL;
    // FIFO sample count is equal to or greater than the threshold count.
    unsigned char    bFIFO_THRESHOLD;
    // Reserved
    unsigned char    bRESV;
    unsigned char    baPadding[2];
}   MC3635_InterruptEvent;

/* general accel methods */
class MC3635{
 public:
    MC3635(I2Cdev* i2c_bus);
    // Setup and begin measurements
    bool start();
    // Start measurement
    void wake();
    // End measurement
    void stop();
  // Sensor reset
    void reset();
    // Sensor sniff
    void sniff();
    void sniffreset();
    void SetMode(MC3635_mode_t mode);
    void SetRangeCtrl(MC3635_range_t range);
    void SetResolutionCtrl(MC3635_resolution_t resolution);
    void SetCWakeSampleRate(MC3635_cwake_sr_t sample_rate);
  void SetSniffSampleRate(MC3635_sniff_sr_t sniff_sr);
  void SetFIFOCtrl(MC3635_fifo_ctl_t fifo_ctl,
                     MC3635_fifo_mode_t fifo_mode,
           uint8_t fifo_thr);
  void SetINTCtrl(uint8_t fifo_thr_int_ctl,
          uint8_t fifo_full_int_ctl,
          uint8_t fifo_empty_int_ctl,
          uint8_t acq_int_ctl,
          uint8_t wake_int_ctl);
  void INTHandler(MC3635_interrupt_event_t *ptINT_Event);
  void SetWakeAGAIN(MC3635_gain_t gain);
  void SetSniffAGAIN(MC3635_gain_t gain);
  void SetSniffThreshold(MC3635_axis_t axis_cfg, uint8_t sniff_thr);
  void SetSniffDetectCount(MC3635_axis_t axis_cfg, uint8_t sniff_cnt);
  void SetSniffAndOrN(MC3635_andorn_t logicandor);
  void SetSniffDeltaMode(MC3635_delta_mode_t deltamode);
       MC3635_range_t GetRangeCtrl(void);
       MC3635_resolution_t GetResolutionCtrl(void);
       MC3635_cwake_sr_t GetCWakeSampleRate(void);
       MC3635_sniff_sr_t GetSniffSampleRate(void);
  bool IsFIFOEmpty(void);
       MC3635_acc_t readRawAccel(void);

 private:
    short x, y, z;
    // Raw Accelerometer data
    MC3635_acc_t AccRaw;
    I2Cdev* _i2c_bus;
};
#endif
