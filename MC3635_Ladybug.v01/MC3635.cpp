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
 * @file    MC3635.c
 * @author  mCube
 * @date    10 May 2018
 * @brief   Driver interface header file for accelerometer MC3635 series.
 * @see     http://www.mcubemems.com
 */

#include "MC3635.h"
#include "I2Cdev.h"

#define MC3635_CFG_I2C_ADDR        0x4C

#define MC3635_CFG_MODE_DEFAULT                 MC3635_MODE_STANDBY
#define MC3635_CFG_SAMPLE_RATE_CWAKE_DEFAULT    MC3635_CWAKE_SR_DEFAULT_54Hz
#define MC3635_CFG_SAMPLE_RATE_SNIFF_DEFAULT    MC3635_SNIFF_SR_DEFAULT_7Hz
#define MC3635_CFG_RANGE_DEFAULT                MC3635_RANGE_8G
#define MC3635_CFG_RESOLUTION_DEFAULT           MC3635_RESOLUTION_14BIT
#define MC3635_CFG_ORIENTATION_MAP_DEFAULT      ORIENTATION_TOP_RIGHT_UP

uint8_t CfgRange, CfgResolution, CfgFifo, CfgINT;

MC3635::MC3635(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}

//Initialize the MC3635 sensor and set as the default configuration
bool MC3635::start(void)
{
    //Init Reset
    reset();
    SetMode(MC3635_MODE_STANDBY);

    //SetWakeAGAIN
    SetWakeAGAIN(MC3635_GAIN_1X);
    //SetSniffAGAIN
    SetSniffAGAIN(MC3635_GAIN_1X);

    /* Check I2C connection */
    uint8_t id = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_PROD);
    if (id != 0x71)
    {
        /* No MC3635 detected ... return false */
        Serial.println("No MC3635 detected!");
        Serial.println(id, HEX);
        return false;
    }

    //Range: 8g
    SetRangeCtrl(MC3635_CFG_RANGE_DEFAULT);
    //Resolution: 14bit
    SetResolutionCtrl(MC3635_CFG_RESOLUTION_DEFAULT);
    //Sampling Rate: 50Hz by default
    SetCWakeSampleRate(MC3635_CFG_SAMPLE_RATE_CWAKE_DEFAULT);
  //Sampling Rate: 7Hz by default
    SetSniffSampleRate(MC3635_CFG_SAMPLE_RATE_SNIFF_DEFAULT);
    //Mode: Active
    SetMode(MC3635_MODE_CWAKE);

    delay(50);

    return true;
}

void MC3635::wake()
{
    //Set mode as wake
    SetMode(MC3635_MODE_CWAKE);
}

void MC3635::stop()
{
    //Set mode as Sleep
    SetMode(MC3635_MODE_STANDBY);
}

//Initial reset
void MC3635::reset()
{
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x10, 0x01);

    delay(10);

    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x24, 0x40);

    delay(50);

    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x09, 0x00);
    delay(10);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x0F, 0x42);
    delay(10);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x20, 0x01);
    delay(10);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x21, 0x80);
    delay(10);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x28, 0x00);
    delay(10);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x1a, 0x00);

    delay(50);

    uint8_t _bRegIO_C = 0;

    _bRegIO_C = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, 0x0D);

    _bRegIO_C &= 0x3F; // for I2C
    _bRegIO_C |= 0x40;

    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x0D, _bRegIO_C);

    delay(50);

    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x10, 0x01);

    delay(10);
}

void MC3635::sniff()
{
    //Set mode as Sleep
    SetMode(MC3635_MODE_SNIFF);
}

void MC3635::sniffreset()
{
    uint8_t value;
  
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C);
    value |= 0b10000000;
  
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C, value);
}

//Set the operation mode
void MC3635::SetMode(MC3635_mode_t mode)
{
    uint8_t value;
  uint8_t cfgfifovdd = 0x42;
  
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_MODE_C);
    value &= 0b11110000;
    value |= mode;
  
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_PWR_CONTROL, cfgfifovdd);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_MODE_C, value);
}

//Set the range control
void MC3635::SetRangeCtrl(MC3635_range_t range)
{
    uint8_t value;
    CfgRange = range;
    SetMode(MC3635_MODE_STANDBY);
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C);
    value &= 0b00000111;
    value |= (range << 4)&0x70 ;
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C, value);
}

//Set the resolution control
void MC3635::SetResolutionCtrl(MC3635_resolution_t resolution)
{
    uint8_t value;
    CfgResolution = resolution;
    SetMode(MC3635_MODE_STANDBY);
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C);
    value &= 0b01110000;
    value |= resolution;
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C, value);
}

//Set the sampling rate
void MC3635::SetCWakeSampleRate(MC3635_cwake_sr_t sample_rate)
{
    uint8_t value;
    SetMode(MC3635_MODE_STANDBY);
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_WAKE_C);
    value &= 0b00000000;
    value |= sample_rate;
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_WAKE_C, value);
}

//Set the sniff sampling rate
void MC3635::SetSniffSampleRate(MC3635_sniff_sr_t sniff_sr)
{
    uint8_t value;
    SetMode(MC3635_MODE_STANDBY);
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_C);
    value &= 0b00000000;
    value |= sniff_sr;
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_C, value);
}

//Set FIFO
void MC3635::SetFIFOCtrl(MC3635_fifo_ctl_t fifo_ctl,
                         MC3635_fifo_mode_t fifo_mode,
             uint8_t fifo_thr)
{
    if (fifo_thr > 31)  //maximum threshold
        fifo_thr = 31;
    
    SetMode(MC3635_MODE_STANDBY);
    
    CfgFifo = ((fifo_ctl << 6) | (fifo_mode << 5) | fifo_thr);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_FIFO_C, CfgFifo);
}

//Set interrupt control register
void MC3635::SetINTCtrl(uint8_t fifo_thr_int_ctl,
                        uint8_t fifo_full_int_ctl,
            uint8_t fifo_empty_int_ctl,
            uint8_t acq_int_ctl,
            uint8_t wake_int_ctl)
{
    
    SetMode(MC3635_MODE_STANDBY);
    
    CfgINT = (((fifo_thr_int_ctl & 0x01) << 6)
           | ((fifo_full_int_ctl & 0x01) << 5)
           | ((fifo_empty_int_ctl & 0x01) << 4)
           | ((acq_int_ctl & 0x01) << 3)
           | ((wake_int_ctl & 0x01) << 2)
           | MC3635_INTR_C_IAH_ACTIVE_HIGH//MC3635_INTR_C_IAH_ACTIVE_LOW//
           | MC3635_INTR_C_IPP_MODE_PUSH_PULL);//MC3635_INTR_C_IPP_MODE_OPEN_DRAIN);//
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_INTR_C, CfgINT);
}

//Interrupt handler (clear interrupt flag)
void MC3635::INTHandler(MC3635_interrupt_event_t *ptINT_Event)
{
    uint8_t value;

    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_STATUS_2);

    ptINT_Event->bWAKE           = ((value >> 2) & 0x01);
    ptINT_Event->bACQ            = ((value >> 3) & 0x01);
    ptINT_Event->bFIFO_EMPTY     = ((value >> 4) & 0x01);
    ptINT_Event->bFIFO_FULL      = ((value >> 5) & 0x01);
    ptINT_Event->bFIFO_THRESHOLD = ((value >> 6) & 0x01);
    ptINT_Event->bSWAKE_SNIFF    = ((value >> 7) & 0x01);
  
  value &= 0x03;
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_STATUS_2, value);
}

//Set CWake Analog Gain
void MC3635::SetWakeAGAIN(MC3635_gain_t gain)
{
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x20, 0x01);
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_GAIN, value);
}

//Set Sniff Analog Gain
void MC3635::SetSniffAGAIN(MC3635_gain_t gain)
{
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, 0x20, 0x00);
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_GAIN);
    value &= 0b00111111;
    value |= (gain << 6);
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_GAIN, value);
}

//Set Sniff threshold
void MC3635::SetSniffThreshold(MC3635_axis_t axis_cfg, uint8_t sniff_thr)
{
    uint8_t value;
  uint8_t regSniff_addr;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C);

    switch(axis_cfg)
    {
    case MC3635_AXIS_X:
        regSniff_addr = 0x01; //Put X-axis to active
        break;
    case MC3635_AXIS_Y: //Put Y-axis to active
        regSniff_addr = 0x02;
        break;
    case MC3635_AXIS_Z: //Put Z-axis to active
        regSniff_addr = 0x03;
        break;
    default:
        break;
    }
  
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C, regSniff_addr);
    value |= sniff_thr;
    _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C, value);
}

//Set Sniff detect counts, 1~62 events
void MC3635::SetSniffDetectCount(MC3635_axis_t axis_cfg, uint8_t sniff_cnt)
{
    uint8_t value;
  uint8_t sniff_cfg;
  uint8_t regSniff_addr;
  
    sniff_cfg = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C);
  
    switch(axis_cfg)
    {
    case MC3635_AXIS_X: //Select x detection count shadow register
        regSniff_addr = 0x05;
        break;
    case MC3635_AXIS_Y: //Select y detection count shadow register
        regSniff_addr = 0x06;
        break;
    case MC3635_AXIS_Z: //Select z detection count shadow register
        regSniff_addr = 0x07;
        break;
    default:
        break;
    }
  
  sniff_cfg |= regSniff_addr;
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C, sniff_cfg);
  
  value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C);
  
  value |= sniff_cnt;
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C, value);
  
  sniff_cfg |= 0x08;
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_CONF_C, sniff_cfg);
}

//Set sensor interrupt mode
void MC3635::SetSniffAndOrN(MC3635_andorn_t logicandor)
{
    uint8_t value;
  
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C);
  
    switch(logicandor)
    {
    case MC3635_ANDORN_OR:  //Axis or mode
        value &= 0xBF;
        break;
    case MC3635_ANDORN_AND: //Axis and mode
        value |= 0x40;
        break;
    default:
        break;
    }
  
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR,MC3635_REG_SNIFFTH_C, value);
}

//Set sensor sniff delta mode
void MC3635::SetSniffDeltaMode(MC3635_delta_mode_t deltamode)
{
    uint8_t value;
  
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C);
  
    switch(deltamode)
    {
    case MC3635_DELTA_MODE_C2P: //Axis C2P mode
        value &= 0x7F;
        break;
    case MC3635_DELTA_MODE_C2B: //Axis C2B mode
        value |= 0x80;
        break;
    default:
        break;
    }
  
  _i2c_bus->writeByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C, value);
  
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFFTH_C);
    Serial.println("SniffModeSet");
    Serial.println(value, HEX);
}

//Get the range control
MC3635_range_t MC3635::GetRangeCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C);
    Serial.println("GetRangeCtrl");
    Serial.println(value, HEX);
    value &= 0x70;
    return (MC3635_range_t) (value >> 4);
}

//Get the range control
MC3635_resolution_t MC3635::GetResolutionCtrl(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_RANGE_C);
    Serial.println("GetResolutionCtrl");
    Serial.println(value, HEX);
    value &= 0x07;
    return (MC3635_resolution_t) (value);
}

//Get the output sampling rate
MC3635_cwake_sr_t MC3635::GetCWakeSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_WAKE_C);
    Serial.println("GetCWakeSampleRate");
    Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC3635_cwake_sr_t) (value);
}

//Get the sniff sample rate
MC3635_sniff_sr_t MC3635::GetSniffSampleRate(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_SNIFF_C);
    Serial.println("GetSniffSampleRate");
    Serial.println(value, HEX);
    value &= 0b00001111;
    return (MC3635_sniff_sr_t) (value);
}

//Is FIFO empty
bool MC3635::IsFIFOEmpty(void)
{
    // Read the data format register to preserve bits
    uint8_t value;
    value = _i2c_bus->readByte(MC3635_CFG_I2C_ADDR, MC3635_REG_STATUS_1);
    value &= 0x10;
  Serial.println("FIFO_Status");
    Serial.println(value, HEX);
  
  if (value^0x10)
    return false; //Not empty
  else
    return true;  //Is empty
}

//Read the raw counts and SI units measurement data
MC3635_acc_t MC3635::readRawAccel(void)
{
    //{2g, 4g, 8g, 16g, 12g}
    float faRange[5] = { 19.614f, 39.228f, 78.456f, 156.912f, 117.684f};
    //{6bit, 7bit, 8bit, 10bit, 12bit, 14bit}
    float faResolution[6] = { 32.0f, 64.0f, 128.0f, 512.0f, 2048.0f, 8192.0f};

    byte rawData[6];
    // Read the six raw data registers into data array
    _i2c_bus->readBytes(MC3635_CFG_I2C_ADDR, MC3635_REG_XOUT_LSB, 6, rawData);
    x = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    y = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    z = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

    AccRaw.XAxis = (short) (x);
    AccRaw.YAxis = (short) (y);
    AccRaw.ZAxis = (short) (z);
    AccRaw.XAxis_g = (float) (x)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.YAxis_g = (float) (y)/faResolution[CfgResolution]*faRange[CfgRange];
    AccRaw.ZAxis_g = (float) (z)/faResolution[CfgResolution]*faRange[CfgRange];

    return AccRaw;
}
