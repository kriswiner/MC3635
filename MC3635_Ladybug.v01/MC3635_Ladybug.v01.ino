/*****************************************************************************
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
 * @file    MC3635_demo.ino
 * @author  mCube
 * @date    10 May 2018
 * @brief   Arduino example code for accelerometer MC3635 series.
 * @see     http://www.mcubemems.com
 * 
 ******************************************************************************
 * 
 * Modified by Kris Winer, Tlera Corporation 05/31/2021
 * 
 * Modified the mCube demo sketch to support the MC3635 with I2C
 * for wake-on-motion functionality alone.
 * Added proper use of hardware interrupt and separate I2CDev Wire library
 * but otherwise kept the structure of the original demo sketch.
 * 
 * Sketch is intended to be used with an STM32L432 (Ladybug) development board, but
 * almost any modern MCU that supports I2C and interrupts will work.
 * 
 * Unlimited distribution allowed with attribution, meaning copies of this
 * sketch must include the text above in the main header file as here.
 * 
 */

#include "MC3635.h"
#include "I2Cdev.h"

#define INTERRUPT_PIN                8
#define myLed                       13

#define I2C_BUS          Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

MC3635_interrupt_event_t evt_MC3635 = {0};
MC3635 MC3635_acc = MC3635(&i2c_0);         // Instantiate MC3635 sensor

volatile bool intFlag = false;

void setup()
{
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);  // start with led off since active HIGH
    
    pinMode(INTERRUPT_PIN, INPUT);
    
    Serial.begin(115200);
    Serial. blockOnOverrun(false);
    delay(2000);
    Serial.println("mCube Accelerometer MC3635:");

    I2C_BUS.begin();                                      // Set master mode, default on SDA/SCL for STM32L4
    delay(1000);
    I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
    delay(1000);
  
    Serial.println("Scan for I2C devices:");
    i2c_0.I2Cscan();                                      // should detect MC3635 at 0x4C
    delay(1000);
  
    MC3635_acc.start();
    checkRange();
    checkResolution();
    checkSamplingRate();
    checkSniffSamplingRate();
    Serial.println();
    digitalWrite(myLed, LOW); // turn off led when accel configured

    //Test read
    MC3635_acc_t rawAccel = MC3635_acc.readRawAccel();
    delay(10);
    Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
    Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
    Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
    Serial.println("counts");

    // Display the results (acceleration is measured in m/s^2)
    Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
    Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
    Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
    Serial.println("m/s^2");

    Serial.println("---------------------------------------------------------");

    attachInterrupt(INTERRUPT_PIN, myIntHandler, RISING);
    MC3635_acc.INTHandler(&evt_MC3635);  // clear interrupt 
    sensorsniff();

} // end of setup


void loop()
{
       if (intFlag == true) {
           intFlag = false;
           
            // Read the raw sensor data count
            MC3635_acc_t rawAccel = MC3635_acc.readRawAccel();
            
            Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
            Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
            Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
            Serial.println("counts");
        
            // Display the results (acceleration is measured in m/s^2)
            Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
            Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
            Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
            Serial.println("m/s^2");
        
            Serial.println("---------------------------------------------------------");

            rawAccel = MC3635_acc.readRawAccel();
            
            Serial.print("X:\t"); Serial.print(rawAccel.XAxis); Serial.print("\t");
            Serial.print("Y:\t"); Serial.print(rawAccel.YAxis); Serial.print("\t");
            Serial.print("Z:\t"); Serial.print(rawAccel.ZAxis); Serial.print("\t");
            Serial.println("counts");
        
            // Display the results (acceleration is measured in m/s^2)
            Serial.print("X: \t"); Serial.print(rawAccel.XAxis_g); Serial.print("\t");
            Serial.print("Y: \t"); Serial.print(rawAccel.YAxis_g); Serial.print("\t");
            Serial.print("Z: \t"); Serial.print(rawAccel.ZAxis_g); Serial.print("\t");
            Serial.println("m/s^2");
        
            Serial.println("---------------------------------------------------------");
            
            digitalWrite(myLed, HIGH); delay(10); digitalWrite(myLed, LOW); 
                     
        MC3635_acc.INTHandler(&evt_MC3635);  // clear interrupt 
        sensorsniff();
        }

       STM32.stop(); // wait for an interrupt
} // end of main loop


// Useful functions

void myIntHandler()
{
  intFlag = true;
}


void sensorsniff()
{
    //Sensor sniff
    MC3635_acc.stop();
    MC3635_acc.SetSniffThreshold(MC3635_AXIS_X,3);
    MC3635_acc.SetSniffThreshold(MC3635_AXIS_Y,3);
    MC3635_acc.SetSniffThreshold(MC3635_AXIS_Z,3);
    MC3635_acc.SetSniffDetectCount(MC3635_AXIS_X,3);
    MC3635_acc.SetSniffDetectCount(MC3635_AXIS_Y,3);
    MC3635_acc.SetSniffDetectCount(MC3635_AXIS_Z,3);
    MC3635_acc.SetSniffAndOrN(MC3635_ANDORN_OR);
    MC3635_acc.SetSniffDeltaMode(MC3635_DELTA_MODE_C2B);
    MC3635_acc.SetINTCtrl(0,0,0,0,1); //Enable wake-up INT
    MC3635_acc.sniff();
    Serial.println("Sensor sniff.");
}

void checkRange()
{
    switch(MC3635_acc.GetRangeCtrl())
    {
    case MC3635_RANGE_16G:
        Serial.println("Range: +/- 16 g");
        break;
    case MC3635_RANGE_12G:
        Serial.println("Range: +/- 12 g");
        break;
    case MC3635_RANGE_8G:
        Serial.println("Range: +/- 8 g");
        break;
    case MC3635_RANGE_4G:
        Serial.println("Range: +/- 4 g");
        break;
    case MC3635_RANGE_2G:
        Serial.println("Range: +/- 2 g");
        break;
    default:
        Serial.println("Range: +/- 8 g");
        break;
    }
}

void checkResolution()
{
    switch(MC3635_acc.GetResolutionCtrl())
    {
    case MC3635_RESOLUTION_6BIT:
        Serial.println("Resolution: 6bit");
        break;
    case MC3635_RESOLUTION_7BIT:
        Serial.println("Resolution: 7bit");
        break;
    case MC3635_RESOLUTION_8BIT:
        Serial.println("Resolution: 8bit");
        break;
    case MC3635_RESOLUTION_10BIT:
        Serial.println("Resolution: 10bit");
        break;
    case MC3635_RESOLUTION_14BIT:
        Serial.println("Resolution: 14bit");
        break;
    case MC3635_RESOLUTION_12BIT:
        Serial.println("Resolution: 12bit");
        break;
    default:
        Serial.println("Resolution: 14bit");
        break;
    }
}

void checkSamplingRate()
{
    Serial.println("Low Power Mode SR");
    switch(MC3635_acc.GetCWakeSampleRate())
    {
    case MC3635_CWAKE_SR_DEFAULT_54Hz:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    case MC3635_CWAKE_SR_14Hz:
        Serial.println("Output Sampling Rate: 14 Hz");
        break;
    case MC3635_CWAKE_SR_28Hz:
        Serial.println("Output Sampling Rate: 28 Hz");
        break;
    case MC3635_CWAKE_SR_54Hz:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    case MC3635_CWAKE_SR_105Hz:
        Serial.println("Output Sampling Rate: 105 Hz");
        break;
    case MC3635_CWAKE_SR_210Hz:
        Serial.println("Output Sampling Rate: 210 Hz");
        break;
    case MC3635_CWAKE_SR_400Hz:
        Serial.println("Output Sampling Rate: 400 Hz");
        break;
    case MC3635_CWAKE_SR_600Hz:
        Serial.println("Output Sampling Rate: 600 Hz");
        break;
    default:
        Serial.println("Output Sampling Rate: 54 Hz");
        break;
    }
}

void checkSniffSamplingRate()
{
    Serial.println("Sniff Mode SR");
    switch(MC3635_acc.GetSniffSampleRate())
    {
    case MC3635_SNIFF_SR_DEFAULT_7Hz:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    case MC3635_SNIFF_SR_0p4Hz:
        Serial.println("Sniff Sampling Rate: 0.4 Hz");
        break;
    case MC3635_SNIFF_SR_0p8Hz:
        Serial.println("Sniff Sampling Rate: 0.8 Hz");
        break;
    case MC3635_SNIFF_SR_1p5Hz:
        Serial.println("Sniff Sampling Rate: 1.5 Hz");
        break;
    case MC3635_SNIFF_SR_7Hz:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    case MC3635_SNIFF_SR_14Hz:
        Serial.println("Sniff Sampling Rate: 14 Hz");
        break;
    case MC3635_SNIFF_SR_28Hz:
        Serial.println("Sniff Sampling Rate: 28 Hz");
        break;
    case MC3635_SNIFF_SR_54Hz:
        Serial.println("Sniff Sampling Rate: 54 Hz");
        break;
    case MC3635_SNIFF_SR_105Hz:
        Serial.println("Sniff Sampling Rate: 105 Hz");
        break;
    case MC3635_SNIFF_SR_210Hz:
        Serial.println("Sniff Sampling Rate: 210 Hz");
        break;
    case MC3635_SNIFF_SR_400Hz:
        Serial.println("Sniff Sampling Rate: 400 Hz");
        break;
    case MC3635_SNIFF_SR_600Hz:
        Serial.println("Sniff Sampling Rate: 600 Hz");
        break;
    default:
        Serial.println("Sniff Sampling Rate: 7 Hz");
        break;
    }
}
