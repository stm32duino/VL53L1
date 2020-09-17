/**
 ******************************************************************************
 * @file    VL53L1_Sat_HelloWorld_Interrupt.ino
 * @author  SRA
 * @version V1.0.0
 * @date    30 July 2020
 * @brief   Arduino test application for the STMicrolectronics VL53L1
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


//On some boards like the Arduino Uno the pin used by the sensor to raise interrupts (A2)
//can't be mapped as an interrupt pin. For this this reason this sketch will not work
//unless some additional cabling is done and the interrupt pin is changed.

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53l1_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#define DEV_I2C Wire
#define SerialPort Serial

#define interruptPin A2

// Components.
VL53L1 *sensor_vl53l1_sat;

volatile int interruptCount=0;

void measure()
{
   interruptCount=1;
}

void setup()
{
   VL53L1_Error status;
   // Led.
   pinMode(13, OUTPUT);
   pinMode(interruptPin, INPUT_PULLUP);
   attachInterrupt(interruptPin, measure, FALLING);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin();

   // Create VL53L1 satellite component.
   sensor_vl53l1_sat = new VL53L1(&DEV_I2C, A1, A2);

   // Switch off VL53L1 satellite component.
   sensor_vl53l1_sat->VL53L1_Off();

   // Initialize VL53L1 satellite component.
   status = sensor_vl53l1_sat->InitSensor(0x12);
   if(status)
   {
      SerialPort.println("Init sensor_vl53l1_sat failed...");
   }

   sensor_vl53l1_sat->VL53L1_SetPresetMode(VL53L1_PRESETMODE_RANGING);

   sensor_vl53l1_sat->VL53L1_StartMeasurement();
}

void loop()
{
   VL53L1_MultiRangingData_t MultiRangingData;
   VL53L1_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   char report[64];
   if (interruptCount)
   {
      int status;

      interruptCount=0;
      // Led blinking.
      digitalWrite(13, HIGH);

      status = sensor_vl53l1_sat->VL53L1_GetMeasurementDataReady(&NewDataReady);
      if((!status)&&(NewDataReady!=0))
      {
         status = sensor_vl53l1_sat->VL53L1_GetMultiRangingData(pMultiRangingData);
         no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
         snprintf(report, sizeof(report), "Count=%d, #Objs=%1d ", pMultiRangingData->StreamCount, no_of_object_found);
         SerialPort.print(report);
         for(j=0;j<no_of_object_found;j++)
         {
            if(j!=0)SerialPort.print("\r\n                   ");
            SerialPort.print("status=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeStatus);
            SerialPort.print(", D=");
            SerialPort.print(pMultiRangingData->RangeData[j].RangeMilliMeter);
            SerialPort.print("mm");
            SerialPort.print(", Signal=");
            SerialPort.print((float)pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0);
            SerialPort.print(" Mcps, Ambient=");
            SerialPort.print((float)pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
            SerialPort.print(" Mcps");
         }
         SerialPort.println("");
         if (status==0)
         {
            status = sensor_vl53l1_sat->VL53L1_ClearInterruptAndStartMeasurement();
         }
      }

      digitalWrite(13, LOW);
   }
}