/*******************************************************************************
 Copyright Ã‚Â© 2020, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __VL53L1_CLASS_H
#define __VL53L1_CLASS_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "Wire.h"
#include "RangeSensor.h"
#include "vl53l1_def.h"
#include "vl53l1_dmax_private_structs.h"
#include "vl53l1_dmax_structs.h"
#include "vl53l1_error_codes.h"
#include "vl53l1_error_exceptions.h"
#include "vl53l1_hist_map.h"
#include "vl53l1_hist_private_structs.h"
#include "vl53l1_hist_structs.h"
#include "vl53l1_ll_def.h"
#include "vl53l1_ll_device.h"
#include "vl53l1_nvm_map.h"
#include "vl53l1_nvm_structs.h"
#include "vl53l1_platform_user_config.h"
#include "vl53l1_platform_user_defines.h"
#include "vl53l1_preset_setup.h"
#include "vl53l1_register_map.h"
#include "vl53l1_register_settings.h"
#include "vl53l1_register_structs.h"
#include "vl53l1_tuning_parm_defaults.h"
#include "vl53l1_types.h"
#include "vl53l1_xtalk_private_structs.h"

/** @brief  Contains the current state and internal values of the API
 */

typedef struct {
  VL53L1_DevData_t   Data;
  /*!< Low Level Driver data structure */
  uint8_t   i2c_slave_address;
  uint8_t   comms_type;
  uint16_t  comms_speed_khz;
  TwoWire *I2cHandle;
  uint8_t   I2cDevAddr;
  int     Present;
  int   Enabled;
  int LoopState;
  int FirstStreamCountZero;
  int   Idle;
  int   Ready;
  uint8_t RangeStatus;
  FixPoint1616_t SignalRateRtnMegaCps;
  VL53L1_DeviceState   device_state;  /*!< Device State */
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

#define VL53L1DevDataGet(Dev, field) (Dev->Data.field)
#define VL53L1DevDataSet(Dev, field, VL53L1_PRM_00005) (Dev->Data.field)=(VL53L1_PRM_00005)
#define VL53L1DevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)
#define VL53L1DevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)

#define VL53L1_DEFAULT_DEVICE_ADDRESS 0x52

#define VL53L1_NVM_POWER_UP_DELAY_US             50
#define VL53L1_NVM_READ_TRIGGER_DELAY_US          5

#define VL53L1_D_002    0xFFFF
#define VL53L1_D_008  0xFFFF
#define VL53L1_D_003  0xFFFFFF
#define VL53L1_D_007  0xFFFFFFFF
#define VL53L1_D_005  0x7FFFFFFFFF
#define VL53L1_D_009  0xFFFFFFFFFF
#define VL53L1_D_010  0xFFFFFFFFFFFF
#define VL53L1_D_004  0xFFFFFFFFFFFFFF
#define VL53L1_D_006  0x7FFFFFFFFFFFFFFF
#define VL53L1_D_011  0xFFFFFFFFFFFFFFFF

#ifndef VL53L1_USE_EMPTY_STRING
#define  VL53L1_STRING_DEVICE_INFO_NAME0          "VL53L1 cut1.0"
#define  VL53L1_STRING_DEVICE_INFO_NAME1          "VL53L1 cut1.1"
#define  VL53L1_STRING_DEVICE_INFO_TYPE          "VL53L1"

/* Range Status */
#define  VL53L1_STRING_RANGESTATUS_NONE                 "No Update"
#define  VL53L1_STRING_RANGESTATUS_RANGEVALID           "Range Valid"
#define  VL53L1_STRING_RANGESTATUS_SIGMA                "Sigma Fail"
#define  VL53L1_STRING_RANGESTATUS_SIGNAL               "Signal Fail"
#define  VL53L1_STRING_RANGESTATUS_MINRANGE             "Min Range Fail"
#define  VL53L1_STRING_RANGESTATUS_PHASE                "Phase Fail"
#define  VL53L1_STRING_RANGESTATUS_HW                   "Hardware Fail"


/* Range Status */
#define  VL53L1_STRING_STATE_POWERDOWN               "POWERDOWN State"
#define  VL53L1_STRING_STATE_WAIT_STATICINIT \
     "Wait for staticinit State"
#define  VL53L1_STRING_STATE_STANDBY                 "STANDBY State"
#define  VL53L1_STRING_STATE_IDLE                    "IDLE State"
#define  VL53L1_STRING_STATE_RUNNING                 "RUNNING State"
#define  VL53L1_STRING_STATE_RESET                   "RESET State"
#define  VL53L1_STRING_STATE_UNKNOWN                 "UNKNOWN State"
#define  VL53L1_STRING_STATE_ERROR                   "ERROR State"



/* Check Enable */
#define  VL53L1_STRING_CHECKENABLE_SIGMA_FINAL_RANGE \
     "SIGMA FINAL RANGE"
#define  VL53L1_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE \
     "SIGNAL RATE FINAL RANGE"
#define  VL53L1_STRING_CHECKENABLE_SIGNAL_MIN_CLIP \
     "SIGNAL MIN CLIP"
#define  VL53L1_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD \
     "RANGE IGNORE THRESHOLD"
#define  VL53L1_STRING_CHECKENABLE_RANGE_PHASE_HIGH \
     "RANGE PHASE HIGH"
#define  VL53L1_STRING_CHECKENABLE_RANGE_PHASE_LOW \
     "RANGE PHASE LOW"
#define  VL53L1_STRING_CHECKENABLE_RANGE_PHASE_CONSISTENCY \
     "RANGE PHASE CONSISTENCY"

/* Sequence Step */
#define  VL53L1_STRING_SEQUENCESTEP_VHV         "VHV"
#define  VL53L1_STRING_SEQUENCESTEP_PHASECAL    "PHASE CAL"
#define  VL53L1_STRING_SEQUENCESTEP_REFPHASE    "REF PHASE"
#define  VL53L1_STRING_SEQUENCESTEP_DSS1        "DSS1"
#define  VL53L1_STRING_SEQUENCESTEP_DSS2        "DSS2"
#define  VL53L1_STRING_SEQUENCESTEP_MM1         "MM1"
#define  VL53L1_STRING_SEQUENCESTEP_MM2         "MM2"
#define  VL53L1_STRING_SEQUENCESTEP_RANGE       "RANGE"

#define  VL53L1_STRING_ERROR_NONE \
     "No Error"
#define  VL53L1_STRING_ERROR_CALIBRATION_WARNING \
     "Calibration Warning Error"
#define  VL53L1_STRING_ERROR_MIN_CLIPPED \
     "Min clipped error"
#define  VL53L1_STRING_ERROR_UNDEFINED \
     "Undefined error"
#define  VL53L1_STRING_ERROR_INVALID_PARAMS \
     "Invalid parameters error"
#define  VL53L1_STRING_ERROR_NOT_SUPPORTED \
     "Not supported error"
#define  VL53L1_STRING_ERROR_RANGE_ERROR \
     "Range error"
#define  VL53L1_STRING_ERROR_TIME_OUT \
     "Time out error"
#define  VL53L1_STRING_ERROR_MODE_NOT_SUPPORTED \
     "Mode not supported error"
#define  VL53L1_STRING_ERROR_BUFFER_TOO_SMALL \
     "Buffer too small"
#define  VL53L1_STRING_ERROR_COMMS_BUFFER_TOO_SMALL \
     "Comms Buffer too small"
#define  VL53L1_STRING_ERROR_GPIO_NOT_EXISTING \
     "GPIO not existing"
#define  VL53L1_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED \
     "GPIO funct not supported"
#define  VL53L1_STRING_ERROR_CONTROL_INTERFACE \
     "Control Interface Error"
#define  VL53L1_STRING_ERROR_INVALID_COMMAND \
     "Invalid Command Error"
#define  VL53L1_STRING_ERROR_DIVISION_BY_ZERO \
     "Division by zero Error"
#define  VL53L1_STRING_ERROR_REF_SPAD_INIT \
     "Reference Spad Init Error"
#define  VL53L1_STRING_ERROR_GPH_SYNC_CHECK_FAIL \
     "GPH Sync Check Fail - API out of sync"
#define  VL53L1_STRING_ERROR_STREAM_COUNT_CHECK_FAIL \
     "Stream Count Check Fail - API out of sync"
#define  VL53L1_STRING_ERROR_GPH_ID_CHECK_FAIL \
     "GPH ID Check Fail - API out of sync"
#define  VL53L1_STRING_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL \
     "Zone Stream Count Check Fail - API out of sync"
#define  VL53L1_STRING_ERROR_ZONE_GPH_ID_CHECK_FAIL \
     "Zone GPH ID Check Fail - API out of sync"

#define  VL53L1_STRING_ERROR_XTALK_EXTRACTION_NO_SAMPLES_FAIL \
     "No Xtalk using full array - Xtalk Extract Fail"
#define  VL53L1_STRING_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL \
     "Xtalk does not meet required VL53L1_p_011 limit - Xtalk Extract Fail"

#define  VL53L1_STRING_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL \
   "Offset Cal - one of more stages with no valid samples - fatal"
#define  VL53L1_STRING_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL \
   "Offset Cal - one of more stages with no SPADS enables - fatal"
#define  VL53L1_STRING_ERROR_ZONE_CAL_NO_SAMPLE_FAIL \
   "Zone Cal - one of more zones with no valid samples - fatal"

#define  VL53L1_STRING_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS \
   "Ref SPAD Char - Not Enough Good SPADs"
#define  VL53L1_STRING_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH \
   "Ref SPAD Char - Final Ref Rate too high"
#define  VL53L1_STRING_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW \
   "Ref SPAD Char - Final Ref Rate too low"

#define  VL53L1_STRING_WARNING_OFFSET_CAL_MISSING_SAMPLES \
   "Offset Cal - Less than the requested number of valid samples"
#define  VL53L1_STRING_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH \
   "Offset Cal - Sigma estimate value too high - offset not stable"
#define  VL53L1_STRING_WARNING_OFFSET_CAL_RATE_TOO_HIGH \
   "Offset Cal - Rate too high - in pile up"
#define  VL53L1_STRING_WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW \
   "Offset Cal - Insufficient SPADs - offset may not be stable"

#define  VL53L1_STRING_WARNING_ZONE_CAL_MISSING_SAMPLES \
   "Zone Cal - One or more zone with less than requested valid samples"
#define  VL53L1_STRING_WARNING_ZONE_CAL_SIGMA_TOO_HIGH \
   "Zone Cal - One of more zones the VL53L1_p_011 estimate too high"
#define  VL53L1_STRING_WARNING_ZONE_CAL_RATE_TOO_HIGH \
   "Zone Cal - One of more zones with rate too high - in pile up"

#define  VL53L1_STRING_WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT \
     "Xtalk - Gradient sample num = 0"
#define  VL53L1_STRING_WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT \
     "Xtalk - Gradient Sigma > Limit"
#define  VL53L1_STRING_WARNING_XTALK_MISSING_SAMPLES \
     "Xtalk - Some missing and invalid samples"

#define  VL53L1_STRING_ERROR_DEVICE_FIRMWARE_TOO_OLD \
     "Device Firmware too old"
#define  VL53L1_STRING_ERROR_DEVICE_FIRMWARE_TOO_NEW \
     "Device Firmware too new"
#define  VL53L1_STRING_ERROR_UNIT_TEST_FAIL \
     "Unit Test Fail"
#define  VL53L1_STRING_ERROR_FILE_READ_FAIL \
     "File Read Fail"
#define  VL53L1_STRING_ERROR_FILE_WRITE_FAIL \
     "File Write Fail"

#define  VL53L1_STRING_ERROR_NOT_IMPLEMENTED \
     "Not implemented error"
#define  VL53L1_STRING_UNKNOW_ERROR_CODE \
     "Unknown Error Code"
#endif /* VL53L1_USE_EMPTY_STRING */


/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L1 sensor component
 */
class VL53L1Base : public RangeSensor {
  public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     */
    VL53L1Base(TwoWire *i2c) : RangeSensor(), dev_i2c(i2c)
    {
      Dev = &MyDevice;
      memset((void *)Dev, 0x0, sizeof(VL53L1_Dev_t));
      MyDevice.I2cDevAddr = VL53L1_DEFAULT_DEVICE_ADDRESS;
      MyDevice.I2cHandle = i2c;
    }

    /** Destructor
     */
    virtual ~VL53L1Base() {}
    /* warning: VL53L1 class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

    virtual int begin()
    {
      VL53L1_XshutInitialize();
      return 0;
    }

    virtual int end()
    {
      VL53L1_XshutDeinitialize();
      return 0;
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    /* turns on the sensor */
    virtual void VL53L1_On(void)
    {
      VL53L1_XshutSetHigh();
      delay(10);
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    /* turns off the sensor */
    virtual void VL53L1_Off(void)
    {
      VL53L1_XshutSetLow();
      delay(10);
    }

    /**
     * @brief       Initialize the sensor with default values
     * @return      0 on Success
     */

    VL53L1_Error InitSensor(uint8_t address)
    {
      VL53L1_Error status = VL53L1_ERROR_NONE;
      VL53L1_Off();
      VL53L1_On();
      status = VL53L1_SetDeviceAddress(address);

#ifdef DEBUG_MODE
      uint8_t byteData;
      uint16_t wordData;
      status = VL53L1_RdByte(Dev, VL53L1_IDENTIFICATION__MODEL_ID, &byteData);
      Serial.println("VL53L1 Model_ID: " + String(byteData));
      status = VL53L1_RdByte(Dev, VL53L1_IDENTIFICATION__MODULE_TYPE, &byteData);
      Serial.println("VL53L1 Module_Type: " + String(byteData));
      status = VL53L1_RdWord(Dev, VL53L1_IDENTIFICATION__MODULE_ID, &wordData);
      Serial.println("VL53L1 Module_ID: " + String(wordData));
#endif

      if (status == VL53L1_ERROR_NONE) {
        status = VL53L1_WaitDeviceBooted();
      }

      if (status == VL53L1_ERROR_NONE) {
        status = VL53L1_DataInit();
      }

      if (status == VL53L1_ERROR_NONE) {
        status = VL53L1_StaticInit();
      }

      return status;
    }



    /**
     *
     * @brief One time device initialization
     * @param void
     * @return     0 on success
     */
    virtual int Init()
    {
      return VL53L1_DataInit();
    }



    /* Read function of the ID device */
    virtual int ReadID()
    {
      uint64_t Uid;
      VL53L1_GetUID(&Uid);
      if (Uid == 0x00FF000000FF) {
        return 0;
      }
      return -1;
    }



    /**
     * @brief Get ranging result and only that
     * @param pRange_mm  Pointer to range distance
     * @return           0 on success
     */
    virtual int GetDistance(uint32_t *piData)
    {
      uint8_t NewDataReady = 0;
      VL53L1_RangingMeasurementData_t RangingData;
      int status;
      int ret_val = 0;

      do {
        status = VL53L1_GetMeasurementDataReady(&NewDataReady);

        if (status) {
          return status;
        }
      } while (!NewDataReady);


      status = VL53L1_GetRangingMeasurementData(&RangingData);

      if (status) {
        return status;
      }

      if (RangingData.RangeStatus == 0) {
        *piData = RangingData.RangeMilliMeter;
      } else {
        *piData = 0;
        ret_val = VL53L1_ERROR_RANGE_ERROR;
      }

      status = VL53L1_ClearInterruptAndStartMeasurement();

      if (status) {
        return status;
      }

      return ret_val;
    }

    /* VL53L1_api.h functions */

    /**
     * @brief Return the VL53L1 driver Version
     *
     * @note This function doesn't access to the device
     *
     * @param   pVersion              pointer to current driver Version
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetVersion(VL53L1_Version_t *pVersion);

    /**
     * @brief Reads the Product Revision for a for given Device
     * This function can be used to distinguish cut1.0 from cut1.1.
     *
     * @param   pProductRevisionMajor  Pointer to Product Revision Major for a given Device
     * @param   pProductRevisionMinor  Pointer to Product Revision Minor for a given Device
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetProductRevision(uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor);

    /**
     * @brief Reads the Device information for given Device
     *
     * @note This function Access to the device
     *
     * @param   pVL53L1_DeviceInfo  Pointer to current device info for a given Device
     * @return  VL53L1_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetDeviceInfo(VL53L1_DeviceInfo_t *pVL53L1_DeviceInfo);

    /**
     * @brief Reads the Device unique identifier
     *
     * @note This function Access to the device
     *
     * @param   pUid                Pointer to current device unique ID
     * @return  VL53L1_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetUID(uint64_t *pUid);

    /**
     * @brief Human readable Range Status string for a given RangeStatus
     *
     * @note This function doesn't access to the device
     *
     * @param   RangeStatus         The RangeStatus code as stored on VL53L1_RangingMeasurementData_t
     * @param   pRangeStatusString  The returned RangeStatus string. Shall be
     * defined as char buf[VL53L1_MAX_STRING_LENGTH]
     * @return  VL53L1_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetRangeStatusString(uint8_t RangeStatus, char *pRangeStatusString);

    /**
     * @brief Human readable error string for driver error status
     *
     * @note This function doesn't access to the device
     *
     * @param   PalErrorCode       The error code as stored on @a VL53L1_Error
     * @param   pPalErrorString    The error string corresponding to the
     * PalErrorCode. Shall be defined as char buf[VL53L1_MAX_STRING_LENGTH]
     * @return  VL53L1_ERROR_NONE  Success
     * @return  "Other error code" See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetPalErrorString(VL53L1_Error PalErrorCode, char *pPalErrorString);

    /**
     * @brief Human readable driver State string
     *
     * @note This function doesn't access to the device
     *
     * @param   PalStateCode          The State code as stored on @a VL53L1_State
     * @param   pPalStateString       The State string corresponding to the
     * PalStateCode. Shall be defined as char buf[VL53L1_MAX_STRING_LENGTH]
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetPalStateString(VL53L1_State PalStateCode, char *pPalStateString);

    /**
     * @brief Reads the internal state of the driver for a given Device
     *
     * @note This function doesn't access to the device
     *
     * @param   pPalState             Pointer to current state of the PAL for a given Device
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetPalState(VL53L1_State *pPalState);

    /**
     * @brief Set new device address
     *
     * After completion the device will answer to the new address programmed.
     * This function should be called when several devices are used in parallel
     * before start programming the sensor.
     * When a single device us used, there is no need to call this function.
     *
     * When it is requested for multi devices system this function MUST be called
     * prior to VL53L1_DataInit()
     *
     * @note This function Access to the device
     *
     * @param   DeviceAddress         The new Device address
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetDeviceAddress(uint8_t DeviceAddress);

    /**
     *
     * @brief One time device initialization
     *
     * To be called after device has been powered on and booted
     * see @a VL53L1_WaitDeviceBooted()
     *
     * @par Function Description
     * When not used after a fresh device "power up", it may return
     * @a #VL53L1_ERROR_CALIBRATION_WARNING meaning wrong calibration data
     * may have been fetched from device that can result in ranging offset error\n
     * If VL53L1_DataInit is called several times then the application must restore
     * calibration calling @a VL53L1_SetOffsetCalibrationData()
     * It implies application has gathered calibration data thanks to
     * @a VL53L1_GetOffsetCalibrationData() after an initial calibration stage.
     * This function will change the VL53L1_State from VL53L1_STATE_POWERDOWN to
     * VL53L1_STATE_WAIT_STATICINIT.
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_DataInit();

    /**
     * @brief Do basic device init (and eventually patch loading)
     * This function will change the VL53L1_State from
     * VL53L1_STATE_WAIT_STATICINIT to VL53L1_STATE_IDLE.
     * In this stage all default setting will be applied.
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_StaticInit();

    /**
     * @brief Wait for device booted after chip enable (hardware standby)
     * This function can be run only when VL53L1_State is VL53L1_STATE_POWERDOWN.
     *
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     *
     */
    VL53L1_Error VL53L1_WaitDeviceBooted();

    /**
     * @brief  Set a new Preset Mode
     * @par Function Description
     * Set device to a new Operating Mode (High speed ranging, Multi objects ...)
     *
     * @note This function doesn't Access to the device
     *
     * @warning This function change the timing budget to 16 ms and the inter-
     * measurement period to 1000 ms. Also the VL53L1_DISTANCEMODE_LONG is used.
     *
     * @param   PresetMode            New Preset mode to apply
     * <br>Valid values are:
     */
    /**
     * @li VL53L1_PRESETMODE_MULTIZONES_SCANNING
     * @li VL53L1_PRESETMODE_RANGING
     * @li VL53L1_PRESETMODE_AUTONOMOUS
     * @li VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS
     * @li VL53L1_PRESETMODE_LITE_RANGING
     * @li VL53L1_PRESETMODE_OLT
     */
    /**
     *
     * @return  VL53L1_ERROR_NONE               Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED This error occurs when PresetMode is
     *                                          not in the supported list
     */
    VL53L1_Error VL53L1_SetPresetMode(VL53L1_PresetModes PresetMode);

    /**
     * @brief  Get current Preset Mode
     * @par Function Description
     * Get actual mode of the device(ranging, histogram ...)
     *
     * @note This function doesn't Access to the device
     *
     * @param   pPresetMode           Pointer to current apply mode value
     *
     * @return  VL53L1_ERROR_NONE                   Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED     This error occurs when
     * DeviceMode is not in the supported list
     */
    VL53L1_Error VL53L1_GetPresetMode(VL53L1_PresetModes *pPresetMode);

    /**
     * @brief  Set the distance mode
     * @par Function Description
     * Set the distance mode to be used for the next ranging.<br>
     * The modes Short, Medium and Long are used to optimize the ranging accuracy
     * in a specific range of distance.<br> The user select one of these modes to
     * select the distance range. <br>
     * Two additional modes are supported: AUTO and AUTO_LITE the difference between
     * these modes is the following.<br>
     * The mode AUTO take into account both the ranging distance (RangeMilliMeter)
     * and the dmax distance (DmaxMilliMeter).<br> The algorithm uses the ranging
     * distance when the range status is ok and uses the dmax distance when the
     * range status is not ok.<br>
     * The AUTO_LITE take into account only the ranging distance, so nothing is done
     * in case of range error i.e. the distance mode will not be changed.
     * @note This function doesn't Access to the device
     *
     * @warning This function should be called after @a VL53L1_SetPresetMode().
     *
     * @param   DistanceMode          Distance mode to apply valid values are:
     * @li VL53L1_DISTANCEMODE_SHORT
     * @li VL53L1_DISTANCEMODE_MEDIUM
     * @li VL53L1_DISTANCEMODE_LONG
     * @li VL53L1_DISTANCEMODE_AUTO_LITE
     * @li VL53L1_DISTANCEMODE_AUTO
     * @return  VL53L1_ERROR_NONE               Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED This error occurs when DistanceMode
     *                                          is not in the supported list
     * @return  "Other error code"              See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetDistanceMode(VL53L1_DistanceModes DistanceMode);

    /**
     * @brief  Get the distance mode
     * @par Function Description
     * Get the distance mode used for the next ranging.
     *
     * @param   Dev                   Device Handle
     * @param   *pDistanceMode        Pointer to Distance mode
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetDistanceMode(VL53L1_DistanceModes *pDistanceMode);

    /**
     * @brief  Set the output mode
     * @par Function Description
     * Set the output mode to be used for the next ranging. The output mode is used
     * to select, in case of multiple objects, which one will be used in
     * function @a VL53L1_GetRangingMeasurementData().
     * VL53L1_SetOutputMode also sets the object used by automatic
     * distance mode algorithm when @a VL53L1_SetDistanceMode() is
     * set to automatic mode.
     *
     * @note This function doesn't Access to the device
     *
     * @warning This function should be called after @a VL53L1_SetPresetMode().
     *
     * @param   OutputMode            Output mode to apply valid values are:
     * @li VL53L1_OUTPUTMODE_NEAREST
     * @li VL53L1_OUTPUTMODE_STRONGEST
     *
     * @return  VL53L1_ERROR_NONE               Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED This error occurs when OutputMode
     *                                          is not in the supported list
     * @return  "Other error code"              See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetOutputMode(VL53L1_OutputModes OutputMode);

    /**
     * @brief  Get the output mode
     * @par Function Description
     * Get the output mode used for the next ranging.
     *
     * @param   *pOutputMode          Pointer to Output mode
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetOutputMode(VL53L1_OutputModes *pOutputMode);

    /**
     * @brief Set Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Defines the maximum time allowed by the user to the device to run a
     * full ranging sequence for the current mode (ranging, histogram, ASL ...)
     *
     * @param   Dev                                Device Handle
     * @param MeasurementTimingBudgetMicroSeconds  Max measurement time in
     * microseconds.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  VL53L1_ERROR_INVALID_PARAMS  Error timing parameter not
     *                                       supported.
     *                                       The maximum accepted value for the
     *                                       computed timing budget is 10 seconds
     *                                       the minimum value depends on the preset
     *                                       mode selected.
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds);

    /**
     * @brief Get Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Returns the programmed the maximum time allowed by the user to the
     * device to run a full ranging sequence for the current mode
     * (ranging, histogram, ASL ...)
     *
     * @param   pMeasurementTimingBudgetMicroSeconds   Max measurement time in
     * microseconds.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetMeasurementTimingBudgetMicroSeconds(uint32_t *pMeasurementTimingBudgetMicroSeconds);

    /**
     * Program continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     * When trying to set too short time return  INVALID_PARAMS minimal value
     *
     * @param   InterMeasurementPeriodMilliSeconds   Inter-Measurement Period in ms.
     *  this value should be greater than the duration set in
     *  @a VL53L1_SetMeasurementTimingBudgetMicroSeconds() to ensure smooth ranging
     *  operation.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds);

    /**
     * Get continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     *
     * @param   pInterMeasurementPeriodMilliSeconds  Pointer to programmed
     *  Inter-Measurement Period in milliseconds.
     * @return  VL53L1_ERROR_NONE
     */
    VL53L1_Error VL53L1_GetInterMeasurementPeriodMilliSeconds(uint32_t *pInterMeasurementPeriodMilliSeconds);

    /**
     * @brief  target reflectance for Dmax setting
     * @par Function Description
     * Allow user to set the value for target reflectance @ 940nm to calculate the
     * ambient DMAX values for. Set to 50% by default by @a VL53L1_DataInit()
     *
     * @param   DmaxReflectance       Reflectance % in 16.16 fixed point
     * @return  VL53L1_ERROR_NONE     Success
     * @return  VL53L1_ERROR_INVALID_PARAMS     in case input value is not in range
     * from 0 to 100. Note that this is a fix point value so the max value is
     * 100 * 65536.
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetDmaxReflectance(FixPoint1616_t DmaxReflectance);

    /**
     * @brief  Get target reflectance for Dmax
     * @par Function Description
     * Retrieves the value for target reflectance @ 940nm to calculate the
     * ambient DMAX values for. Set to 50% by default by @a VL53L1_DataInit()
     *
     * @param   pDmaxReflectance      pointer to Reflectance % in 16.16 fixed point
     * @return  VL53L1_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetDmaxReflectance(FixPoint1616_t *pDmaxReflectance);

    /**
     * @brief Set function for ambient Dmax mode
     *
     * @param    DmaxMode             DMAX mode to be used in ranging
     *
     * @return   VL53L1_ERROR_NONE    Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetDmaxMode(VL53L1_DeviceDmaxModes DmaxMode);

    /**
     * @brief Get function for ambient Dmax mode
     *
     * @param pDmaxMode        output pointer to DMAX mode currently in use
     *
     * @return   VL53L1_ERROR_NONE    Success
     * @return  "Other error code"    See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetDmaxMode(VL53L1_DeviceDmaxModes *pDmaxMode);

    /**
     * @brief  Get the number of the check limit managed by a given Device
     *
     * @par Function Description
     * This function give the number of the check limit managed by the Device
     *
     * @param   pNumberOfLimitCheck           Pointer to the number of check limit.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetNumberOfLimitCheck(uint16_t *pNumberOfLimitCheck);

    /**
     * @brief  Return a description string for a given limit check number
     *
     * @par Function Description
     * This function returns a description string for a given limit check number.
     * The limit check is identified with the LimitCheckId.
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckString             Pointer to the description string of
     *          the given check limit. Shall be defined as char buf[VL53L1_MAX_STRING_LENGTH]
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetLimitCheckInfo(uint16_t LimitCheckId, char *pLimitCheckString);

    /**
     * @brief  Return a the Status of the specified check limit
     *
     * @par Function Description
     * This function returns the Status of the specified check limit.
     * The value indicate if the check is fail or not.
     * The limit check is identified with the LimitCheckId.
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckStatus             Pointer to the
     *          Limit Check Status of the given check limit.
     *          LimitCheckStatus :
     *          0 the check is not fail or not enabled
     *          1 the check if fail
     *
     * <p><ul>
     *    <li>VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE: the sigma indicate the quality
     *    of the measure. The more it is little the better it is.
     *    The status is 1 when current sigma is greater then the limit.</li>
     *    <li>VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE: the signal rate indicate
     *    the strength of the returned signal. The more it is big the better it is.
     *    The status is 1 when current signal is lower then the limit.</li>
     * </ul></p>
     *
     *
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetLimitCheckStatus(uint16_t LimitCheckId, uint8_t *pLimitCheckStatus);

    /**
     * @brief  Enable/Disable a specific limit check
     *
     * @par Function Description
     * This function Enable/Disable a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   LimitCheckEnable
     * @li set LimitCheckEnable=1 enables the LimitCheckId limit
     * @li set LimitCheckEnable=0 disables the LimitCheckId limit
     * @return  VL53L1_ERROR_NONE            Success
     * @return  VL53L1_ERROR_INVALID_PARAMS   This error is returned
     *          when LimitCheckId value is out of range.
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetLimitCheckEnable(uint16_t LimitCheckId, uint8_t LimitCheckEnable);

    /**
     * @brief  Get specific limit check enable state
     *
     * @par Function Description
     * This function get the enable state of a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckEnable             Pointer to the check limit enable value.
     * @li if 1 the check limit corresponding to LimitCheckId is Enabled
     * @li if 0 the check limit corresponding to LimitCheckId is disabled
     * @return  VL53L1_ERROR_NONE             Success
     * @return  VL53L1_ERROR_INVALID_PARAMS   This error is returned
     *          when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetLimitCheckEnable(uint16_t LimitCheckId, uint8_t *pLimitCheckEnable);

    /**
     * @brief  Set a specific limit check value
     *
     * @par Function Description
     * This function set a specific limit check value.
     * The limit check is identified with the LimitCheckId.
     *
     * @note Note that the value written with that function will not be applied if
     * the limit is not enabled. In other words this function will not enable the
     * limit but change only the value. In case the limit is not enabled the value
     * is saved internally and applied with VL53L1_SetLimitCheckEnable.
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   LimitCheckValue               Limit check Value for a given LimitCheckId
     * @return  VL53L1_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetLimitCheckValue(uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue);

    /**
     * @brief  Get a specific limit check value
     *
     * @par Function Description
     * This function get a specific limit check value from device then it updates
     * internal values and check enables.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function get the current value from device if zero then the value
     * returned is the one stored by the user, but in that case the check is store
     * as disabled. If the value from device is not zero, this is returned and set
     * into the memory at the same way that user call VL53L1_SetLimitCheckValue()
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckValue              Pointer to Limit check Value for a given LimitCheckId.
     * @return  VL53L1_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetLimitCheckValue(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue);

    /**
     * @brief  Get the current value of the signal used for the limit check
     *
     * @par Function Description
     * This function get a the current value of the signal used for the limit check.
     * To obtain the latest value you should run a valid ranging before.
     * The value reported is linked to the limit check identified with the
     * LimitCheckId.
     *
     * @param   LimitCheckId                  Limit Check ID
     *          (0<= LimitCheckId < VL53L1_GetNumberOfLimitCheck() ).
     * @param   pLimitCheckCurrent            Pointer to current Value for a given LimitCheckId.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetLimitCheckCurrent(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent);

    /**
     * @brief Get the Maximum number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get Maximum number of ROI Zones managed by the Device.
     *
     * @note The number of Zone depends on the preset mode used so to have the
     * right number this function should be call after @a VL53L1_SetPresetMode()
     * @note This function doesn't Access to the device
     *
     * @param   pMaxNumberOfROI   Pointer to the Maximum Number
     *  of ROI Zones value.
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetMaxNumberOfROI(uint8_t *pMaxNumberOfROI);

    /**
     * @brief Set the ROI  to be used for ranging
     *
     * @par Function Description
     * The user defined ROIs are rectangles described as per the following system
     * from the Top Left corner to the Bottom Right corner.
     * <br>Minimal ROI size is 4x4 spads
     * @image html roi_coord.png
     *
     * @param   pRoiConfig               Pointer to the Structure containing all the
     * ROI to be used.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetROI(VL53L1_RoiConfig_t *pRoiConfig);

    /**
     * @brief Get the ROI managed by the Device
     *
     * @par Function Description
     * Get the ROI managed by the Device
     *
     * @param   pRoiConfig            Pointer to the Structure containing all the
     * ROI to be used.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetROI(VL53L1_RoiConfig_t *pRoiConfig);

    /**
     * @brief Gets number of sequence steps managed by the API.
     *
     * @par Function Description
     * This function retrieves the number of sequence steps currently managed
     * by the API
     *
     * @note This function Accesses the device
     *
     * @param   pNumberOfSequenceSteps       Out parameter reporting the number of
     *                                       sequence steps.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetNumberOfSequenceSteps(uint8_t *pNumberOfSequenceSteps);

    /**
     * @brief Gets the name of a given sequence step.
     *
     * @par Function Description
     * This function retrieves the name of sequence steps corresponding to
     * SequenceStepId.
     *
     * @note This function doesn't Accesses the device
     *
     * @param   SequenceStepId               Sequence step identifier.
     * @param   pSequenceStepsString         Pointer to Info string. Shall be
     * defined as char buf[VL53L1_MAX_STRING_LENGTH]
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetSequenceStepsInfo(VL53L1_SequenceStepId SequenceStepId, char *pSequenceStepsString);

    /**
     * @brief Sets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function enables/disables a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   SequenceStepId           Sequence step identifier.
     * @param   SequenceStepEnabled          Demanded state {0=Off,1=On}
     *                                       is enabled.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  VL53L1_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetSequenceStepEnable(VL53L1_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled);

    /**
     * @brief Gets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function retrieves the state of a requested sequence step, i.e. on/off.
     *
     * @note This function Accesses the device
     *
     * @param   SequenceStepId         Sequence step identifier.
     * @param   pSequenceStepEnabled   Out parameter reporting if the sequence step
     *                                 is enabled {0=Off,1=On}.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  VL53L1_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetSequenceStepEnable(VL53L1_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled);

    /**
     * @brief Start device measurement
     *
     * @details Started measurement will depend on preset parameters set through
     * @a VL53L1_SetPreseMode()
     * This function will change the VL53L1_State from VL53L1_STATE_IDLE to
     * VL53L1_STATE_RUNNING.
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE                  Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED    This error occurs when
     * PresetMode programmed with @a VL53L1_SetPresetMode
     * @return  VL53L1_ERROR_TIME_OUT    Time out on start measurement
     * @return  VL53L1_ERROR_INVALID_PARAMS This error might occur in timed mode
     * when inter measurement period is smaller or too close to the timing budget.
     * In such case measurements are not started and user must correct the timings
     * passed to @a VL53L1_SetMeasurementTimingBudgetMicroSeconds() and
     * @a VL53L1_SetInterMeasurementPeriodMilliSeconds() functions.
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_StartMeasurement();

    /**
     * @brief Stop device measurement
     *
     * @details Will set the device in standby mode at end of current measurement\n
     *          Not necessary in single mode as device shall return automatically
     *          in standby mode at end of measurement.
     *          This function will change the VL53L1_State from VL53L1_STATE_RUNNING
     *          to VL53L1_STATE_IDLE.
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_StopMeasurement();

    /**
     * @brief Clear the Interrupt flag and start new measurement
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_ClearInterruptAndStartMeasurement();

    /**
     * @brief Return Measurement Data Ready
     *
     * @par Function Description
     * This function indicate that a measurement data is ready.
     * This function is used for non-blocking capture.
     *
     * @note This function Access to the device
     *
     * @param   pMeasurementDataReady  Pointer to Measurement Data Ready.
     * 0 = data not ready, 1 = data ready
     * @return  VL53L1_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetMeasurementDataReady(uint8_t *pMeasurementDataReady);

    /**
     * @brief Wait for measurement data ready.
     * Blocking function.
     * Note that the timeout is given by:
     * VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS defined in def.h
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE        Success
     * @return  VL53L1_ERROR_TIME_OUT In case of timeout
     */
    VL53L1_Error VL53L1_WaitMeasurementDataReady();

    /**
     * @brief Retrieve the measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Ranging measurement
     */
    /**
     * @warning this function will return only the first ROI data and only the
     * first object. For multi objects or multi ROI use:
     * @a Vl53L1_GetMultiRangingData.
     * In case of RANGING only one output is given, this can
     * be selected with the help of @a VL53L1_SetOutputMode()
     * In case of MULTIZONES_SCANNING and error will be raised because not
     * supported in that function.
     */
    /**
     *
     * @warning USER must call @a VL53L1_ClearInterruptAndStartMeasurement() prior
     * to call again this function
     *
     * @note This function Access to the device
     *
     * @note The first valid value returned by this function will have a range
     * status equal to VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK which means that
     * the data is valid but no wrap around check have been done. User should take
     * care about that.
     *
     * @param   pRangingMeasurementData  Pointer to the data structure to fill up.
     * @return  VL53L1_ERROR_NONE        Success
     * @return  VL53L1_ERROR_MODE_NOT_SUPPORTED    in case of MULTIZONES_SCANNING
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetRangingMeasurementData(VL53L1_RangingMeasurementData_t *pRangingMeasurementData);

    /**
     * @brief Retrieve all ROI's measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Ranging measurement
     * @warning USER should take care about  @a VL53L1_GetNumberOfROI()
     * before get data.
     * Bare driver will fill a NumberOfROI times the corresponding data
     * structure used in the measurement function.
     *
     * @warning USER must call @a VL53L1_ClearInterruptAndStartMeasurement() prior
     * to call again this function
     *
     * @note This function Access to the device
     *
     * @note The first valid value returned by this function will have a range
     * status equal to VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK which means that
     * the data is valid but no wrap around check have been done. User should take
     * care about that.
     *
     * @param   pMultiRangingData        Pointer to the data structure to fill up.
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetMultiRangingData(VL53L1_MultiRangingData_t *pMultiRangingData);

    /**
     * @brief Get Additional Data
     *
     * @par Function Description
     * This function is used to get lld debugging data on the last histogram
     * measurement. shall be called when a new measurement is ready (interrupt or
     * positive VL53L1_GetMeasurementDataReady() polling) and before a call to
     * VL53L1_ClearInterruptAndStartMeasurement(). Depending on the PresetMode
     * currently set parts of the returned data structure may be not relevant.
     *
     * @param   pAdditionalData          Pointer to Additional data
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetAdditionalData(VL53L1_AdditionalData_t *pAdditionalData);

    /**
     * @brief Set Tuning Parameter value for a given parameter ID
     *
     * @par Function Description
     * This function is used to improve the performance of the device. It permit to
     * change a particular value used for a timeout or a threshold or a constant
     * in an algorithm. The function will change the value of the parameter
     * identified by an unique ID.
     *
     * @note This function doesn't Access to the device
     *
     * @param   TuningParameterId            Tuning Parameter ID
     * @param   TuningParameterValue         Tuning Parameter Value
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetTuningParameter(uint16_t TuningParameterId, int32_t TuningParameterValue);

    /**
     * @brief Get Tuning Parameter value for a given parameter ID
     *
     * @par Function Description
     * This function is used to get the value of the parameter
     * identified by an unique ID.
     *
     * @note This function doesn't Access to the device
     *
     * @param   TuningParameterId            Tuning Parameter ID
     * @param   pTuningParameterValue        Pointer to Tuning Parameter Value
     * for a given TuningParameterId.
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetTuningParameter(uint16_t TuningParameterId, int32_t *pTuningParameterValue);

    /**
     * @brief Performs Reference Spad Management
     *
     * @par Function Description
     * The reference SPAD initialization procedure determines the minimum amount
     * of reference spads to be enables to achieve a target reference signal rate
     * and should be performed once during initialization.
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformRefSpadManagement();

    /**
     * @brief Enable/Disable dynamic Xtalk compensation feature
     *
     * Enable/Disable dynamic Xtalk compensation (aka smudge correction).
     *
     * @param   Mode   Set the smudge correction mode
     * See ::VL53L1_SmudgeCorrectionModes
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SmudgeCorrectionEnable(VL53L1_SmudgeCorrectionModes Mode);

    /**
     * @brief Enable/Disable Cross talk compensation feature
     *
     * Enable/Disable Cross Talk correction.
     *
     * @param   XTalkCompensationEnable   Cross talk compensation
     *  to be set 0 = disabled or 1 = enabled.
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetXTalkCompensationEnable(uint8_t XTalkCompensationEnable);

    /**
     * @brief Get Cross talk compensation rate enable
     *
     * Get if the Cross Talk is Enabled or Disabled.
     *
     * @note This function doesn't access to the device
     *
     * @param   pXTalkCompensationEnable   Pointer to the Cross talk compensation
     *  state 0=disabled or 1 = enabled
     * @return  VL53L1_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetXTalkCompensationEnable(uint8_t *pXTalkCompensationEnable);

    /**
     * @brief Perform XTalk Calibration
     *
     * @details Perform a XTalk calibration of the Device.
     * This function will launch a ranging measurement, if interrupts
     * are enabled an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     * This function will program a new value for the XTalk compensation
     * and it will enable the cross talk before exit.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   CalibrationOption    Select the Calibration to be run :
     * @param                        CalibrationOption
     * @li VL53L1_XTALKCALIBRATIONMODE_SINGLE_TARGET the calibration uses current
     * preset and distance mode without altering them.<br>
     * User must call @a VL53L1_SetPresetMode() with VL53L1_PRESETMODE_AUTONOMOUS,
     * VL53L1_PRESETMODE_LITE_RANGING or VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS
     * parameter prior to launch calibration
     * @li VL53L1_XTALKCALIBRATIONMODE_NO_TARGET the calibration sets appropriate
     * preset and distance mode and thus override existing ones<br>
     * User must call @a VL53L1_SetPresetMode() again after calibration to set the
     * desired one. during this calibration mode no object must be put below a 80cm
     * distance from the target
     * @li VL53L1_XTALKCALIBRATIONMODE_FULL_ROI the calibration sets appropriate
     * preset and distance mode and thus override existing ones<br>
     * User must call @a VL53L1_SetPresetMode() again after calibration to set the
     * desired one.
     * The ROI settings must define a single 16x16 ROI before to launch this
     * function.
     * The calibration uses a target which should be located at least @60cm from the
     * device. The actual location of the target shall be passed
     * through the bare driver tuning parameters table
     *
     * @return  VL53L1_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformXTalkCalibration(uint8_t CalibrationOption);

    /**
     * @brief Define the mode to be used for the offset calibration
     *
     * Define the mode to be used for the offset calibration. This function should
     * be called before run the @a VL53L1_PerformOffsetCalibration()
     *
     * @param   OffsetCalibrationMode     Offset Calibration Mode valid values are:
     * @li                                VL53L1_OFFSETCALIBRATIONMODE_STANDARD
     * @li                                VL53L1_OFFSETCALIBRATIONMODE_PRERANGE_ONLY
     * @li                                VL53L1_OFFSETCALIBRATIONMODE_MULTI_ZONE
     *
     * @return  VL53L1_ERROR_NONE         Success
     * @return  "Other error code"        See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetOffsetCalibrationMode(VL53L1_OffsetCalibrationModes OffsetCalibrationMode);

    /**
     * @brief Define the mode to be used for the offset correction
     *
     * Define the mode to be used for the offset correction.
     *
     * @param   OffsetCorrectionMode      Offset Correction Mode valid values are:
     * @li                                VL53L1_OFFSETCORRECTIONMODE_STANDARD
     * @li                                VL53L1_OFFSETCORRECTIONMODE_PERZONE
     * @li                                VL53L1_OFFSETCORRECTIONMODE_PERVCSEL
     *
     * @return  VL53L1_ERROR_NONE         Success
     * @return  "Other error code"        See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetOffsetCorrectionMode(VL53L1_OffsetCorrectionModes OffsetCorrectionMode);

    /**
     * @brief Perform Offset Calibration
     *
     * @details Perform a Offset calibration of the Device.
     * This function will launch a ranging measurement, if interrupts are
     * enabled interrupts will be done.
     * This function will program a new value for the Offset calibration value
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   CalDistanceMilliMeter     Calibration distance value used for the
     * offset compensation.
     * @param   CalReflectancePercent     Calibration Target reflectance @ 940nm
     * in percentage.
     *
     * @return  VL53L1_ERROR_NONE
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformOffsetCalibration(int32_t CalDistanceMilliMeter, FixPoint1616_t CalReflectancePercent);

    /**
     * @brief Perform Offset simple Calibration
     *
     * @details Perform a very simple offset calibration of the Device.
     * This function will launch few ranging measurements and computes offset
     * calibration. The preset mode and the distance mode MUST be set by the
     * application before to call this function.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   CalDistanceMilliMeter     Calibration distance value used for the
     * offset compensation.
     *
     * @return  VL53L1_ERROR_NONE
     * @return  VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance combined to the number of loops performed in the calibration lead to
     * an internal overflow. Try to reduce the distance of the target (140 mm)
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformOffsetSimpleCalibration(int32_t CalDistanceMilliMeter);

    /**
     * @brief Perform Offset simple Calibration with a "zero distance" target
     *
     * @details Perform a simple offset calibration of the Device.
     * This function will launch few ranging measurements and computes offset
     * calibration. The preset mode and the distance mode MUST be set by the
     * application before to call this function.
     * A target must be place very close to the device.
     * Ideally the target shall be touching the coverglass.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @return  VL53L1_ERROR_NONE
     * @return  VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance is too large, try to put the target closer to the device
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformOffsetZeroDistanceCalibration();

    /**
     * @brief Perform Offset per Vcsel Calibration. i.e. per distance mode
     *
     * @details Perform offset calibration of the Device depending on the
     * three distance mode settings: short, medium and long.
     * This function will launch few ranging measurements and computes offset
     * calibration in each of the three distance modes.
     * The preset mode MUST be set by the application before to call this function.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   CalDistanceMilliMeter     Distance of the target used for the
     * offset compensation calibration.
     *
     * @return  VL53L1_ERROR_NONE
     * @return  VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL the calibration failed by
     * lack of valid measurements
     * @return  VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH means that the target
     * distance combined to the number of loops performed in the calibration lead to
     * an internal overflow. Try to reduce the distance of the target (140 mm)
     * @return  "Other error code"   See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_PerformOffsetPerVcselCalibration(int32_t CalDistanceMilliMeter);

    /**
     * @brief Sets the Calibration Data.
     *
     * @par Function Description
     * This function set all the Calibration Data issued from the functions
     * @a VL53L1_PerformRefSpadManagement(), @a VL53L1_PerformXTalkCalibration,
     * @a VL53L1_PerformOffsetCalibration()
     *
     * @note This function doesn't Accesses the device
     *
     * @param   *pCalibrationData            Pointer to Calibration data to be set.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  VL53L1_ERROR_INVALID_PARAMS  pCalibrationData points to an older
     * version of the inner structure. Need for support to convert its content.
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetCalibrationData(VL53L1_CalibrationData_t *pCalibrationData);

    /**
     * @brief Gets the Calibration Data.
     *
     * @par Function Description
     * This function get all the Calibration Data issued from the functions
     * @a VL53L1_PerformRefSpadManagement(), @a VL53L1_PerformXTalkCalibration,
     * @a VL53L1_PerformOffsetCalibration()
     *
     * @note This function doesn't Accesses the device
     *
     * @param   *pCalibrationData            pointer where to store Calibration
     *  data.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetCalibrationData(VL53L1_CalibrationData_t  *pCalibrationData);

    /**
     * @brief Sets the Zone Calibration Data.
     *
     * @par Function Description
     * This function set all the Zone nCalibration Data issued from the functions
     * @a VL53L1_PerformOffsetCalibration() in multi zone
     *
     * @note This function doesn't Accesses the device
     *
     * @param   *pZoneCalibrationData        Pointer to Zone Calibration data to be
     *  set.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetZoneCalibrationData(VL53L1_ZoneCalibrationData_t *pZoneCalibrationData);

    /**
     * @brief Gets the Zone Calibration Data.
     *
     * @par Function Description
     * This function get all the Zone Calibration Data issued from the functions
     * @a VL53L1_PerformOffsetCalibration()
     *
     * @note This function doesn't Accesses the device
     *
     * @param   *pZoneCalibrationData        pointer where to store Zone Calibration
     *  data.
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetZoneCalibrationData(VL53L1_ZoneCalibrationData_t *pZoneCalibrationData);

    /**
     * @brief Gets the optical center.
     *
     * @par Function Description
     * This function get the optical center issued from the nvm set at FTM stage
     * expressed in the same coordinate system as the ROI are
     *
     * @note This function doesn't Accesses the device
     *
     * @param   pOpticalCenterX              pointer to the X position of center
     * in 16.16 fix point
     * @param   pOpticalCenterY              pointer to the Y position of center
     * in 16.16 fix point
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetOpticalCenter(FixPoint1616_t *pOpticalCenterX, FixPoint1616_t *pOpticalCenterY);

    /**
     * @brief Configure the interrupt config, from the given structure
     *
     * @param[in]    pConfig : pointer to configuration structure
     *
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_SetThresholdConfig(VL53L1_DetectionConfig_t *pConfig);

    /**
     * @brief Retrieves the interrupt config structure currently programmed
     *                             into the API
     *
     * @param[out]   pConfig : pointer to configuration structure
     *
     * @return  VL53L1_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L1_Error
     */
    VL53L1_Error VL53L1_GetThresholdConfig(VL53L1_DetectionConfig_t *pConfig);


  protected:

    /* Class */
    VL53L1_Error SingleTargetXTalkCalibration(VL53L1_DEV Dev);
    VL53L1_Error CheckValidRectRoi(VL53L1_UserRoi_t ROI);
    VL53L1_GPIO_Interrupt_Mode ConvertModeToLLD(VL53L1_Error *pStatus, VL53L1_ThresholdMode CrossMode);
    VL53L1_Error ComputeDevicePresetMode(VL53L1_PresetModes PresetMode, VL53L1_DistanceModes DistanceMode, VL53L1_DevicePresetModes *pDevicePresetMode);
    VL53L1_Error SetPresetMode(VL53L1_DEV Dev, VL53L1_PresetModes PresetMode, VL53L1_DistanceModes DistanceMode, uint32_t inter_measurement_period_ms);
    VL53L1_ThresholdMode ConvertModeFromLLD(VL53L1_Error *pStatus, VL53L1_GPIO_Interrupt_Mode CrossMode);
    VL53L1_Error SetLimitValue(VL53L1_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t value);
    void GenNewPresetMode(int16_t RefRange, VL53L1_DistanceModes InternalDistanceMode, VL53L1_DistanceModes *pNewDistanceMode);
    void CheckAndChangeDistanceMode(VL53L1_DEV Dev, VL53L1_TargetRangeData_t *pRangeData, int16_t Ambient100DmaxMm, VL53L1_DistanceModes *pNewDistanceMode);
    uint8_t ComputeRQL(uint8_t active_results, uint8_t FilteredRangeStatus, VL53L1_range_data_t *presults_data);
    uint8_t ConvertStatusLite(uint8_t FilteredRangeStatus);
    uint8_t ConvertStatusHisto(uint8_t FilteredRangeStatus);
    VL53L1_Error SetSimpleData(VL53L1_DEV Dev, uint8_t active_results, uint8_t device_status, VL53L1_range_data_t *presults_data, VL53L1_RangingMeasurementData_t *pRangeData);
    VL53L1_Error SetTargetData(VL53L1_DEV Dev, uint8_t active_results, uint8_t device_status, VL53L1_range_data_t *presults_data, VL53L1_TargetRangeData_t *pRangeData);
    uint8_t GetOutputDataIndex(VL53L1_DEV Dev, VL53L1_range_results_t *presults);
    VL53L1_Error SetMeasurementData(VL53L1_DEV Dev, VL53L1_range_results_t *presults, VL53L1_MultiRangingData_t *pMultiRangingData);


    /* API Calibration */
    VL53L1_Error VL53L1_run_ref_spad_char(VL53L1_DEV Dev,
                                          VL53L1_Error            *pcal_status);




    VL53L1_Error VL53L1_run_device_test(
      VL53L1_DEV                 Dev,
      VL53L1_DeviceTestMode      device_test_mode);




    VL53L1_Error VL53L1_run_spad_rate_map(
      VL53L1_DEV                 Dev,
      VL53L1_DeviceTestMode      device_test_mode,
      VL53L1_DeviceSscArray      array_select,
      uint32_t                   ssc_config_timeout_us,
      VL53L1_spad_rate_data_t   *pspad_rate_data);




    VL53L1_Error   VL53L1_run_xtalk_extraction(
      VL53L1_DEV                          Dev,
      VL53L1_Error                       *pcal_status);



    VL53L1_Error VL53L1_get_and_avg_xtalk_samples(
      VL53L1_DEV                    Dev,
      uint8_t                       num_of_samples,
      uint8_t                       measurement_mode,
      int16_t                       xtalk_filter_thresh_max_mm,
      int16_t                       xtalk_filter_thresh_min_mm,
      uint16_t                      xtalk_max_valid_rate_kcps,
      uint8_t                       xtalk_result_id,
      uint8_t                       xtalk_histo_id,
      VL53L1_xtalk_range_results_t *pxtalk_results,
      VL53L1_histogram_bin_data_t  *psum_histo,
      VL53L1_histogram_bin_data_t  *pavg_histo);



    VL53L1_Error   VL53L1_run_offset_calibration(
      VL53L1_DEV                    Dev,
      int16_t                       cal_distance_mm,
      uint16_t                      cal_reflectance_pc,
      VL53L1_Error                 *pcal_status);




    VL53L1_Error   VL53L1_run_phasecal_average(
      VL53L1_DEV              Dev,
      uint8_t                 measurement_mode,
      uint8_t                 phasecal_result__vcsel_start,
      uint16_t                phasecal_num_of_samples,
      VL53L1_range_results_t *prange_results,
      uint16_t               *pphasecal_result__reference_phase,
      uint16_t               *pzero_distance_phase);




    VL53L1_Error VL53L1_run_zone_calibration(
      VL53L1_DEV                    Dev,
      VL53L1_DevicePresetModes      device_preset_mode,
      VL53L1_DeviceZonePreset       zone_preset,
      VL53L1_zone_config_t         *pzone_cfg,
      int16_t                       cal_distance_mm,
      uint16_t                      cal_reflectance_pc,
      VL53L1_Error                 *pcal_status);




    void VL53L1_hist_xtalk_extract_data_init(
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data);



    VL53L1_Error VL53L1_hist_xtalk_extract_update(
      int16_t                             target_distance_mm,
      uint16_t                            target_width_oversize,
      VL53L1_histogram_bin_data_t        *phist_bins,
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data);



    VL53L1_Error VL53L1_hist_xtalk_extract_fini(
      VL53L1_histogram_bin_data_t        *phist_bins,
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data,
      VL53L1_xtalk_calibration_results_t *pxtalk_cal,
      VL53L1_xtalk_histogram_shape_t     *pxtalk_shape);




    VL53L1_Error   VL53L1_run_hist_xtalk_extraction(
      VL53L1_DEV                    Dev,
      int16_t                       cal_distance_mm,
      VL53L1_Error                 *pcal_status);



    /* API Core */
    VL53L1_Error VL53L1_get_version(
      VL53L1_DEV            Dev,
      VL53L1_ll_version_t  *pversion);




    VL53L1_Error VL53L1_get_device_firmware_version(
      VL53L1_DEV         Dev,
      uint16_t          *pfw_version);




    VL53L1_Error VL53L1_data_init(
      VL53L1_DEV         Dev,
      uint8_t            read_p2p_data);




    VL53L1_Error VL53L1_read_p2p_data(
      VL53L1_DEV      Dev);




    VL53L1_Error VL53L1_software_reset(
      VL53L1_DEV      Dev);




    VL53L1_Error VL53L1_set_part_to_part_data(
      VL53L1_DEV                            Dev,
      VL53L1_calibration_data_t            *pcal_data);




    VL53L1_Error VL53L1_get_part_to_part_data(
      VL53L1_DEV                            Dev,
      VL53L1_calibration_data_t            *pcal_data);




    VL53L1_Error VL53L1_get_tuning_debug_data(
      VL53L1_DEV                            Dev,
      VL53L1_tuning_parameters_t            *ptun_data);




    VL53L1_Error VL53L1_set_inter_measurement_period_ms(
      VL53L1_DEV          Dev,
      uint32_t            inter_measurement_period_ms);




    VL53L1_Error VL53L1_get_inter_measurement_period_ms(
      VL53L1_DEV          Dev,
      uint32_t           *pinter_measurement_period_ms);




    VL53L1_Error VL53L1_set_timeouts_us(
      VL53L1_DEV          Dev,
      uint32_t            phasecal_config_timeout_us,
      uint32_t            mm_config_timeout_us,
      uint32_t            range_config_timeout_us);




    VL53L1_Error VL53L1_get_timeouts_us(
      VL53L1_DEV          Dev,
      uint32_t           *pphasecal_config_timeout_us,
      uint32_t           *pmm_config_timeout_us,
      uint32_t           *prange_config_timeout_us);




    VL53L1_Error VL53L1_set_calibration_repeat_period(
      VL53L1_DEV          Dev,
      uint16_t            cal_config__repeat_period);




    VL53L1_Error VL53L1_get_calibration_repeat_period(
      VL53L1_DEV          Dev,
      uint16_t           *pcal_config__repeat_period);




    VL53L1_Error VL53L1_set_sequence_config_bit(
      VL53L1_DEV                   Dev,
      VL53L1_DeviceSequenceConfig  bit_id,
      uint8_t                      value);




    VL53L1_Error VL53L1_get_sequence_config_bit(
      VL53L1_DEV                   Dev,
      VL53L1_DeviceSequenceConfig  bit_id,
      uint8_t                     *pvalue);




    VL53L1_Error VL53L1_set_interrupt_polarity(
      VL53L1_DEV                       Dev,
      VL53L1_DeviceInterruptPolarity  interrupt_polarity);




    VL53L1_Error VL53L1_get_interrupt_polarity(
      VL53L1_DEV                      Dev,
      VL53L1_DeviceInterruptPolarity  *pinterrupt_polarity);



    VL53L1_Error VL53L1_get_refspadchar_config_struct(
      VL53L1_DEV                     Dev,
      VL53L1_refspadchar_config_t   *pdata);



    VL53L1_Error VL53L1_set_refspadchar_config_struct(
      VL53L1_DEV                     Dev,
      VL53L1_refspadchar_config_t   *pdata);



    VL53L1_Error VL53L1_set_range_ignore_threshold(
      VL53L1_DEV              Dev,
      uint8_t                 range_ignore_thresh_mult,
      uint16_t                range_ignore_threshold_mcps);



    VL53L1_Error VL53L1_get_range_ignore_threshold(
      VL53L1_DEV              Dev,
      uint8_t                *prange_ignore_thresh_mult,
      uint16_t               *prange_ignore_threshold_mcps_internal,
      uint16_t               *prange_ignore_threshold_mcps_current);




    VL53L1_Error VL53L1_set_user_zone(
      VL53L1_DEV          Dev,
      VL53L1_user_zone_t *puser_zone);




    VL53L1_Error VL53L1_get_user_zone(
      VL53L1_DEV          Dev,
      VL53L1_user_zone_t *puser_zone);




    VL53L1_Error VL53L1_get_mode_mitigation_roi(
      VL53L1_DEV          Dev,
      VL53L1_user_zone_t *pmm_roi);




    VL53L1_Error VL53L1_set_zone_config(
      VL53L1_DEV             Dev,
      VL53L1_zone_config_t  *pzone_cfg);




    VL53L1_Error VL53L1_get_zone_config(
      VL53L1_DEV             Dev,
      VL53L1_zone_config_t  *pzone_cfg);




    VL53L1_Error VL53L1_set_preset_mode(
      VL53L1_DEV                   Dev,
      VL53L1_DevicePresetModes     device_preset_mode,
      uint16_t                     dss_config__target_total_rate_mcps,
      uint32_t                     phasecal_config_timeout_us,
      uint32_t                     mm_config_timeout_us,
      uint32_t                     range_config_timeout_us,
      uint32_t                     inter_measurement_period_ms);




    VL53L1_Error VL53L1_get_preset_mode_timing_cfg(
      VL53L1_DEV                   Dev,
      VL53L1_DevicePresetModes     device_preset_mode,
      uint16_t                    *pdss_config__target_total_rate_mcps,
      uint32_t                    *pphasecal_config_timeout_us,
      uint32_t                    *pmm_config_timeout_us,
      uint32_t                    *prange_config_timeout_us);



    VL53L1_Error VL53L1_set_zone_preset(
      VL53L1_DEV               Dev,
      VL53L1_DeviceZonePreset  zone_preset);



    VL53L1_Error VL53L1_enable_xtalk_compensation(
      VL53L1_DEV                 Dev);



    VL53L1_Error VL53L1_disable_xtalk_compensation(
      VL53L1_DEV                 Dev);




    void VL53L1_get_xtalk_compensation_enable(
      VL53L1_DEV    Dev,
      uint8_t       *pcrosstalk_compensation_enable);



    VL53L1_Error VL53L1_init_and_start_range(
      VL53L1_DEV                      Dev,
      uint8_t                         measurement_mode,
      VL53L1_DeviceConfigLevel        device_config_level);




    VL53L1_Error VL53L1_stop_range(
      VL53L1_DEV  Dev);




    VL53L1_Error VL53L1_get_measurement_results(
      VL53L1_DEV                  Dev,
      VL53L1_DeviceResultsLevel   device_result_level);




    VL53L1_Error VL53L1_get_device_results(
      VL53L1_DEV                 Dev,
      VL53L1_DeviceResultsLevel  device_result_level,
      VL53L1_range_results_t    *prange_results);




    VL53L1_Error VL53L1_clear_interrupt_and_enable_next_range(
      VL53L1_DEV       Dev,
      uint8_t          measurement_mode);




    VL53L1_Error VL53L1_get_histogram_bin_data(
      VL53L1_DEV                   Dev,
      VL53L1_histogram_bin_data_t *phist_data);




    void VL53L1_copy_sys_and_core_results_to_range_results(
      int32_t                           gain_factor,
      VL53L1_system_results_t          *psys,
      VL53L1_core_results_t            *pcore,
      VL53L1_range_results_t           *presults);



    VL53L1_Error VL53L1_set_zone_dss_config(
      VL53L1_DEV                      Dev,
      VL53L1_zone_private_dyn_cfg_t  *pzone_dyn_cfg);




    VL53L1_Error VL53L1_calc_ambient_dmax(
      VL53L1_DEV    Dev,
      uint16_t      target_reflectance,
      int16_t      *pambient_dmax_mm);




    VL53L1_Error VL53L1_set_GPIO_interrupt_config(
      VL53L1_DEV                      Dev,
      VL53L1_GPIO_Interrupt_Mode  intr_mode_distance,
      VL53L1_GPIO_Interrupt_Mode  intr_mode_rate,
      uint8_t       intr_new_measure_ready,
      uint8_t       intr_no_target,
      uint8_t       intr_combined_mode,
      uint16_t      thresh_distance_high,
      uint16_t      thresh_distance_low,
      uint16_t      thresh_rate_high,
      uint16_t      thresh_rate_low
    );



    VL53L1_Error VL53L1_set_GPIO_interrupt_config_struct(
      VL53L1_DEV                      Dev,
      VL53L1_GPIO_interrupt_config_t  intconf);



    VL53L1_Error VL53L1_get_GPIO_interrupt_config(
      VL53L1_DEV                      Dev,
      VL53L1_GPIO_interrupt_config_t  *pintconf);




    VL53L1_Error VL53L1_set_dmax_mode(
      VL53L1_DEV              Dev,
      VL53L1_DeviceDmaxMode   dmax_mode);



    VL53L1_Error VL53L1_get_dmax_mode(
      VL53L1_DEV               Dev,
      VL53L1_DeviceDmaxMode   *pdmax_mode);




    VL53L1_Error VL53L1_get_dmax_calibration_data(
      VL53L1_DEV                      Dev,
      VL53L1_DeviceDmaxMode           dmax_mode,
      uint8_t                         zone_id,
      VL53L1_dmax_calibration_data_t *pdmax_cal);




    VL53L1_Error VL53L1_set_hist_dmax_config(
      VL53L1_DEV                      Dev,
      VL53L1_hist_gen3_dmax_config_t *pdmax_cfg);



    VL53L1_Error VL53L1_get_hist_dmax_config(
      VL53L1_DEV                      Dev,
      VL53L1_hist_gen3_dmax_config_t *pdmax_cfg);




    VL53L1_Error VL53L1_set_offset_calibration_mode(
      VL53L1_DEV                      Dev,
      VL53L1_OffsetCalibrationMode   offset_cal_mode);




    VL53L1_Error VL53L1_get_offset_calibration_mode(
      VL53L1_DEV                      Dev,
      VL53L1_OffsetCalibrationMode  *poffset_cal_mode);




    VL53L1_Error VL53L1_set_offset_correction_mode(
      VL53L1_DEV                     Dev,
      VL53L1_OffsetCalibrationMode   offset_cor_mode);




    VL53L1_Error VL53L1_get_offset_correction_mode(
      VL53L1_DEV                    Dev,
      VL53L1_OffsetCorrectionMode  *poffset_cor_mode);




    VL53L1_Error VL53L1_set_zone_calibration_data(
      VL53L1_DEV                         Dev,
      VL53L1_zone_calibration_results_t *pzone_cal);




    VL53L1_Error VL53L1_get_zone_calibration_data(
      VL53L1_DEV                         Dev,
      VL53L1_zone_calibration_results_t *pzone_cal);




    VL53L1_Error VL53L1_get_lite_xtalk_margin_kcps(
      VL53L1_DEV                          Dev,
      int16_t                           *pxtalk_margin);



    VL53L1_Error VL53L1_set_lite_xtalk_margin_kcps(
      VL53L1_DEV                          Dev,
      int16_t                             xtalk_margin);




    VL53L1_Error VL53L1_get_histogram_xtalk_margin_kcps(
      VL53L1_DEV                          Dev,
      int16_t                           *pxtalk_margin);



    VL53L1_Error VL53L1_set_histogram_xtalk_margin_kcps(
      VL53L1_DEV                          Dev,
      int16_t                             xtalk_margin);



    VL53L1_Error VL53L1_get_histogram_phase_consistency(
      VL53L1_DEV                          Dev,
      uint8_t                            *pphase_consistency);



    VL53L1_Error VL53L1_set_histogram_phase_consistency(
      VL53L1_DEV                          Dev,
      uint8_t                             phase_consistency);



    VL53L1_Error VL53L1_get_histogram_event_consistency(
      VL53L1_DEV                          Dev,
      uint8_t                            *pevent_consistency);



    VL53L1_Error VL53L1_set_histogram_event_consistency(
      VL53L1_DEV                          Dev,
      uint8_t                             event_consistency);



    VL53L1_Error VL53L1_get_histogram_ambient_threshold_sigma(
      VL53L1_DEV                          Dev,
      uint8_t                            *pamb_thresh_sigma);



    VL53L1_Error VL53L1_set_histogram_ambient_threshold_sigma(
      VL53L1_DEV                          Dev,
      uint8_t                             amb_thresh_sigma);



    VL53L1_Error VL53L1_get_lite_min_count_rate(
      VL53L1_DEV                          Dev,
      uint16_t                           *plite_mincountrate);




    VL53L1_Error VL53L1_set_lite_min_count_rate(
      VL53L1_DEV                          Dev,
      uint16_t                            lite_mincountrate);





    VL53L1_Error VL53L1_get_lite_sigma_threshold(
      VL53L1_DEV                          Dev,
      uint16_t                           *plite_sigma);




    VL53L1_Error VL53L1_set_lite_sigma_threshold(
      VL53L1_DEV                          Dev,
      uint16_t                            lite_sigma);




    VL53L1_Error VL53L1_restore_xtalk_nvm_default(
      VL53L1_DEV                     Dev);



    VL53L1_Error VL53L1_get_xtalk_detect_config(
      VL53L1_DEV                          Dev,
      int16_t                            *pmax_valid_range_mm,
      int16_t                            *pmin_valid_range_mm,
      uint16_t                           *pmax_valid_rate_kcps,
      uint16_t                           *pmax_sigma_mm);



    VL53L1_Error VL53L1_set_xtalk_detect_config(
      VL53L1_DEV                          Dev,
      int16_t                             max_valid_range_mm,
      int16_t                             min_valid_range_mm,
      uint16_t                            max_valid_rate_kcps,
      uint16_t                            max_sigma_mm);



    VL53L1_Error VL53L1_get_target_order_mode(
      VL53L1_DEV                          Dev,
      VL53L1_HistTargetOrder             *phist_target_order);



    VL53L1_Error VL53L1_set_target_order_mode(
      VL53L1_DEV                          Dev,
      VL53L1_HistTargetOrder              hist_target_order);




    VL53L1_Error VL53L1_set_dmax_reflectance_values(
      VL53L1_DEV                          Dev,
      VL53L1_dmax_reflectance_array_t    *pdmax_reflectances);



    VL53L1_Error VL53L1_get_dmax_reflectance_values(
      VL53L1_DEV                          Dev,
      VL53L1_dmax_reflectance_array_t    *pdmax_reflectances);



    VL53L1_Error VL53L1_set_vhv_config(
      VL53L1_DEV                   Dev,
      uint8_t                      vhv_init_en,
      uint8_t                      vhv_init_value);



    VL53L1_Error VL53L1_get_vhv_config(
      VL53L1_DEV                   Dev,
      uint8_t                     *pvhv_init_en,
      uint8_t                     *pvhv_init_value);



    VL53L1_Error VL53L1_set_vhv_loopbound(
      VL53L1_DEV                   Dev,
      uint8_t                      vhv_loopbound);



    VL53L1_Error VL53L1_get_vhv_loopbound(
      VL53L1_DEV                   Dev,
      uint8_t                     *pvhv_loopbound);



    VL53L1_Error VL53L1_get_tuning_parm(
      VL53L1_DEV                     Dev,
      VL53L1_TuningParms             tuning_parm_key,
      int32_t                       *ptuning_parm_value);



    VL53L1_Error VL53L1_set_tuning_parm(
      VL53L1_DEV                     Dev,
      VL53L1_TuningParms             tuning_parm_key,
      int32_t                        tuning_parm_value);



    VL53L1_Error VL53L1_dynamic_xtalk_correction_enable(
      VL53L1_DEV                     Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_disable(
      VL53L1_DEV                     Dev
    );




    VL53L1_Error VL53L1_dynamic_xtalk_correction_apply_enable(
      VL53L1_DEV                          Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_apply_disable(
      VL53L1_DEV                          Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_single_apply_enable(
      VL53L1_DEV                          Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_single_apply_disable(
      VL53L1_DEV                          Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_set_scalers(
      VL53L1_DEV  Dev,
      int16_t   x_scaler_in,
      int16_t   y_scaler_in,
      uint8_t   user_scaler_set_in
    );



    VL53L1_Error VL53L1_get_current_xtalk_settings(
      VL53L1_DEV                          Dev,
      VL53L1_xtalk_calibration_results_t *pxtalk
    );



    VL53L1_Error VL53L1_set_current_xtalk_settings(
      VL53L1_DEV                          Dev,
      VL53L1_xtalk_calibration_results_t *pxtalk
    );

    VL53L1_Error VL53L1_load_patch(VL53L1_DEV Dev);

    VL53L1_Error VL53L1_unload_patch(VL53L1_DEV Dev);

    VL53L1_Error select_offset_per_vcsel(VL53L1_LLDriverData_t *pdev,
                                         int16_t *poffset);

    void vl53l1_diff_histo_stddev(VL53L1_LLDriverData_t *pdev,
                                  VL53L1_histogram_bin_data_t *pdata, uint8_t timing, uint8_t HighIndex,
                                  uint8_t prev_pos, int32_t *pdiff_histo_stddev);

    void vl53l1_histo_merge(VL53L1_DEV Dev,
                            VL53L1_histogram_bin_data_t *pdata);


    /* API Debug */

    VL53L1_Error VL53L1_decode_calibration_data_buffer(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_calibration_data_t *pdata);






    VL53L1_Error VL53L1_get_nvm_debug_data(
      VL53L1_DEV                 Dev,
      VL53L1_decoded_nvm_data_t *pdata);



    VL53L1_Error VL53L1_get_histogram_debug_data(
      VL53L1_DEV                   Dev,
      VL53L1_histogram_bin_data_t *pdata);






    VL53L1_Error VL53L1_get_additional_data(
      VL53L1_DEV                Dev,
      VL53L1_additional_data_t *pdata);






    VL53L1_Error VL53L1_get_xtalk_debug_data(
      VL53L1_DEV                 Dev,
      VL53L1_xtalk_debug_data_t *pdata);




    VL53L1_Error VL53L1_get_offset_debug_data(
      VL53L1_DEV                 Dev,
      VL53L1_offset_debug_data_t *pdata);



    /* API Preset Modes */
    VL53L1_Error VL53L1_init_refspadchar_config_struct(
      VL53L1_refspadchar_config_t     *pdata);




    VL53L1_Error VL53L1_init_ssc_config_struct(
      VL53L1_ssc_config_t     *pdata);




    VL53L1_Error VL53L1_init_xtalk_config_struct(
      VL53L1_customer_nvm_managed_t *pnvm,
      VL53L1_xtalk_config_t   *pdata);



    VL53L1_Error VL53L1_init_xtalk_extract_config_struct(
      VL53L1_xtalkextract_config_t   *pdata);



    VL53L1_Error VL53L1_init_offset_cal_config_struct(
      VL53L1_offsetcal_config_t   *pdata);



    VL53L1_Error VL53L1_init_zone_cal_config_struct(
      VL53L1_zonecal_config_t   *pdata);



    VL53L1_Error VL53L1_init_hist_post_process_config_struct(
      uint8_t                              xtalk_compensation_enable,
      VL53L1_hist_post_process_config_t   *pdata);




    VL53L1_Error VL53L1_init_dmax_calibration_data_struct(
      VL53L1_dmax_calibration_data_t   *pdata);




    VL53L1_Error VL53L1_init_tuning_parm_storage_struct(
      VL53L1_tuning_parm_storage_t   *pdata);



    VL53L1_Error VL53L1_init_hist_gen3_dmax_config_struct(
      VL53L1_hist_gen3_dmax_config_t   *pdata);




    VL53L1_Error VL53L1_preset_mode_standard_ranging(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_standard_ranging_short_range(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_standard_ranging_long_range(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_standard_ranging_mm1_cal(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_standard_ranging_mm2_cal(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_timed_ranging(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_timed_ranging_short_range(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_timed_ranging_long_range(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_low_power_auto_ranging(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg,
      VL53L1_low_power_auto_data_t *plpadata);



    VL53L1_Error VL53L1_preset_mode_low_power_auto_short_ranging(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg,
      VL53L1_low_power_auto_data_t *plpadata);



    VL53L1_Error VL53L1_preset_mode_low_power_auto_long_ranging(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg,
      VL53L1_low_power_auto_data_t *plpadata);



    VL53L1_Error VL53L1_preset_mode_histogram_ranging(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_ranging_with_mm1(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_ranging_with_mm2(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_ranging_mm1_cal(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_ranging_mm2_cal(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_ranging_ref(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_characterisation(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_xtalk_planar(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_xtalk_mm1(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);




    VL53L1_Error VL53L1_preset_mode_histogram_xtalk_mm2(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_multizone(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_multizone_short_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_multizone_long_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_ranging_short_timing(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_long_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_medium_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_short_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_special_histogram_short_range(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_long_range_mm1(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_long_range_mm2(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_medium_range_mm1(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_medium_range_mm2(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_short_range_mm1(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_histogram_short_range_mm2(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_static_config_t            *pstatic,
      VL53L1_histogram_config_t         *phistogram,
      VL53L1_general_config_t           *pgeneral,
      VL53L1_timing_config_t            *ptiming,
      VL53L1_dynamic_config_t           *pdynamic,
      VL53L1_system_control_t           *psystem,
      VL53L1_tuning_parm_storage_t      *ptuning_parms,
      VL53L1_zone_config_t              *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_olt(
      VL53L1_static_config_t     *pstatic,
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic,
      VL53L1_system_control_t    *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t       *pzone_cfg);



    VL53L1_Error VL53L1_preset_mode_singleshot_ranging(

      VL53L1_static_config_t    *pstatic,
      VL53L1_histogram_config_t *phistogram,
      VL53L1_general_config_t   *pgeneral,
      VL53L1_timing_config_t    *ptiming,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_system_control_t   *psystem,
      VL53L1_tuning_parm_storage_t *ptuning_parms,
      VL53L1_zone_config_t      *pzone_cfg);




    void VL53L1_copy_hist_cfg_to_static_cfg(
      VL53L1_histogram_config_t  *phistogram,
      VL53L1_static_config_t     *pstatic,
      VL53L1_general_config_t    *pgeneral,
      VL53L1_timing_config_t     *ptiming,
      VL53L1_dynamic_config_t    *pdynamic);



    void VL53L1_copy_hist_bins_to_static_cfg(
      VL53L1_histogram_config_t *phistogram,
      VL53L1_static_config_t    *pstatic,
      VL53L1_timing_config_t    *ptiming);

    /* API String */

    VL53L1_Error VL53L1_get_range_status_string(
      uint8_t   RangeStatus,
      char    *pRangeStatusString);


    VL53L1_Error VL53L1_get_pal_state_string(
      VL53L1_State   PalStateCode,
      char         *pPalStateString);


    VL53L1_Error VL53L1_get_sequence_steps_info(
      VL53L1_SequenceStepId SequenceStepId,
      char *pSequenceStepsString);


    VL53L1_Error VL53L1_get_limit_check_info(uint16_t LimitCheckId,
                                             char *pLimitCheckString);


    /* Core*/
    void VL53L1_init_version(
      VL53L1_DEV         Dev);




    void VL53L1_init_ll_driver_state(
      VL53L1_DEV         Dev,
      VL53L1_DeviceState ll_state);




    VL53L1_Error VL53L1_update_ll_driver_rd_state(
      VL53L1_DEV         Dev);




    VL53L1_Error VL53L1_check_ll_driver_rd_state(
      VL53L1_DEV         Dev);




    VL53L1_Error VL53L1_update_ll_driver_cfg_state(
      VL53L1_DEV         Dev);




    void VL53L1_copy_rtn_good_spads_to_buffer(
      VL53L1_nvm_copy_data_t  *pdata,
      uint8_t                 *pbuffer);




    void VL53L1_init_system_results(
      VL53L1_system_results_t      *pdata);




    void V53L1_init_zone_results_structure(
      uint8_t                 active_zones,
      VL53L1_zone_results_t  *pdata);




    void V53L1_init_zone_dss_configs(
      VL53L1_DEV              Dev);




    void VL53L1_init_histogram_config_structure(
      uint8_t   even_bin0,
      uint8_t   even_bin1,
      uint8_t   even_bin2,
      uint8_t   even_bin3,
      uint8_t   even_bin4,
      uint8_t   even_bin5,
      uint8_t   odd_bin0,
      uint8_t   odd_bin1,
      uint8_t   odd_bin2,
      uint8_t   odd_bin3,
      uint8_t   odd_bin4,
      uint8_t   odd_bin5,
      VL53L1_histogram_config_t  *pdata);



    void VL53L1_init_histogram_multizone_config_structure(
      uint8_t   even_bin0,
      uint8_t   even_bin1,
      uint8_t   even_bin2,
      uint8_t   even_bin3,
      uint8_t   even_bin4,
      uint8_t   even_bin5,
      uint8_t   odd_bin0,
      uint8_t   odd_bin1,
      uint8_t   odd_bin2,
      uint8_t   odd_bin3,
      uint8_t   odd_bin4,
      uint8_t   odd_bin5,
      VL53L1_histogram_config_t  *pdata);




    void VL53L1_init_xtalk_bin_data_struct(
      uint32_t                        bin_value,
      uint16_t                        VL53L1_p_024,
      VL53L1_xtalk_histogram_shape_t *pdata);




    void VL53L1_i2c_encode_uint16_t(
      uint16_t    ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);




    uint16_t VL53L1_i2c_decode_uint16_t(
      uint16_t    count,
      uint8_t    *pbuffer);




    void VL53L1_i2c_encode_int16_t(
      int16_t     ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);




    int16_t VL53L1_i2c_decode_int16_t(
      uint16_t    count,
      uint8_t    *pbuffer);




    void VL53L1_i2c_encode_uint32_t(
      uint32_t    ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);




    uint32_t VL53L1_i2c_decode_uint32_t(
      uint16_t    count,
      uint8_t    *pbuffer);




    uint32_t VL53L1_i2c_decode_with_mask(
      uint16_t    count,
      uint8_t    *pbuffer,
      uint32_t    bit_mask,
      uint32_t    down_shift,
      uint32_t    offset);




    void VL53L1_i2c_encode_int32_t(
      int32_t     ip_value,
      uint16_t    count,
      uint8_t    *pbuffer);




    int32_t VL53L1_i2c_decode_int32_t(
      uint16_t    count,
      uint8_t    *pbuffer);




    VL53L1_Error VL53L1_start_test(
      VL53L1_DEV     Dev,
      uint8_t        test_mode__ctrl);




    VL53L1_Error VL53L1_set_firmware_enable_register(
      VL53L1_DEV         Dev,
      uint8_t            value);




    VL53L1_Error VL53L1_enable_firmware(
      VL53L1_DEV         Dev);




    VL53L1_Error VL53L1_disable_firmware(
      VL53L1_DEV         Dev);




    VL53L1_Error VL53L1_set_powerforce_register(
      VL53L1_DEV         Dev,
      uint8_t            value);





    VL53L1_Error VL53L1_enable_powerforce(
      VL53L1_DEV         Dev);



    VL53L1_Error VL53L1_disable_powerforce(
      VL53L1_DEV         Dev);





    VL53L1_Error VL53L1_clear_interrupt(
      VL53L1_DEV         Dev);





    VL53L1_Error VL53L1_force_shadow_stream_count_to_zero(
      VL53L1_DEV         Dev);




    uint32_t VL53L1_calc_macro_period_us(
      uint16_t fast_osc_frequency,
      uint8_t  VL53L1_p_009);




    uint16_t VL53L1_calc_range_ignore_threshold(
      uint32_t central_rate,
      int16_t  x_gradient,
      int16_t  y_gradient,
      uint8_t  rate_mult);




    uint32_t VL53L1_calc_timeout_mclks(
      uint32_t  timeout_us,
      uint32_t  macro_period_us);



    uint16_t VL53L1_calc_encoded_timeout(
      uint32_t  timeout_us,
      uint32_t  macro_period_us);




    uint32_t VL53L1_calc_timeout_us(
      uint32_t  timeout_mclks,
      uint32_t  macro_period_us);



    uint32_t VL53L1_calc_decoded_timeout_us(
      uint16_t  timeout_encoded,
      uint32_t  macro_period_us);




    uint16_t VL53L1_encode_timeout(
      uint32_t timeout_mclks);




    uint32_t VL53L1_decode_timeout(
      uint16_t encoded_timeout);




    VL53L1_Error  VL53L1_calc_timeout_register_values(
      uint32_t                 phasecal_config_timeout_us,
      uint32_t                 mm_config_timeout_us,
      uint32_t                 range_config_timeout_us,
      uint16_t                 fast_osc_frequency,
      VL53L1_general_config_t *pgeneral,
      VL53L1_timing_config_t  *ptiming);




    uint8_t VL53L1_encode_vcsel_period(
      uint8_t VL53L1_p_031);




    uint32_t VL53L1_decode_unsigned_integer(
      uint8_t  *pbuffer,
      uint8_t   no_of_bytes);




    void   VL53L1_encode_unsigned_integer(
      uint32_t  ip_value,
      uint8_t   no_of_bytes,
      uint8_t  *pbuffer);




    VL53L1_Error VL53L1_hist_copy_and_scale_ambient_info(
      VL53L1_zone_hist_info_t        *pidata,
      VL53L1_histogram_bin_data_t    *podata);




    void  VL53L1_hist_get_bin_sequence_config(
      VL53L1_DEV                     Dev,
      VL53L1_histogram_bin_data_t   *pdata);




    VL53L1_Error  VL53L1_hist_phase_consistency_check(
      VL53L1_DEV                   Dev,
      VL53L1_zone_hist_info_t     *phist_prev,
      VL53L1_zone_objects_t       *prange_prev,
      VL53L1_range_results_t      *prange_curr);







    VL53L1_Error  VL53L1_hist_events_consistency_check(
      uint8_t                      event_sigma,
      uint16_t                     min_effective_spad_count,
      VL53L1_zone_hist_info_t     *phist_prev,
      VL53L1_object_data_t        *prange_prev,
      VL53L1_range_data_t         *prange_curr,
      int32_t                     *pevents_tolerance,
      int32_t                     *pevents_delta,
      VL53L1_DeviceError          *prange_status);







    VL53L1_Error  VL53L1_hist_merged_pulse_check(
      int16_t                      min_max_tolerance_mm,
      VL53L1_range_data_t         *pdata,
      VL53L1_DeviceError          *prange_status);






    VL53L1_Error  VL53L1_hist_xmonitor_consistency_check(
      VL53L1_DEV                   Dev,
      VL53L1_zone_hist_info_t     *phist_prev,
      VL53L1_zone_objects_t       *prange_prev,
      VL53L1_range_data_t         *prange_curr);






    VL53L1_Error  VL53L1_hist_wrap_dmax(
      VL53L1_hist_post_process_config_t *phistpostprocess,
      VL53L1_histogram_bin_data_t       *pcurrent,
      int16_t                           *pwrap_dmax_mm);




    void VL53L1_hist_combine_mm1_mm2_offsets(
      int16_t                              mm1_offset_mm,
      int16_t                              mm2_offset_mm,
      uint8_t                              encoded_mm_roi_centre,
      uint8_t                              encoded_mm_roi_size,
      uint8_t                              encoded_zone_centre,
      uint8_t                              encoded_zone_size,
      VL53L1_additional_offset_cal_data_t *pcal_data,
      uint8_t                             *pgood_spads,
      uint16_t                             aperture_attenuation,
      int16_t                             *prange_offset_mm);




    VL53L1_Error VL53L1_hist_xtalk_extract_calc_window(
      int16_t                             target_distance_mm,
      uint16_t                            target_width_oversize,
      VL53L1_histogram_bin_data_t        *phist_bins,
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data);




    VL53L1_Error VL53L1_hist_xtalk_extract_calc_event_sums(
      VL53L1_histogram_bin_data_t        *phist_bins,
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data);




    VL53L1_Error VL53L1_hist_xtalk_extract_calc_rate_per_spad(
      VL53L1_hist_xtalk_extract_data_t   *pxtalk_data);



    VL53L1_Error VL53L1_hist_xtalk_extract_calc_shape(
      VL53L1_hist_xtalk_extract_data_t  *pxtalk_data,
      VL53L1_xtalk_histogram_shape_t    *pxtalk_shape);



    VL53L1_Error VL53L1_hist_xtalk_shape_model(
      uint16_t                         events_per_bin,
      uint16_t                         pulse_centre,
      uint16_t                         pulse_width,
      VL53L1_xtalk_histogram_shape_t  *pxtalk_shape);




    uint16_t VL53L1_hist_xtalk_shape_model_interp(
      uint16_t      events_per_bin,
      uint32_t      phase_delta);




    void VL53L1_spad_number_to_byte_bit_index(
      uint8_t  spad_number,
      uint8_t *pbyte_index,
      uint8_t *pbit_index,
      uint8_t *pbit_mask);




    void VL53L1_encode_row_col(
      uint8_t  row,
      uint8_t  col,
      uint8_t *pspad_number);




    void VL53L1_decode_zone_size(
      uint8_t   encoded_xy_size,
      uint8_t  *pwidth,
      uint8_t  *pheight);




    void VL53L1_encode_zone_size(
      uint8_t  width,
      uint8_t  height,
      uint8_t *pencoded_xy_size);




    void VL53L1_decode_zone_limits(
      uint8_t   encoded_xy_centre,
      uint8_t   encoded_xy_size,
      int16_t  *px_ll,
      int16_t  *py_ll,
      int16_t  *px_ur,
      int16_t  *py_ur);




    uint8_t VL53L1_is_aperture_location(
      uint8_t   row,
      uint8_t   col);




    void VL53L1_calc_max_effective_spads(
      uint8_t     encoded_zone_centre,
      uint8_t     encoded_zone_size,
      uint8_t    *pgood_spads,
      uint16_t    aperture_attenuation,
      uint16_t   *pmax_effective_spads);




    void VL53L1_calc_mm_effective_spads(
      uint8_t     encoded_mm_roi_centre,
      uint8_t     encoded_mm_roi_size,
      uint8_t     encoded_zone_centre,
      uint8_t     encoded_zone_size,
      uint8_t    *pgood_spads,
      uint16_t    aperture_attenuation,
      uint16_t   *pmm_inner_effective_spads,
      uint16_t   *pmm_outer_effective_spads);




    void VL53L1_hist_copy_results_to_sys_and_core(
      VL53L1_histogram_bin_data_t      *pbins,
      VL53L1_range_results_t           *phist,
      VL53L1_system_results_t          *psys,
      VL53L1_core_results_t            *pcore);




    VL53L1_Error VL53L1_sum_histogram_data(
      VL53L1_histogram_bin_data_t *phist_input,
      VL53L1_histogram_bin_data_t *phist_output);




    VL53L1_Error VL53L1_avg_histogram_data(
      uint8_t no_of_samples,
      VL53L1_histogram_bin_data_t *phist_sum,
      VL53L1_histogram_bin_data_t *phist_avg);




    VL53L1_Error VL53L1_save_cfg_data(
      VL53L1_DEV  Dev);




    VL53L1_Error VL53L1_dynamic_zone_update(
      VL53L1_DEV  Dev,
      VL53L1_range_results_t *presults);




    VL53L1_Error VL53L1_update_internal_stream_counters(
      VL53L1_DEV  Dev,
      uint8_t     external_stream_count,
      uint8_t     *pinternal_stream_count,
      uint8_t     *pinternal_stream_count_val
    );



    VL53L1_Error VL53L1_multizone_hist_bins_update(
      VL53L1_DEV  Dev);



    VL53L1_Error VL53L1_set_histogram_multizone_initial_bin_config(
      VL53L1_zone_config_t           *pzone_cfg,
      VL53L1_histogram_config_t      *phist_cfg,
      VL53L1_histogram_config_t      *pmulti_hist
    );



    uint8_t VL53L1_encode_GPIO_interrupt_config(
      VL53L1_GPIO_interrupt_config_t  *pintconf);



    VL53L1_GPIO_interrupt_config_t VL53L1_decode_GPIO_interrupt_config(
      uint8_t   system__interrupt_config);



    VL53L1_Error VL53L1_set_GPIO_distance_threshold(
      VL53L1_DEV                      Dev,
      uint16_t      threshold_high,
      uint16_t      threshold_low);



    VL53L1_Error VL53L1_set_GPIO_rate_threshold(
      VL53L1_DEV                      Dev,
      uint16_t      threshold_high,
      uint16_t      threshold_low);



    VL53L1_Error VL53L1_set_GPIO_thresholds_from_struct(
      VL53L1_DEV                      Dev,
      VL53L1_GPIO_interrupt_config_t *pintconf);





    VL53L1_Error VL53L1_set_ref_spad_char_config(
      VL53L1_DEV    Dev,
      uint8_t       vcsel_period_a,
      uint32_t      phasecal_timeout_us,
      uint16_t      total_rate_target_mcps,
      uint16_t      max_count_rate_rtn_limit_mcps,
      uint16_t      min_count_rate_rtn_limit_mcps,
      uint16_t      fast_osc_frequency);




    VL53L1_Error VL53L1_set_ssc_config(
      VL53L1_DEV           Dev,
      VL53L1_ssc_config_t *pssc_cfg,
      uint16_t             fast_osc_frequency);




    VL53L1_Error VL53L1_get_spad_rate_data(
      VL53L1_DEV                Dev,
      VL53L1_spad_rate_data_t  *pspad_rates);



    uint32_t VL53L1_calc_crosstalk_plane_offset_with_margin(
      uint32_t     plane_offset_kcps,
      int16_t      margin_offset_kcps);



    VL53L1_Error VL53L1_low_power_auto_data_init(
      VL53L1_DEV                     Dev
    );



    VL53L1_Error VL53L1_low_power_auto_data_stop_range(
      VL53L1_DEV                     Dev
    );




    VL53L1_Error VL53L1_dynamic_xtalk_correction_calc_required_samples(
      VL53L1_DEV                     Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_calc_new_xtalk(
      VL53L1_DEV        Dev,
      uint32_t        xtalk_offset_out,
      VL53L1_smudge_corrector_config_t  *pconfig,
      VL53L1_smudge_corrector_data_t    *pout,
      uint8_t         add_smudge,
      uint8_t         soft_update
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_corrector(
      VL53L1_DEV                     Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_data_init(
      VL53L1_DEV                     Dev
    );



    VL53L1_Error VL53L1_dynamic_xtalk_correction_output_init(
      VL53L1_LLDriverResults_t *pres
    );



    VL53L1_Error VL53L1_xtalk_cal_data_init(
      VL53L1_DEV                          Dev
    );



    VL53L1_Error VL53L1_config_low_power_auto_mode(
      VL53L1_general_config_t   *pgeneral,
      VL53L1_dynamic_config_t   *pdynamic,
      VL53L1_low_power_auto_data_t *plpadata
    );



    VL53L1_Error VL53L1_low_power_auto_setup_manual_calibration(
      VL53L1_DEV        Dev);



    VL53L1_Error VL53L1_low_power_auto_update_DSS(
      VL53L1_DEV        Dev);


    VL53L1_Error VL53L1_compute_histo_merge_nb(
      VL53L1_DEV        Dev,  uint8_t *histo_merge_nb);

    /* Core Support */

    uint32_t VL53L1_calc_pll_period_us(
      uint16_t fast_osc_frequency);





    uint32_t VL53L1_duration_maths(
      uint32_t  pll_period_us,
      uint32_t  vcsel_parm_pclks,
      uint32_t  window_vclks,
      uint32_t  periods_elapsed_mclks);



    uint32_t VL53L1_events_per_spad_maths(
      int32_t   VL53L1_p_013,
      uint16_t  num_spads,
      uint32_t  duration);




    uint32_t VL53L1_isqrt(
      uint32_t  num);




    void VL53L1_hist_calc_zero_distance_phase(
      VL53L1_histogram_bin_data_t    *pdata);




    void VL53L1_hist_estimate_ambient_from_thresholded_bins(
      int32_t                      ambient_threshold_sigma,
      VL53L1_histogram_bin_data_t *pdata);




    void VL53L1_hist_remove_ambient_bins(
      VL53L1_histogram_bin_data_t    *pdata);




    uint32_t VL53L1_calc_pll_period_mm(
      uint16_t fast_osc_frequency);




    uint16_t VL53L1_rate_maths(
      int32_t   VL53L1_p_008,
      uint32_t  time_us);




    uint16_t VL53L1_rate_per_spad_maths(
      uint32_t  frac_bits,
      uint32_t  peak_count_rate,
      uint16_t  num_spads,
      uint32_t  max_output_value);




    int32_t VL53L1_range_maths(
      uint16_t  fast_osc_frequency,
      uint16_t  VL53L1_p_017,
      uint16_t  zero_distance_phase,
      uint8_t   fractional_bits,
      int32_t   gain_factor,
      int32_t   range_offset_mm);




    uint8_t VL53L1_decode_vcsel_period(
      uint8_t vcsel_period_reg);



    void VL53L1_copy_xtalk_bin_data_to_histogram_data_struct(
      VL53L1_xtalk_histogram_shape_t *pxtalk,
      VL53L1_histogram_bin_data_t    *phist);




    void VL53L1_init_histogram_bin_data_struct(
      int32_t                      bin_value,
      uint16_t                     VL53L1_p_024,
      VL53L1_histogram_bin_data_t *pdata);




    void VL53L1_decode_row_col(
      uint8_t   spad_number,
      uint8_t  *prow,
      uint8_t  *pcol);




    void VL53L1_hist_find_min_max_bin_values(
      VL53L1_histogram_bin_data_t   *pdata);




    void VL53L1_hist_estimate_ambient_from_ambient_bins(
      VL53L1_histogram_bin_data_t    *pdata);


    /* dmax */

    VL53L1_Error VL53L1_f_001(
      uint16_t                              target_reflectance,
      VL53L1_dmax_calibration_data_t       *pcal,
      VL53L1_hist_gen3_dmax_config_t       *pcfg,
      VL53L1_histogram_bin_data_t          *pbins,
      VL53L1_hist_gen3_dmax_private_data_t *pdata,
      int16_t                              *pambient_dmax_mm);




    uint32_t VL53L1_f_002(
      uint32_t     events_threshold,
      uint32_t     ref_signal_events,
      uint32_t   ref_distance_mm,
      uint32_t     signal_thresh_sigma);

    /* Error String */

    VL53L1_Error VL53L1_get_pal_error_string(
      VL53L1_Error   PalErrorCode,
      char         *pPalErrorString);

    /* Hist Algos Gen3 */

    void VL53L1_f_016(
      VL53L1_hist_gen3_algo_private_data_t   *palgo);






    VL53L1_Error VL53L1_f_018(
      uint16_t                               ambient_threshold_events_scaler,
      int32_t                                ambient_threshold_sigma,
      int32_t                                min_ambient_threshold_events,
      uint8_t                            algo__crosstalk_compensation_enable,
      VL53L1_histogram_bin_data_t           *pbins,
      VL53L1_histogram_bin_data_t           *pxtalk,
      VL53L1_hist_gen3_algo_private_data_t  *palgo);






    VL53L1_Error VL53L1_f_019(
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_020(
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_021(
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_028(
      VL53L1_HistTargetOrder                target_order,
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_022(
      uint8_t                                pulse_no,
      VL53L1_histogram_bin_data_t           *pbins,
      VL53L1_hist_gen3_algo_private_data_t  *palgo);



    VL53L1_Error VL53L1_f_027(
      uint8_t                                pulse_no,
      uint8_t                             clip_events,
      VL53L1_histogram_bin_data_t           *pbins,
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_030(
      int16_t                            VL53L1_p_022,
      int16_t                            VL53L1_p_026,
      uint8_t                            VL53L1_p_031,
      uint8_t                            clip_events,
      VL53L1_histogram_bin_data_t       *pbins,
      uint32_t                          *pphase);




    VL53L1_Error VL53L1_f_023(
      uint8_t                                pulse_no,
      VL53L1_histogram_bin_data_t           *pbins,
      VL53L1_hist_gen3_algo_private_data_t  *palgo,
      int32_t                                pad_value,
      VL53L1_histogram_bin_data_t           *ppulse);




    VL53L1_Error VL53L1_f_026(
      uint8_t                       bin,
      uint8_t                       sigma_estimator__sigma_ref_mm,
      uint8_t                       VL53L1_p_031,
      uint8_t                       VL53L1_p_055,
      uint8_t                       crosstalk_compensation_enable,
      VL53L1_histogram_bin_data_t  *phist_data_ap,
      VL53L1_histogram_bin_data_t  *phist_data_zp,
      VL53L1_histogram_bin_data_t  *pxtalk_hist,
      uint16_t                     *psigma_est);




    void VL53L1_f_029(
      uint8_t                      range_id,
      uint8_t                      valid_phase_low,
      uint8_t                      valid_phase_high,
      uint16_t                     sigma_thres,
      VL53L1_histogram_bin_data_t *pbins,
      VL53L1_hist_pulse_data_t    *ppulse,
      VL53L1_range_data_t         *pdata);


    /* Hist Algos Gen4 */

    void VL53L1_f_032(
      VL53L1_hist_gen4_algo_filtered_data_t  *palgo);




    VL53L1_Error VL53L1_f_033(
      VL53L1_dmax_calibration_data_t         *pdmax_cal,
      VL53L1_hist_gen3_dmax_config_t         *pdmax_cfg,
      VL53L1_hist_post_process_config_t      *ppost_cfg,
      VL53L1_histogram_bin_data_t            *pbins,
      VL53L1_histogram_bin_data_t            *pxtalk,
      VL53L1_hist_gen3_algo_private_data_t   *palgo,
      VL53L1_hist_gen4_algo_filtered_data_t  *pfiltered,
      VL53L1_hist_gen3_dmax_private_data_t   *pdmax_algo,
      VL53L1_range_results_t                 *presults,
      uint8_t                                histo_merge_nb);





    VL53L1_Error VL53L1_f_034(
      uint8_t                                pulse_no,
      VL53L1_histogram_bin_data_t           *ppulse,
      VL53L1_hist_gen3_algo_private_data_t  *palgo,
      VL53L1_hist_gen4_algo_filtered_data_t *pfiltered);




    VL53L1_Error VL53L1_f_035(
      uint8_t                                pulse_no,
      uint16_t                               noise_threshold,
      VL53L1_hist_gen4_algo_filtered_data_t *pfiltered,
      VL53L1_hist_gen3_algo_private_data_t  *palgo);




    VL53L1_Error VL53L1_f_036(
      uint8_t   bin,
      int32_t   VL53L1_p_003,
      int32_t   VL53L1_p_018,
      int32_t   VL53L1_p_001,
      int32_t   ax,
      int32_t   bx,
      int32_t   cx,
      int32_t   VL53L1_p_004,
      uint8_t   VL53L1_p_031,
      uint32_t *pmedian_phase);


    /* Hist Char */

    VL53L1_Error VL53L1_set_calib_config(
      VL53L1_DEV      Dev,
      uint8_t         vcsel_delay__a0,
      uint8_t         calib_1,
      uint8_t         calib_2,
      uint8_t         calib_3,
      uint8_t         calib_2__a0,
      uint8_t         spad_readout);




    VL53L1_Error VL53L1_set_hist_calib_pulse_delay(
      VL53L1_DEV      Dev,
      uint8_t         calib_delay);




    VL53L1_Error VL53L1_disable_calib_pulse_delay(
      VL53L1_DEV      Dev);


    /* Hist Core */

    void  VL53L1_f_013(
      uint8_t                         VL53L1_p_018,
      uint8_t                         filter_woi,
      VL53L1_histogram_bin_data_t    *pbins,
      int32_t                        *pa,
      int32_t                        *pb,
      int32_t                        *pc);




    VL53L1_Error VL53L1_f_011(
      uint16_t                        vcsel_width,
      uint16_t                        fast_osc_frequency,
      uint32_t                        total_periods_elapsed,
      uint16_t                        VL53L1_p_006,
      VL53L1_range_data_t            *pdata,
      uint8_t histo_merge_nb);




    void VL53L1_f_012(
      uint16_t             gain_factor,
      int16_t              range_offset_mm,
      VL53L1_range_data_t *pdata);




    void  VL53L1_f_037(
      VL53L1_histogram_bin_data_t   *pdata,
      int32_t                        ambient_estimate_counts_per_bin);




    void  VL53L1_f_004(
      VL53L1_histogram_bin_data_t   *pxtalk,
      VL53L1_histogram_bin_data_t   *pbins,
      VL53L1_histogram_bin_data_t   *pxtalk_realigned);



    int8_t  VL53L1_f_038(
      VL53L1_histogram_bin_data_t   *pdata1,
      VL53L1_histogram_bin_data_t   *pdata2);



    VL53L1_Error  VL53L1_f_039(
      VL53L1_histogram_bin_data_t   *pidata,
      VL53L1_histogram_bin_data_t   *podata);


    /* Hist Funcs */

    VL53L1_Error VL53L1_hist_process_data(
      VL53L1_dmax_calibration_data_t    *pdmax_cal,
      VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53L1_hist_post_process_config_t *ppost_cfg,
      VL53L1_histogram_bin_data_t       *pbins,
      VL53L1_xtalk_histogram_data_t     *pxtalk,
      uint8_t                           *pArea1,
      uint8_t                           *pArea2,
      VL53L1_range_results_t             *presults,
      uint8_t                            *HistMergeNumber);




    VL53L1_Error VL53L1_hist_ambient_dmax(
      uint16_t                            target_reflectance,
      VL53L1_dmax_calibration_data_t     *pdmax_cal,
      VL53L1_hist_gen3_dmax_config_t     *pdmax_cfg,
      VL53L1_histogram_bin_data_t        *pbins,
      int16_t                            *pambient_dmax_mm);


    /* NVM */

    VL53L1_Error VL53L1_nvm_enable(
      VL53L1_DEV     Dev,
      uint16_t       nvm_ctrl_pulse_width,
      int32_t        nvm_power_up_delay_us);




    VL53L1_Error VL53L1_nvm_read(
      VL53L1_DEV     Dev,
      uint8_t        start_address,
      uint8_t        count,
      uint8_t       *pdata);




    VL53L1_Error VL53L1_nvm_disable(
      VL53L1_DEV     Dev);




    VL53L1_Error VL53L1_nvm_format_decode(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_decoded_nvm_data_t *pdata);




    VL53L1_Error VL53L1_nvm_decode_optical_centre(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53L1_optical_centre_t             *pdata);




    VL53L1_Error VL53L1_nvm_decode_cal_peak_rate_map(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53L1_cal_peak_rate_map_t          *pdata);




    VL53L1_Error VL53L1_nvm_decode_additional_offset_cal_data(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53L1_additional_offset_cal_data_t *pdata);




    VL53L1_Error VL53L1_nvm_decode_fmt_range_results_data(
      uint16_t                             buf_size,
      uint8_t                             *pbuffer,
      VL53L1_decoded_nvm_fmt_range_data_t *pdata);




    VL53L1_Error VL53L1_nvm_decode_fmt_info(
      uint16_t                       buf_size,
      uint8_t                       *pbuffer,
      VL53L1_decoded_nvm_fmt_info_t *pdata);




    VL53L1_Error VL53L1_nvm_decode_ews_info(
      uint16_t                       buf_size,
      uint8_t                       *pbuffer,
      VL53L1_decoded_nvm_ews_info_t *pdata);




    void VL53L1_nvm_format_encode(
      VL53L1_decoded_nvm_data_t *pnvm_info,
      uint8_t                   *pnvm_data);




    VL53L1_Error VL53L1_read_nvm_raw_data(
      VL53L1_DEV     Dev,
      uint8_t        start_address,
      uint8_t        count,
      uint8_t       *pnvm_raw_data);




    VL53L1_Error VL53L1_read_nvm(
      VL53L1_DEV                 Dev,
      uint8_t                    nvm_format,
      VL53L1_decoded_nvm_data_t *pnvm_info);




    VL53L1_Error VL53L1_read_nvm_optical_centre(
      VL53L1_DEV                           Dev,
      VL53L1_optical_centre_t             *pcentre);




    VL53L1_Error VL53L1_read_nvm_cal_peak_rate_map(
      VL53L1_DEV                           Dev,
      VL53L1_cal_peak_rate_map_t          *pcal_data);




    VL53L1_Error VL53L1_read_nvm_additional_offset_cal_data(
      VL53L1_DEV                           Dev,
      VL53L1_additional_offset_cal_data_t *pcal_data);




    VL53L1_Error VL53L1_read_nvm_fmt_range_results_data(
      VL53L1_DEV                           Dev,
      uint16_t                             range_results_select,
      VL53L1_decoded_nvm_fmt_range_data_t *prange_data);


    /* Platform Ipp */

    VL53L1_Error VL53L1_ipp_hist_process_data(
      VL53L1_DEV                         Dev,
      VL53L1_dmax_calibration_data_t    *pdmax_cal,
      VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53L1_hist_post_process_config_t *ppost_cfg,
      VL53L1_histogram_bin_data_t       *pbins,
      VL53L1_xtalk_histogram_data_t     *pxtalk,
      uint8_t                           *pArea1,
      uint8_t                           *pArea2,
      uint8_t                           *phisto_merge_nb,
      VL53L1_range_results_t            *presults);


    VL53L1_Error VL53L1_ipp_hist_ambient_dmax(
      VL53L1_DEV                         Dev,
      uint16_t                           target_reflectance,
      VL53L1_dmax_calibration_data_t    *pdmax_cal,
      VL53L1_hist_gen3_dmax_config_t    *pdmax_cfg,
      VL53L1_histogram_bin_data_t       *pbins,
      int16_t                           *pambient_dmax_mm);

    VL53L1_Error VL53L1_ipp_xtalk_calibration_process_data(
      VL53L1_DEV                          Dev,
      VL53L1_xtalk_range_results_t       *pxtalk_ranges,
      VL53L1_xtalk_histogram_data_t      *pxtalk_shape,
      VL53L1_xtalk_calibration_results_t *pxtalk_cal);

    VL53L1_Error VL53L1_ipp_hist_xtalk_correction(
      VL53L1_DEV                    Dev,
      VL53L1_customer_nvm_managed_t *pcustomer,
      VL53L1_dynamic_config_t       *pdyn_cfg,
      VL53L1_xtalk_histogram_data_t *pxtalk_shape,
      VL53L1_histogram_bin_data_t   *pip_hist_data,
      VL53L1_histogram_bin_data_t   *pop_hist_data,
      VL53L1_histogram_bin_data_t   *pxtalk_count_data);


    VL53L1_Error VL53L1_ipp_generate_dual_reflectance_xtalk_samples(
      VL53L1_DEV                     Dev,
      VL53L1_xtalk_range_results_t  *pxtalk_results,
      uint16_t                       expected_target_distance_mm,
      uint8_t                        higher_reflectance,
      VL53L1_histogram_bin_data_t   *pxtalk_avg_samples);


    /* Register Funcs */

    VL53L1_Error VL53L1_i2c_encode_static_nvm_managed(
      VL53L1_static_nvm_managed_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_static_nvm_managed(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_static_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_set_static_nvm_managed(
      VL53L1_DEV                 Dev,
      VL53L1_static_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_get_static_nvm_managed(
      VL53L1_DEV                 Dev,
      VL53L1_static_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_customer_nvm_managed(
      VL53L1_customer_nvm_managed_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_customer_nvm_managed(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_customer_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_set_customer_nvm_managed(
      VL53L1_DEV                 Dev,
      VL53L1_customer_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_get_customer_nvm_managed(
      VL53L1_DEV                 Dev,
      VL53L1_customer_nvm_managed_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_static_config(
      VL53L1_static_config_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_static_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_static_config_t    *pdata);




    VL53L1_Error VL53L1_set_static_config(
      VL53L1_DEV                 Dev,
      VL53L1_static_config_t    *pdata);




    VL53L1_Error VL53L1_get_static_config(
      VL53L1_DEV                 Dev,
      VL53L1_static_config_t    *pdata);




    VL53L1_Error VL53L1_i2c_encode_general_config(
      VL53L1_general_config_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_general_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_general_config_t   *pdata);




    VL53L1_Error VL53L1_set_general_config(
      VL53L1_DEV                 Dev,
      VL53L1_general_config_t   *pdata);




    VL53L1_Error VL53L1_get_general_config(
      VL53L1_DEV                 Dev,
      VL53L1_general_config_t   *pdata);




    VL53L1_Error VL53L1_i2c_encode_timing_config(
      VL53L1_timing_config_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_timing_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_timing_config_t    *pdata);




    VL53L1_Error VL53L1_set_timing_config(
      VL53L1_DEV                 Dev,
      VL53L1_timing_config_t    *pdata);




    VL53L1_Error VL53L1_get_timing_config(
      VL53L1_DEV                 Dev,
      VL53L1_timing_config_t    *pdata);




    VL53L1_Error VL53L1_i2c_encode_dynamic_config(
      VL53L1_dynamic_config_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_dynamic_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_dynamic_config_t   *pdata);




    VL53L1_Error VL53L1_set_dynamic_config(
      VL53L1_DEV                 Dev,
      VL53L1_dynamic_config_t   *pdata);




    VL53L1_Error VL53L1_get_dynamic_config(
      VL53L1_DEV                 Dev,
      VL53L1_dynamic_config_t   *pdata);




    VL53L1_Error VL53L1_i2c_encode_system_control(
      VL53L1_system_control_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_system_control(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_system_control_t   *pdata);




    VL53L1_Error VL53L1_set_system_control(
      VL53L1_DEV                 Dev,
      VL53L1_system_control_t   *pdata);




    VL53L1_Error VL53L1_get_system_control(
      VL53L1_DEV                 Dev,
      VL53L1_system_control_t   *pdata);




    VL53L1_Error VL53L1_i2c_encode_system_results(
      VL53L1_system_results_t   *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_system_results_t   *pdata);




    VL53L1_Error VL53L1_set_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_system_results_t   *pdata);




    VL53L1_Error VL53L1_get_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_system_results_t   *pdata);




    VL53L1_Error VL53L1_i2c_encode_core_results(
      VL53L1_core_results_t     *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_core_results_t     *pdata);




    VL53L1_Error VL53L1_set_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_core_results_t     *pdata);




    VL53L1_Error VL53L1_get_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_core_results_t     *pdata);




    VL53L1_Error VL53L1_i2c_encode_debug_results(
      VL53L1_debug_results_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_debug_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_debug_results_t    *pdata);




    VL53L1_Error VL53L1_set_debug_results(
      VL53L1_DEV                 Dev,
      VL53L1_debug_results_t    *pdata);




    VL53L1_Error VL53L1_get_debug_results(
      VL53L1_DEV                 Dev,
      VL53L1_debug_results_t    *pdata);




    VL53L1_Error VL53L1_i2c_encode_nvm_copy_data(
      VL53L1_nvm_copy_data_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_nvm_copy_data(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_nvm_copy_data_t    *pdata);




    VL53L1_Error VL53L1_set_nvm_copy_data(
      VL53L1_DEV                 Dev,
      VL53L1_nvm_copy_data_t    *pdata);




    VL53L1_Error VL53L1_get_nvm_copy_data(
      VL53L1_DEV                 Dev,
      VL53L1_nvm_copy_data_t    *pdata);




    VL53L1_Error VL53L1_i2c_encode_prev_shadow_system_results(
      VL53L1_prev_shadow_system_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_prev_shadow_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_prev_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_set_prev_shadow_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_prev_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_get_prev_shadow_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_prev_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_prev_shadow_core_results(
      VL53L1_prev_shadow_core_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_prev_shadow_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_prev_shadow_core_results_t  *pdata);




    VL53L1_Error VL53L1_set_prev_shadow_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_prev_shadow_core_results_t  *pdata);




    VL53L1_Error VL53L1_get_prev_shadow_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_prev_shadow_core_results_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_patch_debug(
      VL53L1_patch_debug_t      *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_patch_debug(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_patch_debug_t      *pdata);




    VL53L1_Error VL53L1_set_patch_debug(
      VL53L1_DEV                 Dev,
      VL53L1_patch_debug_t      *pdata);




    VL53L1_Error VL53L1_get_patch_debug(
      VL53L1_DEV                 Dev,
      VL53L1_patch_debug_t      *pdata);




    VL53L1_Error VL53L1_i2c_encode_gph_general_config(
      VL53L1_gph_general_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_gph_general_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_gph_general_config_t  *pdata);




    VL53L1_Error VL53L1_set_gph_general_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_general_config_t  *pdata);




    VL53L1_Error VL53L1_get_gph_general_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_general_config_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_gph_static_config(
      VL53L1_gph_static_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_gph_static_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_gph_static_config_t  *pdata);




    VL53L1_Error VL53L1_set_gph_static_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_static_config_t  *pdata);




    VL53L1_Error VL53L1_get_gph_static_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_static_config_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_gph_timing_config(
      VL53L1_gph_timing_config_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_gph_timing_config(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_gph_timing_config_t  *pdata);




    VL53L1_Error VL53L1_set_gph_timing_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_timing_config_t  *pdata);




    VL53L1_Error VL53L1_get_gph_timing_config(
      VL53L1_DEV                 Dev,
      VL53L1_gph_timing_config_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_fw_internal(
      VL53L1_fw_internal_t      *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_fw_internal(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_fw_internal_t      *pdata);




    VL53L1_Error VL53L1_set_fw_internal(
      VL53L1_DEV                 Dev,
      VL53L1_fw_internal_t      *pdata);




    VL53L1_Error VL53L1_get_fw_internal(
      VL53L1_DEV                 Dev,
      VL53L1_fw_internal_t      *pdata);




    VL53L1_Error VL53L1_i2c_encode_patch_results(
      VL53L1_patch_results_t    *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_patch_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_patch_results_t    *pdata);




    VL53L1_Error VL53L1_set_patch_results(
      VL53L1_DEV                 Dev,
      VL53L1_patch_results_t    *pdata);




    VL53L1_Error VL53L1_get_patch_results(
      VL53L1_DEV                 Dev,
      VL53L1_patch_results_t    *pdata);




    VL53L1_Error VL53L1_i2c_encode_shadow_system_results(
      VL53L1_shadow_system_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_shadow_system_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_set_shadow_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_get_shadow_system_results(
      VL53L1_DEV                 Dev,
      VL53L1_shadow_system_results_t  *pdata);




    VL53L1_Error VL53L1_i2c_encode_shadow_core_results(
      VL53L1_shadow_core_results_t  *pdata,
      uint16_t                   buf_size,
      uint8_t                   *pbuffer);




    VL53L1_Error VL53L1_i2c_decode_shadow_core_results(
      uint16_t                   buf_size,
      uint8_t                   *pbuffer,
      VL53L1_shadow_core_results_t  *pdata);




    VL53L1_Error VL53L1_set_shadow_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_shadow_core_results_t  *pdata);




    VL53L1_Error VL53L1_get_shadow_core_results(
      VL53L1_DEV                 Dev,
      VL53L1_shadow_core_results_t  *pdata);


    /* Sigma Estimate */

    uint16_t VL53L1_f_042(
      uint8_t  sigma_estimator__effective_pulse_width_ns,
      uint8_t  sigma_estimator__effective_ambient_width_ns,
      uint8_t  sigma_estimator__sigma_ref_mm,
      VL53L1_range_data_t  *pdata);




    uint16_t VL53L1_f_044(
      uint8_t  sigma_estimator__effective_pulse_width_ns,
      uint8_t  sigma_estimator__effective_ambient_width_ns,
      uint8_t  sigma_estimator__sigma_ref_mm,
      VL53L1_range_data_t *pdata);






    VL53L1_Error  VL53L1_f_045(
      uint8_t       sigma_estimator__sigma_ref_mm,
      uint32_t      VL53L1_p_003,
      uint32_t      VL53L1_p_018,
      uint32_t      VL53L1_p_001,
      uint32_t      a_zp,
      uint32_t      c_zp,
      uint32_t      bx,
      uint32_t      ax_zp,
      uint32_t      cx_zp,
      uint32_t      VL53L1_p_004,
      uint16_t      fast_osc_frequency,
      uint16_t      *psigma_est);







    VL53L1_Error  VL53L1_f_014(
      uint8_t       sigma_estimator__sigma_ref_mm,
      uint32_t      VL53L1_p_003,
      uint32_t      VL53L1_p_018,
      uint32_t      VL53L1_p_001,
      uint32_t      a_zp,
      uint32_t      c_zp,
      uint32_t      bx,
      uint32_t      ax_zp,
      uint32_t      cx_zp,
      uint32_t      VL53L1_p_004,
      uint16_t      fast_osc_frequency,
      uint16_t      *psigma_est);



    uint32_t VL53L1_f_046(
      uint64_t VL53L1_p_003,
      uint32_t size
    );





    uint32_t VL53L1_f_043(
      uint32_t  VL53L1_p_003,
      uint32_t  VL53L1_p_018);


    /* Silicon Core */

    VL53L1_Error VL53L1_is_firmware_ready_silicon(
      VL53L1_DEV      Dev,
      uint8_t        *pready);

    /* Wait */

    VL53L1_Error VL53L1_wait_for_boot_completion(
      VL53L1_DEV      Dev);




    VL53L1_Error VL53L1_wait_for_firmware_ready(
      VL53L1_DEV      Dev);




    VL53L1_Error VL53L1_wait_for_range_completion(
      VL53L1_DEV   Dev);




    VL53L1_Error VL53L1_wait_for_test_completion(
      VL53L1_DEV   Dev);






    VL53L1_Error VL53L1_is_boot_complete(
      VL53L1_DEV      Dev,
      uint8_t        *pready);



    VL53L1_Error VL53L1_is_firmware_ready(
      VL53L1_DEV      Dev,
      uint8_t        *pready);




    VL53L1_Error VL53L1_is_new_data_ready(
      VL53L1_DEV      Dev,
      uint8_t        *pready);






    VL53L1_Error VL53L1_poll_for_boot_completion(
      VL53L1_DEV      Dev,
      uint32_t        timeout_ms);




    VL53L1_Error VL53L1_poll_for_firmware_ready(
      VL53L1_DEV      Dev,
      uint32_t        timeout_ms);




    VL53L1_Error VL53L1_poll_for_range_completion(
      VL53L1_DEV   Dev,
      uint32_t     timeout_ms);


    /* Xtalk */

    VL53L1_Error VL53L1_xtalk_calibration_process_data(
      VL53L1_xtalk_range_results_t        *pxtalk_ranges,
      VL53L1_xtalk_histogram_data_t       *pxtalk_shape,
      VL53L1_xtalk_calibration_results_t  *pxtalk_cal);




    VL53L1_Error VL53L1_f_049(
      VL53L1_histogram_bin_data_t        *pavg_bins,
      VL53L1_xtalk_algo_data_t           *pdebug,
      VL53L1_xtalk_range_data_t          *pxtalk_data,
      uint8_t                             histogram__window_start,
      uint8_t                             histogram__window_end,
      VL53L1_xtalk_histogram_shape_t     *pxtalk_shape);



    VL53L1_Error VL53L1_f_047(
      VL53L1_xtalk_range_results_t  *pxtalk_results,
      VL53L1_xtalk_algo_data_t      *pdebug,
      int16_t                       *xgradient,
      int16_t                       *ygradient);




    VL53L1_Error VL53L1_f_048(
      VL53L1_xtalk_range_data_t *pxtalk_data,
      VL53L1_xtalk_algo_data_t  *pdebug,
      uint32_t                  *xtalk_mean_offset_kcps);




    VL53L1_Error VL53L1_f_053(
      VL53L1_histogram_bin_data_t    *phist_data,
      VL53L1_xtalk_range_data_t      *pxtalk_data,
      VL53L1_xtalk_algo_data_t       *pdebug,
      VL53L1_xtalk_histogram_shape_t *pxtalk_histo);





    VL53L1_Error VL53L1_f_040(
      uint32_t                       mean_offset,
      int16_t                        xgradient,
      int16_t                        ygradient,
      int8_t                         centre_offset_x,
      int8_t                         centre_offset_y,
      uint16_t                       roi_effective_spads,
      uint8_t                        roi_centre_spad,
      uint8_t                        roi_xy_size,
      uint32_t                      *xtalk_rate_kcps);




    VL53L1_Error VL53L1_f_041(
      VL53L1_histogram_bin_data_t    *phist_data,
      VL53L1_xtalk_histogram_shape_t *pxtalk_data,
      uint32_t                        xtalk_rate_kcps,
      VL53L1_histogram_bin_data_t    *pxtalkcount_data);




    VL53L1_Error VL53L1_f_055(
      VL53L1_histogram_bin_data_t   *phist_data,
      VL53L1_histogram_bin_data_t   *pxtalk_data,
      uint8_t                        xtalk_bin_offset);



    VL53L1_Error VL53L1_f_052(
      VL53L1_histogram_bin_data_t       *pxtalk_data,
      uint32_t                           amb_threshold,
      uint8_t                            VL53L1_p_022,
      uint8_t                            VL53L1_p_026);



    VL53L1_Error VL53L1_f_054(
      VL53L1_customer_nvm_managed_t *pcustomer,
      VL53L1_dynamic_config_t       *pdyn_cfg,
      VL53L1_xtalk_histogram_data_t *pxtalk_shape,
      VL53L1_histogram_bin_data_t   *pip_hist_data,
      VL53L1_histogram_bin_data_t   *pop_hist_data,
      VL53L1_histogram_bin_data_t   *pxtalk_count_data);




    VL53L1_Error VL53L1_f_051(
      uint8_t                      sigma_mult,
      int32_t                      VL53L1_p_004,
      uint32_t                    *ambient_noise);



    VL53L1_Error VL53L1_generate_dual_reflectance_xtalk_samples(
      VL53L1_xtalk_range_results_t *pxtalk_results,
      uint16_t                      expected_target_distance_mm,
      uint8_t                       higher_reflectance,
      VL53L1_histogram_bin_data_t  *pxtalk_avg_samples
    );





    VL53L1_Error VL53L1_f_050(
      VL53L1_histogram_bin_data_t   *pzone_avg_1,
      VL53L1_histogram_bin_data_t   *pzone_avg_2,
      uint16_t                             expected_target_distance,
      uint8_t                              subtract_amb,
      uint8_t                              higher_reflectance,
      VL53L1_histogram_bin_data_t     *pxtalk_output
    );


    /* Zone Presets */
    VL53L1_Error VL53L1_init_zone_config_structure(
      uint8_t x_off,
      uint8_t x_inc,
      uint8_t x_zones,
      uint8_t y_off,
      uint8_t y_inc,
      uint8_t y_zones,
      uint8_t width,
      uint8_t height,
      VL53L1_zone_config_t   *pdata);




    VL53L1_Error VL53L1_zone_preset_xtalk_planar(
      VL53L1_general_config_t *pgeneral,
      VL53L1_zone_config_t    *pzone_cfg);



    VL53L1_Error VL53L1_init_zone_config_histogram_bins(
      VL53L1_zone_config_t   *pdata);





    /* Write and read functions from I2C */

    VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data);
    VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data);
    VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data);
    VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data);
    VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data);
    VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data);
    VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData);

    VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);
    VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count);

    VL53L1_Error VL53L1_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
    VL53L1_Error VL53L1_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
    VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms);
    VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us);
    VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms);

    VL53L1_Error VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms);

  private:

    /**
     * @name Helper functions to control Xshutdown signal.
     *
     * The Xshutdown signal (XSHUT) is sometimes also referred to as GPIO0.
     * These functions are virtual in order to allow different ways for
     * controlling the signal.
     * For example using an output pin of the microcontroller or indirectly
     * using an IO expander or a shift register.
     */
    ///@{
    /**
     * Set XSHUT to low.
     */
    virtual void VL53L1_XshutSetLow() = 0;

    /**
     * Set XSHUT to high.
     */
    virtual void VL53L1_XshutSetHigh() = 0;

    /**
     * Initialize the control mechanism for XSHUT pin.
     */
    virtual void VL53L1_XshutDeinitialize() = 0;

    /**
     * Initialize the control mechanism for XSHUT pin.
     */
    virtual void VL53L1_XshutInitialize() = 0;
    ///@}

  protected:

    /* IO Device */
    TwoWire *dev_i2c;
    /* Device data */
    VL53L1_Dev_t MyDevice;
    VL53L1_DEV Dev;
};

/** Class representing a VL53L1 sensor component
 *
 * The Xshutdown pin (XSHUT / GPIO0) is controlled by a microcontroller
 * general purpose output pin.
 */
class VL53L1 : public VL53L1Base {
  public:
    /**
     * @param[in] i2c device I2C to be used for communication
     * @param[in] pin shutdown pin to be used as component GPIO0
     */
    VL53L1(TwoWire *const i2c, const int pin);

  protected:

    /**
     * Set pin controlling XSHUT to low.
     */
    virtual void VL53L1_XshutSetLow() override;

    /**
     * Set pin controlling XSHUT to high.
     */
    virtual void VL53L1_XshutSetHigh() override;

    /**
     * Set pin mode controlling XSHUT to input.
     */
    virtual void VL53L1_XshutDeinitialize() override;

    /**
     * Set pin mode controlling XSHUT to output.
     */
    virtual void VL53L1_XshutInitialize() override;

    /** Number of digital output pin controlling XSHUT. */
    int xshut;
};

#endif /* _VL53L1_CLASS_H_ */
