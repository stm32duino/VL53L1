
/*******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 Core and is dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

********************************************************************************

 'STMicroelectronics Proprietary license'

********************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document is strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


********************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones
 mentioned above :

********************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


********************************************************************************

*/


#include "vl53l1_class.h"


#define ZONE_CHECK VL53L1_MAX_USER_ZONES

#if ZONE_CHECK < 5
  #error Must define at least 5 zones in MAX_USER_ZONES constant
#endif


#ifndef MIN
  #define MIN(v1, v2) ((v1) < (v2) ? (v1) : (v2))
#endif
#ifndef MAX
  #define MAX(v1, v2) ((v1) < (v2) ? (v2) : (v1))
#endif

#define DMAX_REFLECTANCE_IDX 2



#define LOWPOWER_AUTO_VHV_LOOP_DURATION_US 245
#define LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING 1448
#define LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING 2100

#define FDA_MAX_TIMING_BUDGET_US 550000






static int32_t BDTable[VL53L1_TUNING_MAX_TUNABLE_KEY] = {
  TUNING_VERSION,
  TUNING_PROXY_MIN,
  TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM,
  TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER,
  TUNING_MIN_AMBIENT_DMAX_VALID,
  TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER,
  TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM,
  TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT,
  TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN,
  TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET,
  TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR_DEFAULT,
};


VL53L1_Error VL53L1::SingleTargetXTalkCalibration(VL53L1_DEV Dev)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;

  uint32_t sum_ranging = 0;
  uint32_t sum_spads = 0;
  FixPoint1616_t sum_signalRate = 0;
  FixPoint1616_t total_count = 0;
  uint8_t xtalk_meas = 0;
  uint8_t xtalk_measmax =
    BDTable[VL53L1_TUNING_SINGLE_TARGET_XTALK_SAMPLE_NUMBER];
  VL53L1_RangingMeasurementData_t RMData;
  FixPoint1616_t xTalkStoredMeanSignalRate;
  FixPoint1616_t xTalkStoredMeanRange;
  FixPoint1616_t xTalkStoredMeanRtnSpads;
  uint32_t xTalkStoredMeanRtnSpadsAsInt;
  uint32_t xTalkCalDistanceAsInt;
  FixPoint1616_t XTalkCompensationRateMegaCps;
  uint32_t signalXTalkTotalPerSpad;
  VL53L1_PresetModes PresetMode;
  VL53L1_CalibrationData_t  CalibrationData;
  VL53L1_CustomerNvmManaged_t *pC;





  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);

  if ((PresetMode != VL53L1_PRESETMODE_AUTONOMOUS) &&
      (PresetMode != VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS) &&
      (PresetMode != VL53L1_PRESETMODE_LITE_RANGING)) {
    Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;
    goto ENDFUNC;
  }


  Status = VL53L1_disable_xtalk_compensation(Dev);

  if (Status != VL53L1_ERROR_NONE) {
    goto ENDFUNC;
  }

  Status = VL53L1_StartMeasurement();

  if (Status != VL53L1_ERROR_NONE) {
    goto ENDFUNC;
  }


  VL53L1_WaitMeasurementDataReady();
  VL53L1_GetRangingMeasurementData(&RMData);
  VL53L1_ClearInterruptAndStartMeasurement();

  sum_ranging = 0;
  sum_spads = 0;
  sum_signalRate = 0;
  total_count = 0;
  for (xtalk_meas = 0; xtalk_meas < xtalk_measmax; xtalk_meas++) {
    VL53L1_WaitMeasurementDataReady();
    VL53L1_GetRangingMeasurementData(&RMData);
    VL53L1_ClearInterruptAndStartMeasurement();
    if (RMData.RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) {
      sum_ranging += RMData.RangeMilliMeter;
      sum_signalRate += RMData.SignalRateRtnMegaCps;
      sum_spads += RMData.EffectiveSpadRtnCount / 256;
      total_count++;
    }
  }
  Status = VL53L1_StopMeasurement();

  if (total_count > 0) {

    xTalkStoredMeanSignalRate = sum_signalRate / total_count;
    xTalkStoredMeanRange = (FixPoint1616_t)(sum_ranging << 16);
    xTalkStoredMeanRange /= total_count;
    xTalkStoredMeanRtnSpads = (FixPoint1616_t)(sum_spads << 16);
    xTalkStoredMeanRtnSpads /= total_count;


    xTalkStoredMeanRtnSpadsAsInt = (xTalkStoredMeanRtnSpads +
                                    0x8000) >> 16;


    xTalkCalDistanceAsInt = ((uint32_t)BDTable[
                            VL53L1_TUNING_SINGLE_TARGET_XTALK_TARGET_DISTANCE_MM]);
    if (xTalkStoredMeanRtnSpadsAsInt == 0 ||
        xTalkCalDistanceAsInt == 0 ||
        xTalkStoredMeanRange >= (xTalkCalDistanceAsInt << 16)) {
      XTalkCompensationRateMegaCps = 0;
    } else {

      signalXTalkTotalPerSpad = (xTalkStoredMeanSignalRate) /
                                xTalkStoredMeanRtnSpadsAsInt;


      signalXTalkTotalPerSpad *= (((uint32_t)1 << 16) -
                                  (xTalkStoredMeanRange / xTalkCalDistanceAsInt));


      XTalkCompensationRateMegaCps = (signalXTalkTotalPerSpad
                                      + 0x8000) >> 16;
    }


    Status = VL53L1_GetCalibrationData(&CalibrationData);

    if (Status != VL53L1_ERROR_NONE) {
      goto ENDFUNC;
    }

    pC = &CalibrationData.customer;

    pC->algo__crosstalk_compensation_plane_offset_kcps =
      (uint32_t)(1000 * ((XTalkCompensationRateMegaCps  +
                          ((uint32_t)1 << 6)) >> (16 - 9)));

    Status = VL53L1_SetCalibrationData(&CalibrationData);

    if (Status != VL53L1_ERROR_NONE) {
      goto ENDFUNC;
    }

    Status = VL53L1_enable_xtalk_compensation(Dev);

  } else

  {
    Status = VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL;
  }

ENDFUNC:

  return Status;

}


VL53L1_Error VL53L1::CheckValidRectRoi(VL53L1_UserRoi_t ROI)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;




  if ((ROI.TopLeftX > 15) || (ROI.TopLeftY > 15) ||
      (ROI.BotRightX > 15) || (ROI.BotRightY > 15)) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if ((ROI.TopLeftX > ROI.BotRightX) || (ROI.TopLeftY < ROI.BotRightY)) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }


  return Status;
}

VL53L1_GPIO_Interrupt_Mode VL53L1::ConvertModeToLLD(VL53L1_Error *pStatus, VL53L1_ThresholdMode CrossMode)
{
  VL53L1_GPIO_Interrupt_Mode Mode;

  switch (CrossMode) {
    case VL53L1_THRESHOLD_CROSSED_LOW:
      Mode = VL53L1_GPIOINTMODE_LEVEL_LOW;
      break;
    case VL53L1_THRESHOLD_CROSSED_HIGH:
      Mode = VL53L1_GPIOINTMODE_LEVEL_HIGH;
      break;
    case VL53L1_THRESHOLD_OUT_OF_WINDOW:
      Mode = VL53L1_GPIOINTMODE_OUT_OF_WINDOW;
      break;
    case VL53L1_THRESHOLD_IN_WINDOW:
      Mode = VL53L1_GPIOINTMODE_IN_WINDOW;
      break;
    default:

      Mode = VL53L1_GPIOINTMODE_LEVEL_HIGH;
      *pStatus = VL53L1_ERROR_INVALID_PARAMS;
  }
  return Mode;
}

VL53L1_ThresholdMode VL53L1::ConvertModeFromLLD(VL53L1_Error *pStatus, VL53L1_GPIO_Interrupt_Mode CrossMode)
{
  VL53L1_ThresholdMode Mode;

  switch (CrossMode) {
    case VL53L1_GPIOINTMODE_LEVEL_LOW:
      Mode = VL53L1_THRESHOLD_CROSSED_LOW;
      break;
    case VL53L1_GPIOINTMODE_LEVEL_HIGH:
      Mode = VL53L1_THRESHOLD_CROSSED_HIGH;
      break;
    case VL53L1_GPIOINTMODE_OUT_OF_WINDOW:
      Mode = VL53L1_THRESHOLD_OUT_OF_WINDOW;
      break;
    case VL53L1_GPIOINTMODE_IN_WINDOW:
      Mode = VL53L1_THRESHOLD_IN_WINDOW;
      break;
    default:

      Mode = VL53L1_THRESHOLD_CROSSED_HIGH;
      *pStatus = VL53L1_ERROR_UNDEFINED;
  }
  return Mode;
}



VL53L1_Error VL53L1::VL53L1_GetVersion(VL53L1_Version_t *pVersion)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  pVersion->major = VL53L1_IMPLEMENTATION_VER_MAJOR;
  pVersion->minor = VL53L1_IMPLEMENTATION_VER_MINOR;
  pVersion->build = VL53L1_IMPLEMENTATION_VER_SUB;

  pVersion->revision = VL53L1_IMPLEMENTATION_VER_REVISION;


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetProductRevision(uint8_t *pProductRevisionMajor, uint8_t *pProductRevisionMinor)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t revision_id;
  VL53L1_LLDriverData_t   *pLLData;



  pLLData =  VL53L1DevStructGetLLDriverHandle(Dev);
  revision_id = pLLData->nvm_copy_data.identification__revision_id;
  *pProductRevisionMajor = 1;
  *pProductRevisionMinor = (revision_id & 0xF0) >> 4;


  return Status;

}

VL53L1_Error VL53L1::VL53L1_GetDeviceInfo(VL53L1_DeviceInfo_t *pVL53L1_DeviceInfo)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t revision_id;
  VL53L1_LLDriverData_t   *pLLData;



  pLLData =  VL53L1DevStructGetLLDriverHandle(Dev);

  strncpy(pVL53L1_DeviceInfo->ProductId, "",
          VL53L1_DEVINFO_STRLEN - 1);
  pVL53L1_DeviceInfo->ProductType =
    pLLData->nvm_copy_data.identification__module_type;

  revision_id = pLLData->nvm_copy_data.identification__revision_id;
  pVL53L1_DeviceInfo->ProductRevisionMajor = 1;
  pVL53L1_DeviceInfo->ProductRevisionMinor = (revision_id & 0xF0) >> 4;

#ifndef VL53L1_USE_EMPTY_STRING
  if (pVL53L1_DeviceInfo->ProductRevisionMinor == 0)
    strncpy(pVL53L1_DeviceInfo->Name,
            VL53L1_STRING_DEVICE_INFO_NAME0,
            VL53L1_DEVINFO_STRLEN - 1);
  else
    strncpy(pVL53L1_DeviceInfo->Name,
            VL53L1_STRING_DEVICE_INFO_NAME1,
            VL53L1_DEVINFO_STRLEN - 1);
  strncpy(pVL53L1_DeviceInfo->Type,
          VL53L1_STRING_DEVICE_INFO_TYPE,
          VL53L1_DEVINFO_STRLEN - 1);

  if (pVL53L1_DeviceInfo->ProductType == 0xAA) {
    pVL53L1_DeviceInfo->Name[5] = '3';
    pVL53L1_DeviceInfo->Type[5] = '3';
  }
#else
  pVL53L1_DeviceInfo->Name[0] = 0;
  pVL53L1_DeviceInfo->Type[0] = 0;
#endif


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetUID(uint64_t *pUid)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t fmtdata[8];



  Status = VL53L1_read_nvm_raw_data(Dev,
                                    (uint8_t)(0x1F8 >> 2),
                                    (uint8_t)(8 >> 2),
                                    fmtdata);
  memcpy(pUid, fmtdata, sizeof(uint64_t));


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetRangeStatusString(uint8_t RangeStatus, char *pRangeStatusString)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_range_status_string(RangeStatus,
                                          pRangeStatusString);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetPalErrorString(VL53L1_Error PalErrorCode, char *pPalErrorString)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_pal_error_string(PalErrorCode, pPalErrorString);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetPalStateString(VL53L1_State PalStateCode, char *pPalStateString)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_pal_state_string(PalStateCode, pPalStateString);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetPalState(VL53L1_State *pPalState)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  *pPalState = VL53L1DevDataGet(Dev, PalState);


  return Status;
}


VL53L1_Error VL53L1::VL53L1_SetDeviceAddress(uint8_t DeviceAddress)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_WrByte(Dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS,
                         DeviceAddress / 2);

  if (Status == VL53L1_ERROR_NONE) {
    Dev->I2cDevAddr = DeviceAddress;
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_DataInit()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t i;
  VL53L1_LLDriverData_t *pdev;




#ifdef USE_I2C_2V8
  Status = VL53L1_RdByte(Dev, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, &i);
  if (Status == VL53L1_ERROR_NONE) {
    i = (i & 0xfe) | 0x01;
    Status = VL53L1_WrByte(Dev, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG,
                           i);
  }
#endif

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_data_init(Dev, 1);
  }

  if (Status == VL53L1_ERROR_NONE) {
    pdev = VL53L1DevStructGetLLDriverHandle(Dev);
    memset(&pdev->per_vcsel_cal_data, 0,
           sizeof(pdev->per_vcsel_cal_data));
  }

  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev, PalState, VL53L1_STATE_WAIT_STATICINIT);
    VL53L1DevDataSet(Dev, CurrentParameters.PresetMode,
                     VL53L1_PRESETMODE_RANGING);
  }


  for (i = 0; i < VL53L1_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
    if (Status == VL53L1_ERROR_NONE) {
      Status |= VL53L1_SetLimitCheckEnable(i, 1);
    } else {
      break;
    }

  }


  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_set_dmax_mode(Dev,
                                  VL53L1_DEVICEDMAXMODE__CUST_CAL_DATA);
  }



  return Status;
}


VL53L1_Error VL53L1::VL53L1_StaticInit()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t  measurement_mode;



  VL53L1DevDataSet(Dev, PalState, VL53L1_STATE_IDLE);

  measurement_mode  = VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;
  VL53L1DevDataSet(Dev, LLData.measurement_mode, measurement_mode);

  VL53L1DevDataSet(Dev, CurrentParameters.DistanceMode,
                   VL53L1_DISTANCEMODE_LONG);
  VL53L1DevDataSet(Dev, CurrentParameters.OutputMode,
                   VL53L1_OUTPUTMODE_NEAREST);

  return Status;
}

VL53L1_Error VL53L1::VL53L1_WaitDeviceBooted()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_poll_for_boot_completion(Dev,
                                           VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS);


  return Status;
}




VL53L1_Error VL53L1::ComputeDevicePresetMode(VL53L1_PresetModes PresetMode, VL53L1_DistanceModes DistanceMode, VL53L1_DevicePresetModes *pDevicePresetMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;

  uint8_t DistIdx;
  VL53L1_DevicePresetModes LightModes[3] = {
    VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE,
    VL53L1_DEVICEPRESETMODE_STANDARD_RANGING,
    VL53L1_DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE
  };

  VL53L1_DevicePresetModes RangingModes[3] = {
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE,
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE,
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE
  };

  VL53L1_DevicePresetModes ScanningModes[3] = {
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_SHORT_RANGE,
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE,
    VL53L1_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_LONG_RANGE
  };

  VL53L1_DevicePresetModes TimedModes[3] = {
    VL53L1_DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE,
    VL53L1_DEVICEPRESETMODE_TIMED_RANGING,
    VL53L1_DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE
  };

  VL53L1_DevicePresetModes LowPowerTimedModes[3] = {
    VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE,
    VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE,
    VL53L1_DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE
  };

  *pDevicePresetMode = VL53L1_DEVICEPRESETMODE_STANDARD_RANGING;

  switch (DistanceMode) {
    case VL53L1_DISTANCEMODE_SHORT:
      DistIdx = 0;
      break;
    case VL53L1_DISTANCEMODE_MEDIUM:
      DistIdx = 1;
      break;
    default:
      DistIdx = 2;
  }

  switch (PresetMode) {
    case VL53L1_PRESETMODE_LITE_RANGING:
      *pDevicePresetMode = LightModes[DistIdx];
      break;

    case VL53L1_PRESETMODE_RANGING:
      *pDevicePresetMode = RangingModes[DistIdx];
      break;

    case VL53L1_PRESETMODE_MULTIZONES_SCANNING:
      *pDevicePresetMode = ScanningModes[DistIdx];
      break;

    case VL53L1_PRESETMODE_AUTONOMOUS:
      *pDevicePresetMode = TimedModes[DistIdx];
      break;

    case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
      *pDevicePresetMode = LowPowerTimedModes[DistIdx];
      break;
    case VL53L1_PRESETMODE_OLT:
      *pDevicePresetMode = VL53L1_DEVICEPRESETMODE_OLT;
      break;
    case VL53L1_PRESETMODE_PROXY_RANGING_MODE:
      *pDevicePresetMode =
        VL53L1_DEVICEPRESETMODE_SPECIAL_HISTOGRAM_SHORT_RANGE;
      break;

    default:

      Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;
  }

  return Status;
}

VL53L1_Error VL53L1::SetPresetMode(VL53L1_DEV Dev, VL53L1_PresetModes PresetMode, VL53L1_DistanceModes DistanceMode, uint32_t inter_measurement_period_ms)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_DevicePresetModes   device_preset_mode;
  uint8_t measurement_mode;
  uint16_t dss_config__target_total_rate_mcps;
  uint32_t phasecal_config_timeout_us;
  uint32_t mm_config_timeout_us;
  uint32_t lld_range_config_timeout_us;

  if ((PresetMode == VL53L1_PRESETMODE_AUTONOMOUS) ||
      (PresetMode == VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS)) {
    measurement_mode  = VL53L1_DEVICEMEASUREMENTMODE_TIMED;
  } else {
    measurement_mode  = VL53L1_DEVICEMEASUREMENTMODE_BACKTOBACK;
  }


  Status = ComputeDevicePresetMode(PresetMode, DistanceMode,
                                   &device_preset_mode);

  if (Status == VL53L1_ERROR_NONE)
    Status =  VL53L1_get_preset_mode_timing_cfg(Dev,
                                                device_preset_mode,
                                                &dss_config__target_total_rate_mcps,
                                                &phasecal_config_timeout_us,
                                                &mm_config_timeout_us,
                                                &lld_range_config_timeout_us);

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_set_preset_mode(
               Dev,
               device_preset_mode,
               dss_config__target_total_rate_mcps,
               phasecal_config_timeout_us,
               mm_config_timeout_us,
               lld_range_config_timeout_us,
               inter_measurement_period_ms);

  if (Status == VL53L1_ERROR_NONE)
    VL53L1DevDataSet(Dev, LLData.measurement_mode,
                     measurement_mode);

  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev, CurrentParameters.PresetMode, PresetMode);
  }

  VL53L1DevDataSet(Dev, CurrentParameters.OutputMode,
                   VL53L1_OUTPUTMODE_NEAREST);

  return Status;
}


VL53L1_Error VL53L1::VL53L1_SetPresetMode(VL53L1_PresetModes PresetMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_DistanceModes DistanceMode = VL53L1_DISTANCEMODE_LONG;


  Status = VL53L1_low_power_auto_data_init(Dev);

  if (PresetMode == VL53L1_PRESETMODE_PROXY_RANGING_MODE) {
    DistanceMode = VL53L1_DISTANCEMODE_SHORT;
  }
  Status = SetPresetMode(Dev,
                         PresetMode,
                         DistanceMode,
                         1000);

  if (Status == VL53L1_ERROR_NONE) {
    if ((PresetMode == VL53L1_PRESETMODE_LITE_RANGING) ||
        (PresetMode == VL53L1_PRESETMODE_AUTONOMOUS) ||
        (PresetMode == VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS)) {
      Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(41000);
    } else

    {
      Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(33333);
    }
  }

  if (Status == VL53L1_ERROR_NONE) {

    Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(1000);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetPresetMode(VL53L1_PresetModes *pPresetMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  *pPresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetDistanceMode(VL53L1_DistanceModes DistanceMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_PresetModes PresetMode;
  uint32_t inter_measurement_period_ms;
  uint32_t TimingBudget;
  uint32_t MmTimeoutUs;
  uint32_t PhaseCalTimeoutUs;
  VL53L1_zone_config_t zone_config;

  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);



  if ((PresetMode == VL53L1_PRESETMODE_PROXY_RANGING_MODE) &&
      (DistanceMode != VL53L1_DISTANCEMODE_SHORT)) {
    return VL53L1_ERROR_INVALID_PARAMS;
  }
  if ((DistanceMode != VL53L1_DISTANCEMODE_SHORT) &&
      (DistanceMode != VL53L1_DISTANCEMODE_MEDIUM) &&
      (DistanceMode != VL53L1_DISTANCEMODE_LONG)) {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_get_zone_config(Dev, &zone_config);
  }

  inter_measurement_period_ms =  VL53L1DevDataGet(Dev,
                                                  LLData.inter_measurement_period_ms);

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_get_timeouts_us(Dev, &PhaseCalTimeoutUs,
                                    &MmTimeoutUs, &TimingBudget);

  if (Status == VL53L1_ERROR_NONE)
    Status = SetPresetMode(Dev,
                           PresetMode,
                           DistanceMode,
                           inter_measurement_period_ms);

  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev, CurrentParameters.DistanceMode,
                     DistanceMode);
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_set_timeouts_us(Dev, PhaseCalTimeoutUs,
                                    MmTimeoutUs, TimingBudget);

    if (Status == VL53L1_ERROR_NONE)
      VL53L1DevDataSet(Dev, LLData.range_config_timeout_us,
                       TimingBudget);
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_set_zone_config(Dev, &zone_config);
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetDistanceMode(VL53L1_DistanceModes *pDistanceMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  *pDistanceMode = VL53L1DevDataGet(Dev, CurrentParameters.DistanceMode);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetOutputMode(VL53L1_OutputModes OutputMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  if ((OutputMode != VL53L1_OUTPUTMODE_NEAREST) &&
      (OutputMode != VL53L1_OUTPUTMODE_STRONGEST)) {
    Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;
  } else {
    VL53L1DevDataSet(Dev, CurrentParameters.OutputMode, OutputMode);
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetOutputMode(VL53L1_OutputModes *pOutputMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  *pOutputMode = VL53L1DevDataGet(Dev, CurrentParameters.OutputMode);


  return Status;
}



VL53L1_Error VL53L1::VL53L1_SetMeasurementTimingBudgetMicroSeconds(uint32_t MeasurementTimingBudgetMicroSeconds)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t Mm1Enabled;
  uint8_t Mm2Enabled;
  uint32_t TimingGuard;
  uint32_t divisor;
  uint32_t TimingBudget;
  uint32_t MmTimeoutUs;
  VL53L1_PresetModes PresetMode;
  uint32_t PhaseCalTimeoutUs;
  uint32_t vhv;
  int32_t vhv_loops;
  uint32_t FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;




  if (MeasurementTimingBudgetMicroSeconds > 10000000) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM1, &Mm1Enabled);
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM2, &Mm2Enabled);
  }

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_get_timeouts_us(Dev,
                                    &PhaseCalTimeoutUs,
                                    &MmTimeoutUs,
                                    &TimingBudget);

  if (Status == VL53L1_ERROR_NONE) {
    PresetMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.PresetMode);

    TimingGuard = 0;
    divisor = 1;
    switch (PresetMode) {
      case VL53L1_PRESETMODE_LITE_RANGING:
        if ((Mm1Enabled == 1) || (Mm2Enabled == 1)) {
          TimingGuard = 5000;
        } else {
          TimingGuard = 1000;
        }
        break;

      case VL53L1_PRESETMODE_AUTONOMOUS:
        FDAMaxTimingBudgetUs *= 2;
        if ((Mm1Enabled == 1) || (Mm2Enabled == 1)) {
          TimingGuard = 26600;
        } else {
          TimingGuard = 21600;
        }
        divisor = 2;
        break;

      case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
        FDAMaxTimingBudgetUs *= 2;
        vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
        VL53L1_get_tuning_parm(Dev,
                               VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND,
                               &vhv_loops);
        if (vhv_loops > 0) {
          vhv += vhv_loops *
                 LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
        }
        TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
                      LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING +
                      vhv;
        divisor = 2;
        break;

      case VL53L1_PRESETMODE_RANGING:
      case VL53L1_PRESETMODE_MULTIZONES_SCANNING:
      case VL53L1_PRESETMODE_PROXY_RANGING_MODE:
        TimingGuard = 1700;
        divisor = 6;
        break;

      case VL53L1_PRESETMODE_OLT:
        TimingGuard = MmTimeoutUs + 5000;
        break;
      default:

        Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;
    }

    if (MeasurementTimingBudgetMicroSeconds <= TimingGuard) {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    } else {
      TimingBudget = (MeasurementTimingBudgetMicroSeconds
                      - TimingGuard);
    }

    if (Status == VL53L1_ERROR_NONE) {
      if (TimingBudget > FDAMaxTimingBudgetUs) {
        Status = VL53L1_ERROR_INVALID_PARAMS;
      } else {
        TimingBudget /= divisor;
        Status = VL53L1_set_timeouts_us(
                   Dev,
                   PhaseCalTimeoutUs,
                   MmTimeoutUs,
                   TimingBudget);
      }

      if (Status == VL53L1_ERROR_NONE)
        VL53L1DevDataSet(Dev,
                         LLData.range_config_timeout_us,
                         TimingBudget);
    }
  }
  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev,
                     CurrentParameters.MeasurementTimingBudgetMicroSeconds,
                     MeasurementTimingBudgetMicroSeconds);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetMeasurementTimingBudgetMicroSeconds(uint32_t *pMeasurementTimingBudgetMicroSeconds)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t Mm1Enabled = 0;
  uint8_t Mm2Enabled = 0;
  uint32_t  MmTimeoutUs = 0;
  uint32_t  RangeTimeoutUs = 0;
  uint32_t  MeasTimingBdg = 0;
  uint32_t PhaseCalTimeoutUs = 0;
  VL53L1_PresetModes PresetMode;
  uint32_t TimingGuard;
  uint32_t vhv;
  int32_t vhv_loops;



  *pMeasurementTimingBudgetMicroSeconds = 0;

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM1, &Mm1Enabled);
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_GetSequenceStepEnable(VL53L1_SEQUENCESTEP_MM2, &Mm2Enabled);
  }

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_get_timeouts_us(Dev,
                                    &PhaseCalTimeoutUs,
                                    &MmTimeoutUs,
                                    &RangeTimeoutUs);

  if (Status == VL53L1_ERROR_NONE) {
    PresetMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.PresetMode);

    switch (PresetMode) {
      case VL53L1_PRESETMODE_LITE_RANGING:
        if ((Mm1Enabled == 1) || (Mm2Enabled == 1)) {
          MeasTimingBdg = RangeTimeoutUs + 5000;
        } else {
          MeasTimingBdg = RangeTimeoutUs + 1000;
        }

        break;

      case VL53L1_PRESETMODE_AUTONOMOUS:
        if ((Mm1Enabled == 1) || (Mm2Enabled == 1)) {
          MeasTimingBdg = 2 * RangeTimeoutUs + 26600;
        } else {
          MeasTimingBdg = 2 * RangeTimeoutUs + 21600;
        }

        break;

      case VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS:
        vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
        VL53L1_get_tuning_parm(Dev,
                               VL53L1_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND,
                               &vhv_loops);
        if (vhv_loops > 0) {
          vhv += vhv_loops *
                 LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
        }
        TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
                      LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING +
                      vhv;
        MeasTimingBdg = 2 * RangeTimeoutUs + TimingGuard;
        break;

      case VL53L1_PRESETMODE_RANGING:
      case VL53L1_PRESETMODE_MULTIZONES_SCANNING:
      case VL53L1_PRESETMODE_PROXY_RANGING_MODE:
        MeasTimingBdg = (6 * RangeTimeoutUs) + 1700;
        break;

      case VL53L1_PRESETMODE_OLT:
        MeasTimingBdg = RangeTimeoutUs + MmTimeoutUs + 5000;
        break;
      default:

        Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;
    }
  }
  if (Status == VL53L1_ERROR_NONE) {
    *pMeasurementTimingBudgetMicroSeconds = MeasTimingBdg;
  }


  return Status;
}



VL53L1_Error VL53L1::VL53L1_SetInterMeasurementPeriodMilliSeconds(uint32_t InterMeasurementPeriodMilliSeconds)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint32_t adjustedIMP;




  adjustedIMP = InterMeasurementPeriodMilliSeconds;
  adjustedIMP += (adjustedIMP * 64) / 1000;

  Status = VL53L1_set_inter_measurement_period_ms(Dev,
                                                  adjustedIMP);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetInterMeasurementPeriodMilliSeconds(uint32_t *pInterMeasurementPeriodMilliSeconds)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint32_t adjustedIMP;



  Status = VL53L1_get_inter_measurement_period_ms(Dev, &adjustedIMP);

  adjustedIMP -= (adjustedIMP * 64) / 1000;
  *pInterMeasurementPeriodMilliSeconds = adjustedIMP;



  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetDmaxReflectance(FixPoint1616_t DmaxReflectance)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_dmax_reflectance_array_t dmax_reflectances;



  if (DmaxReflectance > 100 * 65536) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_get_dmax_reflectance_values(Dev,
                                                &dmax_reflectances);

  if (Status == VL53L1_ERROR_NONE) {
    dmax_reflectances.target_reflectance_for_dmax[
    DMAX_REFLECTANCE_IDX] =
        VL53L1_FIXPOINT1616TOFIXPOINT72(DmaxReflectance);
    Status = VL53L1_set_dmax_reflectance_values(Dev,
                                                &dmax_reflectances);
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetDmaxReflectance(FixPoint1616_t *pDmaxReflectance)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_dmax_reflectance_array_t dmax_reflectances;
  uint16_t r;


  Status = VL53L1_get_dmax_reflectance_values(Dev, &dmax_reflectances);
  if (Status == VL53L1_ERROR_NONE) {
    r = dmax_reflectances.target_reflectance_for_dmax[
       DMAX_REFLECTANCE_IDX];
    *pDmaxReflectance = VL53L1_FIXPOINT72TOFIXPOINT1616(r);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_SetDmaxMode(VL53L1_DeviceDmaxModes DmaxMode)
{

  VL53L1_Error  Status = VL53L1_ERROR_NONE;
  VL53L1_DeviceDmaxMode dmax_mode;



  switch (DmaxMode) {
    case VL53L1_DMAXMODE_FMT_CAL_DATA:
      dmax_mode = VL53L1_DEVICEDMAXMODE__FMT_CAL_DATA;
      break;
    case VL53L1_DMAXMODE_CUSTCAL_DATA:
      dmax_mode = VL53L1_DEVICEDMAXMODE__CUST_CAL_DATA;
      break;
    case VL53L1_DMAXMODE_PER_ZONE_CAL_DATA:
      dmax_mode = VL53L1_DEVICEDMAXMODE__PER_ZONE_CAL_DATA;
      break;
    default:
      Status = VL53L1_ERROR_INVALID_PARAMS;
      break;
  }
  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_set_dmax_mode(Dev, dmax_mode);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetDmaxMode(VL53L1_DeviceDmaxModes *pDmaxMode)
{
  VL53L1_Error  Status = VL53L1_ERROR_NONE;
  VL53L1_DeviceDmaxMode dmax_mode;



  Status = VL53L1_get_dmax_mode(Dev, &dmax_mode);
  if (Status == VL53L1_ERROR_NONE) {
    switch (dmax_mode) {
      case VL53L1_DEVICEDMAXMODE__FMT_CAL_DATA:
        *pDmaxMode = VL53L1_DMAXMODE_FMT_CAL_DATA;
        break;
      case VL53L1_DEVICEDMAXMODE__CUST_CAL_DATA:
        *pDmaxMode = VL53L1_DMAXMODE_CUSTCAL_DATA;
        break;
      case VL53L1_DEVICEDMAXMODE__PER_ZONE_CAL_DATA:
        *pDmaxMode = VL53L1_DMAXMODE_PER_ZONE_CAL_DATA;
        break;
      default:
        /* panic we didn't expect that from lld */
        *pDmaxMode = VL53L1_DMAXMODE_CUSTCAL_DATA;
        Status = VL53L1_ERROR_NOT_IMPLEMENTED;
        break;
    }
  }


  return Status;
}






VL53L1_Error VL53L1::VL53L1_GetNumberOfLimitCheck(uint16_t *pNumberOfLimitCheck)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  *pNumberOfLimitCheck = VL53L1_CHECKENABLE_NUMBER_OF_CHECKS;


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetLimitCheckInfo(uint16_t LimitCheckId, char *pLimitCheckString)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_limit_check_info(LimitCheckId,
                                       pLimitCheckString);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetLimitCheckStatus(uint16_t LimitCheckId, uint8_t *pLimitCheckStatus)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t Temp8;



  if (LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  } else {
    VL53L1_GETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                                  LimitCheckId, Temp8);
    *pLimitCheckStatus = Temp8;
  }


  return Status;
}

VL53L1_Error VL53L1::SetLimitValue(VL53L1_DEV Dev, uint16_t LimitCheckId, FixPoint1616_t value)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint16_t tmpuint16;



  switch (LimitCheckId) {
    case VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE:
      tmpuint16 = VL53L1_FIXPOINT1616TOFIXPOINT142(value);
      VL53L1_set_lite_sigma_threshold(Dev, tmpuint16);
      break;
    case VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      tmpuint16 = VL53L1_FIXPOINT1616TOFIXPOINT97(value);
      VL53L1_set_lite_min_count_rate(Dev, tmpuint16);
      break;
    default:
      Status = VL53L1_ERROR_INVALID_PARAMS;
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_SetLimitCheckEnable(uint16_t LimitCheckId, uint8_t LimitCheckEnable)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  FixPoint1616_t TempFix1616 = 0;




  if (LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  } else {

    if (LimitCheckEnable == 0) {
      TempFix1616 = 0;
    } else
      VL53L1_GETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                                    LimitCheckId, TempFix1616);

    Status = SetLimitValue(Dev, LimitCheckId, TempFix1616);
  }

  if (Status == VL53L1_ERROR_NONE)
    VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                  LimitChecksEnable,
                                  LimitCheckId,
                                  ((LimitCheckEnable == 0) ? 0 : 1));




  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetLimitCheckEnable(uint16_t LimitCheckId, uint8_t *pLimitCheckEnable)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t Temp8;



  if (LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
    *pLimitCheckEnable = 0;
  } else {
    VL53L1_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                                  LimitCheckId, Temp8);
    *pLimitCheckEnable = Temp8;
  }



  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetLimitCheckValue(uint16_t LimitCheckId, FixPoint1616_t LimitCheckValue)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t LimitChecksEnable;



  if (LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  } else {

    VL53L1_GETARRAYPARAMETERFIELD(Dev, LimitChecksEnable,
                                  LimitCheckId,
                                  LimitChecksEnable);

    if (LimitChecksEnable == 0) {

      VL53L1_SETARRAYPARAMETERFIELD(Dev, LimitChecksValue,
                                    LimitCheckId, LimitCheckValue);
    } else {

      Status = SetLimitValue(Dev, LimitCheckId,
                             LimitCheckValue);

      if (Status == VL53L1_ERROR_NONE) {
        VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                      LimitChecksValue,
                                      LimitCheckId, LimitCheckValue);
      }
    }
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetLimitCheckValue(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckValue)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint16_t MinCountRate;
  FixPoint1616_t TempFix1616;
  uint16_t SigmaThresh;



  switch (LimitCheckId) {
    case VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE:
      Status = VL53L1_get_lite_sigma_threshold(Dev, &SigmaThresh);
      TempFix1616 = VL53L1_FIXPOINT142TOFIXPOINT1616(SigmaThresh);
      break;
    case VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
      Status = VL53L1_get_lite_min_count_rate(Dev, &MinCountRate);
      TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(MinCountRate);
      break;
    default:
      Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE) {

    if (TempFix1616 == 0) {

      VL53L1_GETARRAYPARAMETERFIELD(Dev,
                                    LimitChecksValue, LimitCheckId,
                                    TempFix1616);
      *pLimitCheckValue = TempFix1616;
      VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                    LimitChecksEnable, LimitCheckId, 0);
    } else {
      *pLimitCheckValue = TempFix1616;
      VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                    LimitChecksValue, LimitCheckId,
                                    TempFix1616);
      VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                    LimitChecksEnable, LimitCheckId, 1);
    }
  }

  return Status;

}

VL53L1_Error VL53L1::VL53L1_GetLimitCheckCurrent(uint16_t LimitCheckId, FixPoint1616_t *pLimitCheckCurrent)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  FixPoint1616_t TempFix1616 = 0;



  if (LimitCheckId >= VL53L1_CHECKENABLE_NUMBER_OF_CHECKS) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  } else {
    VL53L1_GETARRAYPARAMETERFIELD(Dev, LimitChecksCurrent,
                                  LimitCheckId, TempFix1616);
    *pLimitCheckCurrent = TempFix1616;
  }


  return Status;

}








VL53L1_Error VL53L1::VL53L1_GetMaxNumberOfROI(uint8_t *pMaxNumberOfROI)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_PresetModes PresetMode;



  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);


  if (PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) {
    *pMaxNumberOfROI = VL53L1_MAX_USER_ZONES;
  } else {
    *pMaxNumberOfROI = 1;
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetROI(VL53L1_RoiConfig_t *pRoiConfig)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_PresetModes PresetMode;
  uint8_t MaxNumberOfROI = 1;
  VL53L1_zone_config_t  zone_cfg;
  VL53L1_UserRoi_t CurrROI;
  uint8_t  i;
  uint8_t  x_centre;
  uint8_t  y_centre;
  uint8_t  width, height;




  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);


  if (PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) {
    MaxNumberOfROI = VL53L1_MAX_USER_ZONES;
  }

  if ((pRoiConfig->NumberOfRoi > MaxNumberOfROI) ||
      (pRoiConfig->NumberOfRoi < 1)) {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE) {


    zone_cfg.max_zones = MaxNumberOfROI;
    zone_cfg.active_zones = pRoiConfig->NumberOfRoi - 1;

    for (i = 0; i < pRoiConfig->NumberOfRoi; i++) {
      CurrROI = pRoiConfig->UserRois[i];

      Status = CheckValidRectRoi(CurrROI);
      if (Status != VL53L1_ERROR_NONE) {
        break;
      }

      x_centre = (CurrROI.BotRightX + CurrROI.TopLeftX  + 1)
                 / 2;
      y_centre = (CurrROI.TopLeftY  + CurrROI.BotRightY + 1)
                 / 2;
      width = (CurrROI.BotRightX - CurrROI.TopLeftX);
      height = (CurrROI.TopLeftY  - CurrROI.BotRightY);
      if ((width < 3) || (height < 3)) {
        Status = VL53L1_ERROR_INVALID_PARAMS;
        break;
      }
      zone_cfg.user_zones[i].x_centre = x_centre;
      zone_cfg.user_zones[i].y_centre = y_centre;
      zone_cfg.user_zones[i].width = width;
      zone_cfg.user_zones[i].height = height;
    }
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_set_zone_config(Dev, &zone_cfg);
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetROI(VL53L1_RoiConfig_t *pRoiConfig)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_zone_config_t      zone_cfg;
  uint8_t  i;
  uint8_t  TopLeftX;
  uint8_t  TopLeftY;
  uint8_t  BotRightX;
  uint8_t  BotRightY;



  VL53L1_get_zone_config(Dev, &zone_cfg);

  pRoiConfig->NumberOfRoi = zone_cfg.active_zones + 1;

  for (i = 0; i < pRoiConfig->NumberOfRoi; i++) {
    TopLeftX = (2 * zone_cfg.user_zones[i].x_centre -
                zone_cfg.user_zones[i].width) >> 1;
    TopLeftY = (2 * zone_cfg.user_zones[i].y_centre +
                zone_cfg.user_zones[i].height) >> 1;
    BotRightX = (2 * zone_cfg.user_zones[i].x_centre +
                 zone_cfg.user_zones[i].width) >> 1;
    BotRightY = (2 * zone_cfg.user_zones[i].y_centre -
                 zone_cfg.user_zones[i].height) >> 1;
    pRoiConfig->UserRois[i].TopLeftX = TopLeftX;
    pRoiConfig->UserRois[i].TopLeftY = TopLeftY;
    pRoiConfig->UserRois[i].BotRightX = BotRightX;
    pRoiConfig->UserRois[i].BotRightY = BotRightY;
  }


  return Status;
}







VL53L1_Error VL53L1::VL53L1_GetNumberOfSequenceSteps(uint8_t *pNumberOfSequenceSteps)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;

  SUPPRESS_UNUSED_WARNING(Dev);



  *pNumberOfSequenceSteps = VL53L1_SEQUENCESTEP_NUMBER_OF_ITEMS;


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetSequenceStepsInfo(VL53L1_SequenceStepId SequenceStepId, char *pSequenceStepsString)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_sequence_steps_info(
             SequenceStepId,
             pSequenceStepsString);



  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetSequenceStepEnable(VL53L1_SequenceStepId SequenceStepId, uint8_t SequenceStepEnabled)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint32_t MeasurementTimingBudgetMicroSeconds;





  Status = VL53L1_set_sequence_config_bit(Dev,
                                          (VL53L1_DeviceSequenceConfig)SequenceStepId,
                                          SequenceStepEnabled);


  if (Status == VL53L1_ERROR_NONE) {


    MeasurementTimingBudgetMicroSeconds = VL53L1DevDataGet(Dev,
                                                           CurrentParameters.MeasurementTimingBudgetMicroSeconds);

    VL53L1_SetMeasurementTimingBudgetMicroSeconds(MeasurementTimingBudgetMicroSeconds);
  }



  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetSequenceStepEnable(VL53L1_SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_sequence_config_bit(Dev,
                                          (VL53L1_DeviceSequenceConfig)SequenceStepId,
                                          pSequenceStepEnabled);


  return Status;
}










VL53L1_Error VL53L1::VL53L1_StartMeasurement()
{
#define TIMED_MODE_TIMING_GUARD_MILLISECONDS 4
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t DeviceMeasurementMode;
  VL53L1_State CurrPalState;
  VL53L1_Error lStatus;
  uint32_t MTBus, IMPms;


  VL53L1_load_patch(Dev);
  CurrPalState = VL53L1DevDataGet(Dev, PalState);
  switch (CurrPalState) {
    case VL53L1_STATE_IDLE:
      Status = VL53L1_ERROR_NONE;
      break;
    case VL53L1_STATE_POWERDOWN:
    case VL53L1_STATE_WAIT_STATICINIT:
    case VL53L1_STATE_STANDBY:
    case VL53L1_STATE_RUNNING:
    case VL53L1_STATE_RESET:
    case VL53L1_STATE_UNKNOWN:
    case VL53L1_STATE_ERROR:
      Status = VL53L1_ERROR_INVALID_COMMAND;
      break;
    default:
      Status = VL53L1_ERROR_UNDEFINED;
  }

  DeviceMeasurementMode = VL53L1DevDataGet(Dev, LLData.measurement_mode);


  if ((Status == VL53L1_ERROR_NONE) &&
      (DeviceMeasurementMode == VL53L1_DEVICEMEASUREMENTMODE_TIMED)) {
    lStatus = VL53L1_GetMeasurementTimingBudgetMicroSeconds(&MTBus);

    MTBus /= 1000;
    lStatus = VL53L1_GetInterMeasurementPeriodMilliSeconds(&IMPms);

    SUPPRESS_UNUSED_WARNING(lStatus);
    if (IMPms < MTBus + TIMED_MODE_TIMING_GUARD_MILLISECONDS) {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    }
  }

  if (Status == VL53L1_ERROR_NONE)
    Status = VL53L1_init_and_start_range(
               Dev,
               DeviceMeasurementMode,
               VL53L1_DEVICECONFIGLEVEL_FULL);


  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev, PalState, VL53L1_STATE_RUNNING);
  }



  return Status;
}

VL53L1_Error VL53L1::VL53L1_StopMeasurement()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_stop_range(Dev);
  VL53L1_unload_patch(Dev);

  if (Status == VL53L1_ERROR_NONE) {
    VL53L1DevDataSet(Dev, PalState, VL53L1_STATE_IDLE);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_ClearInterruptAndStartMeasurement()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t DeviceMeasurementMode;



  DeviceMeasurementMode = VL53L1DevDataGet(Dev, LLData.measurement_mode);

  Status = VL53L1_clear_interrupt_and_enable_next_range(Dev,
                                                        DeviceMeasurementMode);


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetMeasurementDataReady(uint8_t *pMeasurementDataReady)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_is_new_data_ready(Dev, pMeasurementDataReady);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_WaitMeasurementDataReady()
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;





  Status = VL53L1_poll_for_range_completion(Dev,
                                            VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS);


  return Status;
}

void VL53L1::GenNewPresetMode(int16_t RefRange, VL53L1_DistanceModes InternalDistanceMode, VL53L1_DistanceModes *pNewDistanceMode)
{
  uint16_t HRLI = 600;
  uint16_t HRLH = 700;
  uint16_t MRLI = 1400;
  uint16_t MRLH = 1500;

  switch (InternalDistanceMode) {
    case VL53L1_DISTANCEMODE_SHORT:

      if (RefRange > MRLH) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_LONG;
      } else if (RefRange > HRLH) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_MEDIUM;
      }
      break;
    case VL53L1_DISTANCEMODE_MEDIUM:

      if (RefRange > MRLH) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_LONG;
      } else if (RefRange < HRLI) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_SHORT;
      }
      break;
    default:

      if (RefRange < HRLI) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_SHORT;
      } else if (RefRange < MRLI) {
        *pNewDistanceMode = VL53L1_DISTANCEMODE_MEDIUM;
      }
      break;
  }
}

void VL53L1::CheckAndChangeDistanceMode(VL53L1_DEV Dev, VL53L1_TargetRangeData_t *pRangeData, int16_t Ambient100DmaxMm, VL53L1_DistanceModes *pNewDistanceMode)
{
  VL53L1_DistanceModes DistanceMode;
  uint8_t RangeStatus = pRangeData->RangeStatus;
  uint8_t DmaxValid;
  int32_t MinAmbient = BDTable[VL53L1_TUNING_MIN_AMBIENT_DMAX_VALID];
  VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);
  int32_t  tmpint32;


  switch (RangeStatus) {
    case VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL:
    case VL53L1_RANGESTATUS_WRAP_TARGET_FAIL:
    case VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE:
    case VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL:
    case VL53L1_RANGESTATUS_SYNCRONISATION_INT:
    case VL53L1_RANGESTATUS_NONE:
      return;
    default:

      break;
  }

  DmaxValid = 1;
  tmpint32 = pdev->hist_data.VL53L1_p_004;
  if ((tmpint32 < MinAmbient) || (Ambient100DmaxMm == 0)) {
    DmaxValid = 0;
  }

  DistanceMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.DistanceMode);

  *pNewDistanceMode = DistanceMode;

  if (RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID)
    GenNewPresetMode(pRangeData->RangeMilliMeter,
                     DistanceMode, pNewDistanceMode);
  else {
    if (DmaxValid)
      GenNewPresetMode(Ambient100DmaxMm,
                       DistanceMode, pNewDistanceMode);
    else {
      *pNewDistanceMode = VL53L1_DISTANCEMODE_LONG;
    }
  }
}

uint8_t VL53L1::ComputeRQL(uint8_t active_results, uint8_t FilteredRangeStatus, VL53L1_range_data_t *presults_data)
{
  int16_t T_Wide = 150;
  int16_t SRL = 300;
  uint16_t SRAS = 30;
  FixPoint1616_t RAS;
  FixPoint1616_t SRQL;
  FixPoint1616_t GI =   7713587;
  FixPoint1616_t GGm =  3198157;
  FixPoint1616_t LRAP = 6554;
  FixPoint1616_t partial;
  uint8_t finalvalue;
  uint8_t returnvalue;

  if (active_results == 0) {
    returnvalue = 0;
  } else if (((presults_data->max_range_mm -
               presults_data->min_range_mm) >= T_Wide) ||
             (FilteredRangeStatus == VL53L1_DEVICEERROR_PHASECONSISTENCY)) {
    returnvalue = 50;
  } else {
    if (presults_data->median_range_mm < SRL) {
      RAS = SRAS * 65536;
    } else {
      RAS = LRAP * presults_data->median_range_mm;
    }


    if (RAS != 0) {
      partial = (GGm * presults_data->VL53L1_p_005);
      partial = partial + (RAS >> 1);
      partial = partial / RAS;
      partial = partial * 65536;
      if (partial <= GI) {
        SRQL = GI - partial;
      } else {
        SRQL = 50 * 65536;
      }
    } else {
      SRQL = 100 * 65536;
    }

    finalvalue = (uint8_t)(SRQL >> 16);
    returnvalue = MAX(50, MIN(100, finalvalue));
  }

  return returnvalue;
}


uint8_t VL53L1::ConvertStatusLite(uint8_t FilteredRangeStatus)
{
  uint8_t RangeStatus;

  switch (FilteredRangeStatus) {
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      RangeStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      RangeStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      RangeStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      RangeStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      RangeStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      RangeStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_MINCLIP:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID;
      break;
    default:
      RangeStatus = VL53L1_RANGESTATUS_NONE;
  }

  return RangeStatus;
}


uint8_t VL53L1::ConvertStatusHisto(uint8_t FilteredRangeStatus)
{
  uint8_t RangeStatus;

  switch (FilteredRangeStatus) {
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      RangeStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      RangeStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      RangeStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_PREV_RANGE_NO_TARGETS:
      RangeStatus = VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL;
      break;
    case VL53L1_DEVICEERROR_EVENTCONSISTENCY:
      RangeStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:
      RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID;
      break;
    default:
      RangeStatus = VL53L1_RANGESTATUS_NONE;
  }

  return RangeStatus;
}

VL53L1_Error VL53L1::SetSimpleData(VL53L1_DEV Dev, uint8_t active_results, uint8_t device_status, VL53L1_range_data_t *presults_data, VL53L1_RangingMeasurementData_t *pRangeData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t FilteredRangeStatus;
  uint8_t SigmaLimitflag;
  uint8_t SignalLimitflag;
  uint8_t Temp8Enable;
  uint8_t Temp8;
  FixPoint1616_t AmbientRate;
  FixPoint1616_t SignalRate;
  FixPoint1616_t TempFix1616;
  FixPoint1616_t LimitCheckValue;
  VL53L1_PresetModes PresetMode;
  int16_t Range;

  pRangeData->TimeStamp = presults_data->time_stamp;

  FilteredRangeStatus = presults_data->range_status & 0x1F;

  pRangeData->RangeQualityLevel = ComputeRQL(active_results,
                                             FilteredRangeStatus,
                                             presults_data);

  SignalRate = VL53L1_FIXPOINT97TOFIXPOINT1616(
                 presults_data->peak_signal_count_rate_mcps);
  pRangeData->SignalRateRtnMegaCps
    = SignalRate;

  AmbientRate = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->ambient_count_rate_mcps);
  pRangeData->AmbientRateRtnMegaCps = AmbientRate;

  pRangeData->EffectiveSpadRtnCount =
    presults_data->VL53L1_p_006;

  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->VL53L1_p_005);

  pRangeData->SigmaMilliMeter = TempFix1616;

  pRangeData->RangeMilliMeter = presults_data->median_range_mm;

  pRangeData->RangeFractionalPart = 0;


  switch (device_status) {
    case VL53L1_DEVICEERROR_MULTCLIPFAIL:
    case VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53L1_DEVICEERROR_NOVHVVALUEFOUND:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_HARDWARE_FAIL;
      break;
    case VL53L1_DEVICEERROR_USERROICLIP:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
      break;
    default:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID;
  }


  if (pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) {
    PresetMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.PresetMode);
    if ((PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) ||
        (PresetMode == VL53L1_PRESETMODE_RANGING) ||
        (PresetMode == VL53L1_PRESETMODE_PROXY_RANGING_MODE))
      pRangeData->RangeStatus =
        ConvertStatusHisto(FilteredRangeStatus);
    else
      pRangeData->RangeStatus =
        ConvertStatusLite(FilteredRangeStatus);
  }


  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->VL53L1_p_005);
  VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                LimitChecksCurrent, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                                TempFix1616);

  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->peak_signal_count_rate_mcps);
  VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                LimitChecksCurrent, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                TempFix1616);



  VL53L1_GetLimitCheckValue(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                            &LimitCheckValue);

  SigmaLimitflag = (FilteredRangeStatus ==
                    VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK)
                   ? 1 : 0;

  VL53L1_GetLimitCheckEnable(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                             &Temp8Enable);

  Temp8 = ((Temp8Enable == 1) && (SigmaLimitflag == 1)) ? 1 : 0;
  VL53L1_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                                VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);


  VL53L1_GetLimitCheckValue(VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                            &LimitCheckValue);

  SignalLimitflag = (FilteredRangeStatus ==
                     VL53L1_DEVICEERROR_MSRCNOTARGET)
                    ? 1 : 0;

  VL53L1_GetLimitCheckEnable(VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                             &Temp8Enable);

  Temp8 = ((Temp8Enable == 1) && (SignalLimitflag == 1)) ? 1 : 0;
  VL53L1_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                                VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Temp8);

  Range = pRangeData->RangeMilliMeter;
  if ((pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) &&
      (Range < 0)) {
    if (Range < BDTable[VL53L1_TUNING_PROXY_MIN])
      pRangeData->RangeStatus =
        VL53L1_RANGESTATUS_RANGE_INVALID;
    else {
      pRangeData->RangeMilliMeter = 0;
    }
  }

  return Status;
}

VL53L1_Error VL53L1::SetTargetData(VL53L1_DEV Dev, uint8_t active_results, uint8_t device_status, VL53L1_range_data_t *presults_data, VL53L1_TargetRangeData_t *pRangeData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t FilteredRangeStatus;
  uint8_t SigmaLimitflag;
  uint8_t SignalLimitflag;
  uint8_t Temp8Enable;
  uint8_t Temp8;
  FixPoint1616_t AmbientRate;
  FixPoint1616_t SignalRate;
  FixPoint1616_t TempFix1616;
  FixPoint1616_t LimitCheckValue;
  VL53L1_PresetModes PresetMode;
  int16_t Range;

  FilteredRangeStatus = presults_data->range_status & 0x1F;

  pRangeData->RangeQualityLevel = ComputeRQL(active_results,
                                             FilteredRangeStatus,
                                             presults_data);

  SignalRate = VL53L1_FIXPOINT97TOFIXPOINT1616(
                 presults_data->peak_signal_count_rate_mcps);
  pRangeData->SignalRateRtnMegaCps
    = SignalRate;

  AmbientRate = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->ambient_count_rate_mcps);
  pRangeData->AmbientRateRtnMegaCps = AmbientRate;

  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->VL53L1_p_005);

  pRangeData->SigmaMilliMeter = TempFix1616;

  pRangeData->RangeMilliMeter = presults_data->median_range_mm;
  pRangeData->RangeMaxMilliMeter = presults_data->max_range_mm;
  pRangeData->RangeMinMilliMeter = presults_data->min_range_mm;

  pRangeData->RangeFractionalPart = 0;


  switch (device_status) {
    case VL53L1_DEVICEERROR_MULTCLIPFAIL:
    case VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
    case VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
    case VL53L1_DEVICEERROR_NOVHVVALUEFOUND:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_HARDWARE_FAIL;
      break;
    case VL53L1_DEVICEERROR_USERROICLIP:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
      break;
    default:
      pRangeData->RangeStatus = VL53L1_RANGESTATUS_RANGE_VALID;
  }


  if ((pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) &&
      (active_results == 0)) {
    pRangeData->RangeStatus = VL53L1_RANGESTATUS_NONE;
    pRangeData->SignalRateRtnMegaCps = 0;
    pRangeData->SigmaMilliMeter = 0;
    pRangeData->RangeMilliMeter = 8191;
    pRangeData->RangeMaxMilliMeter = 8191;
    pRangeData->RangeMinMilliMeter = 8191;
  }


  if (pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) {
    PresetMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.PresetMode);
    if ((PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) ||
        (PresetMode == VL53L1_PRESETMODE_RANGING) ||
        (PresetMode == VL53L1_PRESETMODE_PROXY_RANGING_MODE))
      pRangeData->RangeStatus =
        ConvertStatusHisto(FilteredRangeStatus);
    else
      pRangeData->RangeStatus =
        ConvertStatusLite(FilteredRangeStatus);
  }


  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->VL53L1_p_005);
  VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                LimitChecksCurrent, VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                                TempFix1616);

  TempFix1616 = VL53L1_FIXPOINT97TOFIXPOINT1616(
                  presults_data->peak_signal_count_rate_mcps);
  VL53L1_SETARRAYPARAMETERFIELD(Dev,
                                LimitChecksCurrent, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                TempFix1616);



  VL53L1_GetLimitCheckValue(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                            &LimitCheckValue);

  SigmaLimitflag = (FilteredRangeStatus ==
                    VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK)
                   ? 1 : 0;

  VL53L1_GetLimitCheckEnable(VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE,
                             &Temp8Enable);

  Temp8 = ((Temp8Enable == 1) && (SigmaLimitflag == 1)) ? 1 : 0;
  VL53L1_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                                VL53L1_CHECKENABLE_SIGMA_FINAL_RANGE, Temp8);


  VL53L1_GetLimitCheckValue(VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                            &LimitCheckValue);

  SignalLimitflag = (FilteredRangeStatus ==
                     VL53L1_DEVICEERROR_MSRCNOTARGET)
                    ? 1 : 0;

  VL53L1_GetLimitCheckEnable(VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                             &Temp8Enable);

  Temp8 = ((Temp8Enable == 1) && (SignalLimitflag == 1)) ? 1 : 0;
  VL53L1_SETARRAYPARAMETERFIELD(Dev, LimitChecksStatus,
                                VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Temp8);

  Range = pRangeData->RangeMilliMeter;
  if ((pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID) &&
      (Range < 0)) {
    if (Range < BDTable[VL53L1_TUNING_PROXY_MIN])
      pRangeData->RangeStatus =
        VL53L1_RANGESTATUS_RANGE_INVALID;
    else {
      pRangeData->RangeMilliMeter = 0;
    }
  }

  return Status;
}

uint8_t VL53L1::GetOutputDataIndex(VL53L1_DEV Dev, VL53L1_range_results_t *presults)
{
  uint8_t i;
  uint8_t index = 0;
  VL53L1_OutputModes OutputMode;

  OutputMode = VL53L1DevDataGet(Dev, CurrentParameters.OutputMode);


  if (OutputMode == VL53L1_OUTPUTMODE_NEAREST) {
    return 0;
  }


  for (i = 1; i < presults->active_results; i++) {
    if (presults->VL53L1_p_002[i].peak_signal_count_rate_mcps >
        presults->VL53L1_p_002[index].peak_signal_count_rate_mcps) {
      index = i;
    }
  }

  return index;
}

VL53L1_Error VL53L1::VL53L1_GetRangingMeasurementData(VL53L1_RangingMeasurementData_t *pRangingMeasurementData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_LLDriverData_t *pdev =
    VL53L1DevStructGetLLDriverHandle(Dev);
  VL53L1_range_results_t *presults =
    (VL53L1_range_results_t *) pdev->wArea1;
  VL53L1_range_data_t *presults_data;
  VL53L1_PresetModes PresetMode;
  uint8_t index = 0;




  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);

  if (PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) {
    Status = VL53L1_ERROR_MODE_NOT_SUPPORTED;

    return Status;
  }


  memset(pRangingMeasurementData, 0xFF,
         sizeof(VL53L1_RangingMeasurementData_t));


  Status = VL53L1_get_device_results(
             Dev,
             VL53L1_DEVICERESULTSLEVEL_FULL,
             presults);

  if (Status == VL53L1_ERROR_NONE) {
    pRangingMeasurementData->StreamCount = presults->stream_count;


    index = GetOutputDataIndex(Dev, presults);
    presults_data = &(presults->VL53L1_p_002[index]);
    Status = SetSimpleData(Dev, presults->active_results,
                           presults->device_status,
                           presults_data,
                           pRangingMeasurementData);
  }


  return Status;
}

VL53L1_Error VL53L1::SetMeasurementData(VL53L1_DEV Dev, VL53L1_range_results_t *presults, VL53L1_MultiRangingData_t *pMultiRangingData)
{
  uint8_t i;
  uint8_t iteration;
  VL53L1_TargetRangeData_t *pRangeData;
  VL53L1_range_data_t *presults_data;
  int16_t dmax_min;
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t Furthest_idx = 0;
  int16_t Furthest_range = 0;
  uint8_t ActiveResults, amb_idx;

  pMultiRangingData->NumberOfObjectsFound = presults->active_results;
  pMultiRangingData->RoiNumber = presults->zone_id;
  pMultiRangingData->HasXtalkValueChanged =
    presults->smudge_corrector_data.new_xtalk_applied_flag;
  dmax_min = MIN(presults->wrap_dmax_mm,
                 presults->VL53L1_p_007[DMAX_REFLECTANCE_IDX]);
  pMultiRangingData->DmaxMilliMeter = dmax_min;


  pMultiRangingData->TimeStamp = 0;

  pMultiRangingData->StreamCount = presults->stream_count;

  pMultiRangingData->RecommendedDistanceMode =
    VL53L1DevDataGet(Dev, CurrentParameters.DistanceMode);
  ActiveResults = presults->active_results;
  if (ActiveResults < 1)

  {
    iteration = 1;
  } else {
    iteration = ActiveResults;
  }
  for (i = 0; i < iteration; i++) {
    pRangeData = &(pMultiRangingData->RangeData[i]);

    presults_data = &(presults->VL53L1_p_002[i]);
    if (Status == VL53L1_ERROR_NONE)
      Status = SetTargetData(Dev, ActiveResults,
                             presults->device_status,
                             presults_data,
                             pRangeData);

    pMultiRangingData->EffectiveSpadRtnCount =
      presults_data->VL53L1_p_006;

    if ((pRangeData->RangeStatus == VL53L1_RANGESTATUS_RANGE_VALID)
        && (pRangeData->RangeMilliMeter > Furthest_range)) {
      Furthest_range = pRangeData->RangeMilliMeter;
      Furthest_idx = i;
    }

  }

  if ((Status == VL53L1_ERROR_NONE) && (ActiveResults > 0)) {
    pRangeData = &(pMultiRangingData->RangeData[Furthest_idx]);
    amb_idx = VL53L1_MAX_AMBIENT_DMAX_VALUES - 1;
    CheckAndChangeDistanceMode(Dev, pRangeData,
                               presults->VL53L1_p_007[amb_idx],
                               &pMultiRangingData->RecommendedDistanceMode);
  }

  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetMultiRangingData(VL53L1_MultiRangingData_t *pMultiRangingData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_LLDriverData_t *pdev =
    VL53L1DevStructGetLLDriverHandle(Dev);
  VL53L1_range_results_t *presults =
    (VL53L1_range_results_t *) pdev->wArea1;




  memset(pMultiRangingData, 0xFF,
         sizeof(VL53L1_MultiRangingData_t));


  Status = VL53L1_get_device_results(
             Dev,
             VL53L1_DEVICERESULTSLEVEL_FULL,
             presults);


  if (Status == VL53L1_ERROR_NONE) {
    switch (presults->rd_device_state) {
      case VL53L1_DEVICESTATE_RANGING_GATHER_DATA:
        pMultiRangingData->RoiStatus =
          VL53L1_ROISTATUS_VALID_NOT_LAST;
        break;
      case VL53L1_DEVICESTATE_RANGING_OUTPUT_DATA:
        pMultiRangingData->RoiStatus =
          VL53L1_ROISTATUS_VALID_LAST;
        break;
      default:
        pMultiRangingData->RoiStatus =
          VL53L1_ROISTATUS_NOT_VALID;
    }

    Status = SetMeasurementData(Dev,
                                presults,
                                pMultiRangingData);

  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetAdditionalData(VL53L1_AdditionalData_t *pAdditionalData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_additional_data(Dev, pAdditionalData);


  return Status;
}






VL53L1_Error VL53L1::VL53L1_SetTuningParameter(uint16_t TuningParameterId, int32_t TuningParameterValue)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  if (TuningParameterId ==
      VL53L1_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS) {
    return VL53L1_ERROR_INVALID_PARAMS;
  }

  if (TuningParameterId >= 32768)
    Status = VL53L1_set_tuning_parm(Dev,
                                    TuningParameterId,
                                    TuningParameterValue);
  else {
    if (TuningParameterId < VL53L1_TUNING_MAX_TUNABLE_KEY) {
      BDTable[TuningParameterId] = TuningParameterValue;
    } else {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    }
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetTuningParameter(uint16_t TuningParameterId, int32_t *pTuningParameterValue)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  if (TuningParameterId >= 32768)
    Status = VL53L1_get_tuning_parm(Dev,
                                    TuningParameterId,
                                    pTuningParameterValue);
  else {
    if (TuningParameterId < VL53L1_TUNING_MAX_TUNABLE_KEY) {
      *pTuningParameterValue = BDTable[TuningParameterId];
    } else {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    }
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_PerformRefSpadManagement()
{
#ifdef VL53L1_NOCALIB
  VL53L1_Error Status = VL53L1_ERROR_NOT_SUPPORTED;

  SUPPRESS_UNUSED_WARNING(Dev);


#else
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_Error RawStatus;
  uint8_t dcrbuffer[24];
  uint8_t *commbuf;
  uint8_t numloc[2] = {5, 3};
  VL53L1_LLDriverData_t *pdev;
  VL53L1_customer_nvm_managed_t *pc;
  VL53L1_PresetModes PresetMode;



  pdev = VL53L1DevStructGetLLDriverHandle(Dev);
  pc = &pdev->customer;

  if (Status == VL53L1_ERROR_NONE) {
    PresetMode = VL53L1DevDataGet(Dev,
                                  CurrentParameters.PresetMode);
    Status = VL53L1_run_ref_spad_char(Dev, &RawStatus);

    if (Status == VL53L1_ERROR_NONE) {
      Status = VL53L1_SetPresetMode(PresetMode);
    }
  }

  if (Status == VL53L1_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH) {

    Status = VL53L1_read_nvm_raw_data(Dev,
                                      (uint8_t)(0xA0 >> 2),
                                      (uint8_t)(24 >> 2),
                                      dcrbuffer);

    if (Status == VL53L1_ERROR_NONE)
      Status = VL53L1_WriteMulti(Dev,
                                 VL53L1_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
                                 numloc, 2);

    if (Status == VL53L1_ERROR_NONE) {
      pc->ref_spad_man__num_requested_ref_spads = numloc[0];
      pc->ref_spad_man__ref_location = numloc[1];
    }

    if (Status == VL53L1_ERROR_NONE) {
      commbuf = &dcrbuffer[16];
    }



    if (Status == VL53L1_ERROR_NONE)
      Status = VL53L1_WriteMulti(Dev,
                                 VL53L1_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
                                 commbuf, 6);

    if (Status == VL53L1_ERROR_NONE) {
      pc->global_config__spad_enables_ref_0 = commbuf[0];
      pc->global_config__spad_enables_ref_1 = commbuf[1];
      pc->global_config__spad_enables_ref_2 = commbuf[2];
      pc->global_config__spad_enables_ref_3 = commbuf[3];
      pc->global_config__spad_enables_ref_4 = commbuf[4];
      pc->global_config__spad_enables_ref_5 = commbuf[5];
    }

  }

#endif


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SmudgeCorrectionEnable(VL53L1_SmudgeCorrectionModes Mode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_Error s1 = VL53L1_ERROR_NONE;
  VL53L1_Error s2 = VL53L1_ERROR_NONE;
  VL53L1_Error s3 = VL53L1_ERROR_NONE;



  switch (Mode) {
    case VL53L1_SMUDGE_CORRECTION_NONE:
      s1 = VL53L1_dynamic_xtalk_correction_disable(Dev);
      s2 = VL53L1_dynamic_xtalk_correction_apply_disable(Dev);
      s3 = VL53L1_dynamic_xtalk_correction_single_apply_disable(Dev);
      break;
    case VL53L1_SMUDGE_CORRECTION_CONTINUOUS:
      s1 = VL53L1_dynamic_xtalk_correction_enable(Dev);
      s2 = VL53L1_dynamic_xtalk_correction_apply_enable(Dev);
      s3 = VL53L1_dynamic_xtalk_correction_single_apply_disable(Dev);
      break;
    case VL53L1_SMUDGE_CORRECTION_SINGLE:
      s1 = VL53L1_dynamic_xtalk_correction_enable(Dev);
      s2 = VL53L1_dynamic_xtalk_correction_apply_enable(Dev);
      s3 = VL53L1_dynamic_xtalk_correction_single_apply_enable(Dev);
      break;
    case VL53L1_SMUDGE_CORRECTION_DEBUG:
      s1 = VL53L1_dynamic_xtalk_correction_enable(Dev);
      s2 = VL53L1_dynamic_xtalk_correction_apply_disable(Dev);
      s3 = VL53L1_dynamic_xtalk_correction_single_apply_disable(Dev);
      break;
    default:
      Status = VL53L1_ERROR_INVALID_PARAMS;
      break;
  }

  if (Status == VL53L1_ERROR_NONE) {
    Status = s1;
    if (Status == VL53L1_ERROR_NONE) {
      Status = s2;
    }
    if (Status == VL53L1_ERROR_NONE) {
      Status = s3;
    }
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetXTalkCompensationEnable(uint8_t XTalkCompensationEnable)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  if (XTalkCompensationEnable == 0) {
    Status = VL53L1_disable_xtalk_compensation(Dev);
  } else {
    Status = VL53L1_enable_xtalk_compensation(Dev);
  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetXTalkCompensationEnable(uint8_t *pXTalkCompensationEnable)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  VL53L1_get_xtalk_compensation_enable(
    Dev,
    pXTalkCompensationEnable);


  return Status;
}


VL53L1_Error VL53L1::VL53L1_PerformXTalkCalibration(uint8_t CalibrationOption)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_Error UStatus;
  int16_t CalDistanceMm;
  VL53L1_xtalk_calibration_results_t xtalk;

  VL53L1_CalibrationData_t caldata;
  VL53L1_LLDriverData_t *pLLData;
  int i;
  uint32_t *pPlaneOffsetKcps;
  uint32_t Margin =
    BDTable[VL53L1_TUNING_XTALK_FULL_ROI_BIN_SUM_MARGIN];
  uint32_t DefaultOffset =
    BDTable[VL53L1_TUNING_XTALK_FULL_ROI_DEFAULT_OFFSET];
  uint32_t *pLLDataPlaneOffsetKcps;
  uint32_t sum = 0;
  uint8_t binok = 0;
  int32_t merge;



  pPlaneOffsetKcps =
    &caldata.customer.algo__crosstalk_compensation_plane_offset_kcps;
  pLLData = VL53L1DevStructGetLLDriverHandle(Dev);
  pLLDataPlaneOffsetKcps =
    &pLLData->xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps;
  VL53L1_get_tuning_parm(Dev, VL53L1_TUNINGPARM_HIST_MERGE, &merge);

  VL53L1_set_tuning_parm(Dev, VL53L1_TUNINGPARM_HIST_MERGE, 0);
  switch (CalibrationOption) {
    case VL53L1_XTALKCALIBRATIONMODE_NO_TARGET:
      Status = VL53L1_run_xtalk_extraction(Dev, &UStatus);

      if (Status == VL53L1_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL) {
        VL53L1_xtalk_cal_data_init(Dev);
      }
      break;
    case VL53L1_XTALKCALIBRATIONMODE_SINGLE_TARGET:
      Status = SingleTargetXTalkCalibration(Dev);
      break;
    case VL53L1_XTALKCALIBRATIONMODE_FULL_ROI:
      CalDistanceMm = (int16_t)
                      BDTable[VL53L1_TUNING_XTALK_FULL_ROI_TARGET_DISTANCE_MM];

      VL53L1_set_tuning_parm(Dev, VL53L1_TUNINGPARM_HIST_MERGE,
                             merge);
      Status = VL53L1_run_hist_xtalk_extraction(Dev, CalDistanceMm,
                                                &UStatus);

      VL53L1_GetCalibrationData(&caldata);
      for (i = 0; i < VL53L1_XTALK_HISTO_BINS; i++) {
        sum += caldata.xtalkhisto.xtalk_shape.bin_data[i];
        if (caldata.xtalkhisto.xtalk_shape.bin_data[i] > 0) {
          binok++;
        }
      }
      if ((UStatus ==
           VL53L1_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL) ||
          (sum > (1024 + Margin)) || (sum < (1024 - Margin)) ||
          (binok < 3)) {
        *pPlaneOffsetKcps = DefaultOffset;
        *pLLDataPlaneOffsetKcps = DefaultOffset;
        caldata.xtalkhisto.xtalk_shape.bin_data[0] = 307;
        caldata.xtalkhisto.xtalk_shape.bin_data[1] = 410;
        caldata.xtalkhisto.xtalk_shape.bin_data[2] = 410;
        caldata.xtalkhisto.xtalk_shape.bin_data[3] = 307;
        for (i = 4; i < VL53L1_XTALK_HISTO_BINS; i++) {
          caldata.xtalkhisto.xtalk_shape.bin_data[i] = 0;
        }
        for (i = 0; i < VL53L1_BIN_REC_SIZE; i++)
          caldata.algo__xtalk_cpo_HistoMerge_kcps[i] =
            DefaultOffset + DefaultOffset * i;
        VL53L1_SetCalibrationData(&caldata);
      }

      break;
    default:
      Status = VL53L1_ERROR_INVALID_PARAMS;
  }
  VL53L1_set_tuning_parm(Dev, VL53L1_TUNINGPARM_HIST_MERGE, merge);

  if (Status == VL53L1_ERROR_NONE) {
    Status = VL53L1_get_current_xtalk_settings(Dev, &xtalk);
    Status = VL53L1_set_tuning_parm(Dev,
                                    VL53L1_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
                                    xtalk.algo__crosstalk_compensation_plane_offset_kcps);
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetOffsetCalibrationMode(VL53L1_OffsetCalibrationModes OffsetCalibrationMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_OffsetCalibrationMode   offset_cal_mode;



  if (OffsetCalibrationMode == VL53L1_OFFSETCALIBRATIONMODE_STANDARD) {
    offset_cal_mode =
      VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
  } else if (OffsetCalibrationMode ==
             VL53L1_OFFSETCALIBRATIONMODE_PRERANGE_ONLY) {
    offset_cal_mode =
      VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY;
  } else if (OffsetCalibrationMode ==
             VL53L1_OFFSETCALIBRATIONMODE_MULTI_ZONE) {
    offset_cal_mode =
      VL53L1_OFFSETCALIBRATIONMODE__PER_ZONE;
  } else {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE)
    Status =  VL53L1_set_offset_calibration_mode(Dev,
                                                 offset_cal_mode);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetOffsetCorrectionMode(VL53L1_OffsetCorrectionModes OffsetCorrectionMode)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_OffsetCorrectionMode   offset_cor_mode;



  if (OffsetCorrectionMode == VL53L1_OFFSETCORRECTIONMODE_STANDARD) {
    offset_cor_mode =
      VL53L1_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
  } else if (OffsetCorrectionMode ==
             VL53L1_OFFSETCORRECTIONMODE_PERZONE) {
    offset_cor_mode =
      VL53L1_OFFSETCORRECTIONMODE__PER_ZONE_OFFSETS;
  } else if (OffsetCorrectionMode ==
             VL53L1_OFFSETCORRECTIONMODE_PERVCSEL) {
    offset_cor_mode =
      VL53L1_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS;
  } else {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  if (Status == VL53L1_ERROR_NONE)
    Status =  VL53L1_set_offset_correction_mode(Dev,
                                                offset_cor_mode);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_PerformOffsetCalibration(int32_t CalDistanceMilliMeter, FixPoint1616_t CalReflectancePercent)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_Error UnfilteredStatus;
  VL53L1_OffsetCalibrationMode   offset_cal_mode;
  uint16_t CalReflectancePercent_int;

  VL53L1_DevicePresetModes      device_preset_mode;
  VL53L1_DeviceZonePreset       zone_preset;
  VL53L1_zone_config_t         zone_cfg;



  CalReflectancePercent_int =
    VL53L1_FIXPOINT1616TOFIXPOINT72(CalReflectancePercent);

  if (Status == VL53L1_ERROR_NONE)
    Status =  VL53L1_get_offset_calibration_mode(Dev,
                                                 &offset_cal_mode);

  if (Status != VL53L1_ERROR_NONE) {

    return Status;
  }


  if ((offset_cal_mode ==
       VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD) ||
      (offset_cal_mode ==
       VL53L1_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD_PRE_RANGE_ONLY
      )) {
    if (Status == VL53L1_ERROR_NONE)
      Status = VL53L1_run_offset_calibration(
                 Dev,
                 (int16_t)CalDistanceMilliMeter,
                 CalReflectancePercent_int,
                 &UnfilteredStatus);

  } else if (offset_cal_mode ==
             VL53L1_OFFSETCALIBRATIONMODE__PER_ZONE) {
    device_preset_mode =
      VL53L1_DEVICEPRESETMODE_HISTOGRAM_MULTIZONE_LONG_RANGE;
    zone_preset = VL53L1_DEVICEZONEPRESET_CUSTOM;

    Status = VL53L1_get_zone_config(Dev, &zone_cfg);
    if (Status == VL53L1_ERROR_NONE)
      Status = VL53L1_run_zone_calibration(
                 Dev,
                 device_preset_mode,
                 zone_preset,
                 &zone_cfg,
                 (int16_t)CalDistanceMilliMeter,
                 CalReflectancePercent_int,
                 &UnfilteredStatus);

  } else {
    Status = VL53L1_ERROR_INVALID_PARAMS;
  }

  return Status;
}

VL53L1_Error VL53L1::VL53L1_PerformOffsetSimpleCalibration(int32_t CalDistanceMilliMeter)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  int32_t sum_ranging;
  uint8_t offset_meas;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t total_count, inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  int16_t offset;
  VL53L1_RangingMeasurementData_t RangingMeasurementData;
  VL53L1_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53L1_Error SmudgeStatus = VL53L1_ERROR_NONE;
  uint8_t smudge_corr_en;



  pdev = VL53L1DevStructGetLLDriverHandle(Dev);

  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53L1_dynamic_xtalk_correction_disable(Dev);

  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
  Repeat = BDTable[VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
  Max = BDTable[
         VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);
  sum_ranging = 0;
  total_count = 0;

  while ((Repeat > 0) && (Status == VL53L1_ERROR_NONE)) {
    Status = VL53L1_StartMeasurement();

    if (Status == VL53L1_ERROR_NONE) {
      VL53L1_WaitMeasurementDataReady();
      VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      VL53L1_ClearInterruptAndStartMeasurement();
    }

    inloopcount = 0;
    offset_meas = 0;
    while ((Status == VL53L1_ERROR_NONE) && (inloopcount < Max) &&
           (offset_meas < OverMax)) {
      Status = VL53L1_WaitMeasurementDataReady();
      if (Status == VL53L1_ERROR_NONE) {
        Status = VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      }
      goodmeas = (RangingMeasurementData.RangeStatus ==
                  VL53L1_RANGESTATUS_RANGE_VALID);
      if ((Status == VL53L1_ERROR_NONE) && goodmeas) {
        sum_ranging = sum_ranging +
                      RangingMeasurementData.RangeMilliMeter;
        inloopcount++;
      }
      Status = VL53L1_ClearInterruptAndStartMeasurement();
      offset_meas++;
    }
    total_count += inloopcount;


    if (inloopcount < UnderMax) {
      Status = VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }

    VL53L1_StopMeasurement();

    Repeat--;

  }

  if ((SmudgeStatus == VL53L1_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53L1_dynamic_xtalk_correction_enable(Dev);
  }

  if ((sum_ranging < 0) ||
      (sum_ranging > ((int32_t) total_count * 0xffff))) {
    Status = VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
  }

  if ((Status == VL53L1_ERROR_NONE) && (total_count > 0)) {
    IncRounding = total_count / 2;
    meanDistance_mm = (int16_t)((sum_ranging + IncRounding)
                                / total_count);
    offset = (int16_t)CalDistanceMilliMeter - meanDistance_mm;
    pdev->customer.algo__part_to_part_range_offset_mm = 0;
    pdev->customer.mm_config__inner_offset_mm = offset;
    pdev->customer.mm_config__outer_offset_mm = offset;

    Status = VL53L1_set_customer_nvm_managed(Dev,
                                             &(pdev->customer));
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_PerformOffsetZeroDistanceCalibration()
{
#define START_OFFSET 50
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  int32_t sum_ranging;
  uint8_t offset_meas;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t total_count, inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  int16_t offset, ZeroDistanceOffset;
  VL53L1_RangingMeasurementData_t RangingMeasurementData;
  VL53L1_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53L1_Error SmudgeStatus = VL53L1_ERROR_NONE;
  uint8_t smudge_corr_en;



  pdev = VL53L1DevStructGetLLDriverHandle(Dev);
  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53L1_dynamic_xtalk_correction_disable(Dev);
  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = START_OFFSET;
  pdev->customer.mm_config__outer_offset_mm = START_OFFSET;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));
  ZeroDistanceOffset = BDTable[
                        VL53L1_TUNING_ZERO_DISTANCE_OFFSET_NON_LINEAR_FACTOR];
  Repeat = BDTable[VL53L1_TUNING_SIMPLE_OFFSET_CALIBRATION_REPEAT];
  Max = BDTable[
         VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);
  sum_ranging = 0;
  total_count = 0;

  while ((Repeat > 0) && (Status == VL53L1_ERROR_NONE)) {
    Status = VL53L1_StartMeasurement();
    if (Status == VL53L1_ERROR_NONE) {
      VL53L1_WaitMeasurementDataReady();
      VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      VL53L1_ClearInterruptAndStartMeasurement();
    }
    inloopcount = 0;
    offset_meas = 0;
    while ((Status == VL53L1_ERROR_NONE) && (inloopcount < Max) &&
           (offset_meas < OverMax)) {
      Status = VL53L1_WaitMeasurementDataReady();
      if (Status == VL53L1_ERROR_NONE) {
        Status = VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      }
      goodmeas = (RangingMeasurementData.RangeStatus ==
                  VL53L1_RANGESTATUS_RANGE_VALID);
      if ((Status == VL53L1_ERROR_NONE) && goodmeas) {
        sum_ranging = sum_ranging +
                      RangingMeasurementData.RangeMilliMeter;
        inloopcount++;
      }
      Status = VL53L1_ClearInterruptAndStartMeasurement();
      offset_meas++;
    }
    total_count += inloopcount;
    if (inloopcount < UnderMax) {
      Status = VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }
    VL53L1_StopMeasurement();
    Repeat--;
  }
  if ((SmudgeStatus == VL53L1_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53L1_dynamic_xtalk_correction_enable(Dev);
  }
  if ((sum_ranging < 0) ||
      (sum_ranging > ((int32_t) total_count * 0xffff))) {
    Status = VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
  }

  if ((Status == VL53L1_ERROR_NONE) && (total_count > 0)) {
    IncRounding = total_count / 2;
    meanDistance_mm = (int16_t)
                      ((sum_ranging + IncRounding) / total_count);
    offset = START_OFFSET - meanDistance_mm + ZeroDistanceOffset;
    pdev->customer.algo__part_to_part_range_offset_mm = 0;
    pdev->customer.mm_config__inner_offset_mm = offset;
    pdev->customer.mm_config__outer_offset_mm = offset;
    Status = VL53L1_set_customer_nvm_managed(Dev,
                                             &(pdev->customer));
  }


  return Status;
}

VL53L1_Error VL53L1::VL53L1_SetCalibrationData(VL53L1_CalibrationData_t *pCalibrationData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_CustomerNvmManaged_t          *pC;
  VL53L1_calibration_data_t            cal_data;
  uint32_t x;
  VL53L1_xtalk_calibration_results_t xtalk;



  cal_data.struct_version = pCalibrationData->struct_version -
                            VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;




  memcpy(
    &(cal_data.fmt_dmax_cal),
    &(pCalibrationData->fmt_dmax_cal),
    sizeof(VL53L1_dmax_calibration_data_t));


  memcpy(
    &(cal_data.cust_dmax_cal),
    &(pCalibrationData->cust_dmax_cal),
    sizeof(VL53L1_dmax_calibration_data_t));



  memcpy(
    &(cal_data.add_off_cal_data),
    &(pCalibrationData->add_off_cal_data),
    sizeof(VL53L1_additional_offset_cal_data_t));


  memcpy(
    &(cal_data.optical_centre),
    &(pCalibrationData->optical_centre),
    sizeof(VL53L1_optical_centre_t));


  memcpy(
    &(cal_data.xtalkhisto),
    &(pCalibrationData->xtalkhisto),
    sizeof(VL53L1_xtalk_histogram_data_t));


  memcpy(
    &(cal_data.gain_cal),
    &(pCalibrationData->gain_cal),
    sizeof(VL53L1_gain_calibration_data_t));


  memcpy(
    &(cal_data.cal_peak_rate_map),
    &(pCalibrationData->cal_peak_rate_map),
    sizeof(VL53L1_cal_peak_rate_map_t));


  memcpy(
    &(cal_data.per_vcsel_cal_data),
    &(pCalibrationData->per_vcsel_cal_data),
    sizeof(VL53L1_per_vcsel_period_offset_cal_data_t));

  pC = &pCalibrationData->customer;
  x = pC->algo__crosstalk_compensation_plane_offset_kcps;
  cal_data.customer.algo__crosstalk_compensation_plane_offset_kcps =
    (uint16_t)(x & 0x0000FFFF);

  cal_data.customer.global_config__spad_enables_ref_0 =
    pC->global_config__spad_enables_ref_0;
  cal_data.customer.global_config__spad_enables_ref_1 =
    pC->global_config__spad_enables_ref_1;
  cal_data.customer.global_config__spad_enables_ref_2 =
    pC->global_config__spad_enables_ref_2;
  cal_data.customer.global_config__spad_enables_ref_3 =
    pC->global_config__spad_enables_ref_3;
  cal_data.customer.global_config__spad_enables_ref_4 =
    pC->global_config__spad_enables_ref_4;
  cal_data.customer.global_config__spad_enables_ref_5 =
    pC->global_config__spad_enables_ref_5;
  cal_data.customer.global_config__ref_en_start_select =
    pC->global_config__ref_en_start_select;
  cal_data.customer.ref_spad_man__num_requested_ref_spads =
    pC->ref_spad_man__num_requested_ref_spads;
  cal_data.customer.ref_spad_man__ref_location =
    pC->ref_spad_man__ref_location;
  cal_data.customer.algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
  cal_data.customer.algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps;
  cal_data.customer.ref_spad_char__total_rate_target_mcps =
    pC->ref_spad_char__total_rate_target_mcps;
  cal_data.customer.algo__part_to_part_range_offset_mm =
    pC->algo__part_to_part_range_offset_mm;
  cal_data.customer.mm_config__inner_offset_mm =
    pC->mm_config__inner_offset_mm;
  cal_data.customer.mm_config__outer_offset_mm =
    pC->mm_config__outer_offset_mm;

  Status = VL53L1_set_part_to_part_data(Dev, &cal_data);
  if (Status != VL53L1_ERROR_NONE) {
    goto ENDFUNC;
  }

  Status = VL53L1_get_current_xtalk_settings(Dev, &xtalk);

  if (Status != VL53L1_ERROR_NONE) {
    goto ENDFUNC;
  }

  xtalk.algo__crosstalk_compensation_plane_offset_kcps = x;

  Status = VL53L1_set_tuning_parm(Dev,
                                  VL53L1_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS,
                                  x);


  memcpy(
    &(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
    &(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
    sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));

  Status = VL53L1_set_current_xtalk_settings(Dev, &xtalk);

ENDFUNC:

  return Status;

}

VL53L1_Error VL53L1::VL53L1_GetCalibrationData(VL53L1_CalibrationData_t  *pCalibrationData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_calibration_data_t      cal_data;
  VL53L1_CustomerNvmManaged_t         *pC;
  VL53L1_customer_nvm_managed_t       *pC2;
  VL53L1_xtalk_calibration_results_t xtalk;
  uint32_t                          tmp;
  VL53L1_PresetModes PresetMode;




  Status = VL53L1_get_part_to_part_data(Dev, &cal_data);

  pCalibrationData->struct_version = cal_data.struct_version +
                                     VL53L1_ADDITIONAL_CALIBRATION_DATA_STRUCT_VERSION;


  memcpy(
    &(pCalibrationData->fmt_dmax_cal),
    &(cal_data.fmt_dmax_cal),
    sizeof(VL53L1_dmax_calibration_data_t));


  memcpy(
    &(pCalibrationData->cust_dmax_cal),
    &(cal_data.cust_dmax_cal),
    sizeof(VL53L1_dmax_calibration_data_t));


  memcpy(
    &(pCalibrationData->add_off_cal_data),
    &(cal_data.add_off_cal_data),
    sizeof(VL53L1_additional_offset_cal_data_t));


  memcpy(
    &(pCalibrationData->optical_centre),
    &(cal_data.optical_centre),
    sizeof(VL53L1_optical_centre_t));


  memcpy(
    &(pCalibrationData->xtalkhisto),
    &(cal_data.xtalkhisto),
    sizeof(VL53L1_xtalk_histogram_data_t));

  memcpy(
    &(pCalibrationData->gain_cal),
    &(cal_data.gain_cal),
    sizeof(VL53L1_gain_calibration_data_t));


  memcpy(
    &(pCalibrationData->cal_peak_rate_map),
    &(cal_data.cal_peak_rate_map),
    sizeof(VL53L1_cal_peak_rate_map_t));


  memcpy(
    &(pCalibrationData->per_vcsel_cal_data),
    &(cal_data.per_vcsel_cal_data),
    sizeof(VL53L1_per_vcsel_period_offset_cal_data_t));

  pC = &pCalibrationData->customer;
  pC2 = &cal_data.customer;
  pC->global_config__spad_enables_ref_0 =
    pC2->global_config__spad_enables_ref_0;
  pC->global_config__spad_enables_ref_1 =
    pC2->global_config__spad_enables_ref_1;
  pC->global_config__spad_enables_ref_2 =
    pC2->global_config__spad_enables_ref_2;
  pC->global_config__spad_enables_ref_3 =
    pC2->global_config__spad_enables_ref_3;
  pC->global_config__spad_enables_ref_4 =
    pC2->global_config__spad_enables_ref_4;
  pC->global_config__spad_enables_ref_5 =
    pC2->global_config__spad_enables_ref_5;
  pC->global_config__ref_en_start_select =
    pC2->global_config__ref_en_start_select;
  pC->ref_spad_man__num_requested_ref_spads =
    pC2->ref_spad_man__num_requested_ref_spads;
  pC->ref_spad_man__ref_location =
    pC2->ref_spad_man__ref_location;
  pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
    pC2->algo__crosstalk_compensation_x_plane_gradient_kcps;
  pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
    pC2->algo__crosstalk_compensation_y_plane_gradient_kcps;
  pC->ref_spad_char__total_rate_target_mcps =
    pC2->ref_spad_char__total_rate_target_mcps;
  pC->algo__part_to_part_range_offset_mm =
    pC2->algo__part_to_part_range_offset_mm;
  pC->mm_config__inner_offset_mm =
    pC2->mm_config__inner_offset_mm;
  pC->mm_config__outer_offset_mm =
    pC2->mm_config__outer_offset_mm;

  pC->algo__crosstalk_compensation_plane_offset_kcps =
    (uint32_t)(
      pC2->algo__crosstalk_compensation_plane_offset_kcps);

  PresetMode = VL53L1DevDataGet(Dev, CurrentParameters.PresetMode);

  if ((PresetMode == VL53L1_PRESETMODE_RANGING) ||
      (PresetMode == VL53L1_PRESETMODE_MULTIZONES_SCANNING) ||
      (PresetMode == VL53L1_PRESETMODE_PROXY_RANGING_MODE)
     ) {

    Status = VL53L1_get_current_xtalk_settings(Dev, &xtalk);

    if (Status != VL53L1_ERROR_NONE) {
      goto ENDFUNC;
    }

    tmp = xtalk.algo__crosstalk_compensation_plane_offset_kcps;
    pC->algo__crosstalk_compensation_plane_offset_kcps = tmp;
    tmp = xtalk.algo__crosstalk_compensation_x_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_x_plane_gradient_kcps = tmp;
    tmp = xtalk.algo__crosstalk_compensation_y_plane_gradient_kcps;
    pC->algo__crosstalk_compensation_y_plane_gradient_kcps = tmp;

    memcpy(&(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps[0]),
           &(xtalk.algo__xtalk_cpo_HistoMerge_kcps[0]),
           sizeof(pCalibrationData->algo__xtalk_cpo_HistoMerge_kcps));
  }
ENDFUNC:

  return Status;
}


VL53L1_Error VL53L1::VL53L1_SetZoneCalibrationData(VL53L1_ZoneCalibrationData_t *pZoneCalibrationData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_set_zone_calibration_data(Dev, pZoneCalibrationData);


  return Status;

}

VL53L1_Error VL53L1::VL53L1_GetZoneCalibrationData(VL53L1_ZoneCalibrationData_t  *pZoneCalibrationData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;



  Status = VL53L1_get_zone_calibration_data(Dev, pZoneCalibrationData);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_GetOpticalCenter(FixPoint1616_t *pOpticalCenterX, FixPoint1616_t *pOpticalCenterY)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_calibration_data_t  CalibrationData;



  *pOpticalCenterX = 0;
  *pOpticalCenterY = 0;
  Status = VL53L1_get_part_to_part_data(Dev, &CalibrationData);
  if (Status == VL53L1_ERROR_NONE) {
    *pOpticalCenterX = VL53L1_FIXPOINT44TOFIXPOINT1616(
                         CalibrationData.optical_centre.x_centre);
    *pOpticalCenterY = VL53L1_FIXPOINT44TOFIXPOINT1616(
                         CalibrationData.optical_centre.y_centre);
  }


  return Status;
}






VL53L1_Error VL53L1::VL53L1_SetThresholdConfig(VL53L1_DetectionConfig_t *pConfig)
{
#define BADTHRESBOUNDS(T) \
  (((T.CrossMode == VL53L1_THRESHOLD_OUT_OF_WINDOW) || \
  (T.CrossMode == VL53L1_THRESHOLD_IN_WINDOW)) && (T.Low > T.High))

  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_GPIO_interrupt_config_t Cfg;
  uint16_t g;
  FixPoint1616_t gain, high1616, low1616;
  VL53L1_LLDriverData_t *pdev;



  pdev = VL53L1DevStructGetLLDriverHandle(Dev);

  Status = VL53L1_get_GPIO_interrupt_config(Dev, &Cfg);
  if (Status != VL53L1_ERROR_NONE) {
    return Status;
  }

  if (pConfig->DetectionMode == VL53L1_DETECTION_NORMAL_RUN) {
    Cfg.intr_new_measure_ready = 1;
    Status = VL53L1_set_GPIO_interrupt_config_struct(Dev,
                                                     Cfg);
  } else {
    if (BADTHRESBOUNDS(pConfig->Distance)) {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    }
    if ((Status == VL53L1_ERROR_NONE) &&
        (BADTHRESBOUNDS(pConfig->Rate))) {
      Status = VL53L1_ERROR_INVALID_PARAMS;
    }
    if (Status == VL53L1_ERROR_NONE) {
      Cfg.intr_new_measure_ready = 0;
      Cfg.intr_no_target = pConfig->IntrNoTarget;

      g = pdev->gain_cal.standard_ranging_gain_factor;
      if (g != 0) {

        gain = (FixPoint1616_t)((uint32_t)g << 5);
        high1616 = (FixPoint1616_t)((uint32_t)
                                    pConfig->Distance.High << 16);
        low1616 = (FixPoint1616_t)((uint32_t)
                                   pConfig->Distance.Low << 16);

        high1616 = (high1616 + 32768) / gain;
        low1616 = (low1616 + 32768) / gain;
        Cfg.threshold_distance_high = (uint16_t)
                                      (high1616 & 0xFFFF);
        Cfg.threshold_distance_low = (uint16_t)
                                     (low1616 & 0xFFFF);
      }
      Cfg.threshold_rate_high =
        VL53L1_FIXPOINT1616TOFIXPOINT97(
          pConfig->Rate.High);
      Cfg.threshold_rate_low =
        VL53L1_FIXPOINT1616TOFIXPOINT97(
          pConfig->Rate.Low);

      Cfg.intr_mode_distance = ConvertModeToLLD(
                                 &Status,
                                 pConfig->Distance.CrossMode);
      if (Status == VL53L1_ERROR_NONE)
        Cfg.intr_mode_rate = ConvertModeToLLD(
                               &Status,
                               pConfig->Rate.CrossMode);
    }


    if (Status == VL53L1_ERROR_NONE) {
      Cfg.intr_combined_mode = 1;
      switch (pConfig->DetectionMode) {
        case VL53L1_DETECTION_DISTANCE_ONLY:
          Cfg.threshold_rate_high = 0;
          Cfg.threshold_rate_low = 0;
          break;
        case VL53L1_DETECTION_RATE_ONLY:
          Cfg.threshold_distance_high = 0;
          Cfg.threshold_distance_low = 0;
          break;
        case VL53L1_DETECTION_DISTANCE_OR_RATE:

          break;
        case VL53L1_DETECTION_DISTANCE_AND_RATE:
          Cfg.intr_combined_mode = 0;
          break;
        default:
          Status = VL53L1_ERROR_INVALID_PARAMS;
      }
    }

    if (Status == VL53L1_ERROR_NONE)
      Status =
        VL53L1_set_GPIO_interrupt_config_struct(Dev, Cfg);

  }


  return Status;
}


VL53L1_Error VL53L1::VL53L1_GetThresholdConfig(VL53L1_DetectionConfig_t *pConfig)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  VL53L1_GPIO_interrupt_config_t Cfg;



  Status = VL53L1_get_GPIO_interrupt_config(Dev, &Cfg);

  if (Status != VL53L1_ERROR_NONE) {

    return Status;
  }

  pConfig->IntrNoTarget = Cfg.intr_no_target;
  pConfig->Distance.High = Cfg.threshold_distance_high;
  pConfig->Distance.Low = Cfg.threshold_distance_low;
  pConfig->Rate.High =
    VL53L1_FIXPOINT97TOFIXPOINT1616(
      Cfg.threshold_rate_high);
  pConfig->Rate.Low =
    VL53L1_FIXPOINT97TOFIXPOINT1616(Cfg.threshold_rate_low);
  pConfig->Distance.CrossMode =
    ConvertModeFromLLD(&Status, Cfg.intr_mode_distance);
  if (Status == VL53L1_ERROR_NONE)
    pConfig->Rate.CrossMode =
      ConvertModeFromLLD(&Status, Cfg.intr_mode_rate);

  if (Cfg.intr_new_measure_ready == 1) {
    pConfig->DetectionMode = VL53L1_DETECTION_NORMAL_RUN;
  } else {

    if (Status == VL53L1_ERROR_NONE) {
      if (Cfg.intr_combined_mode == 0)
        pConfig->DetectionMode =
          VL53L1_DETECTION_DISTANCE_AND_RATE;
      else {
        if ((Cfg.threshold_distance_high == 0) &&
            (Cfg.threshold_distance_low == 0))
          pConfig->DetectionMode =
            VL53L1_DETECTION_RATE_ONLY;
        else if ((Cfg.threshold_rate_high == 0) &&
                 (Cfg.threshold_rate_low == 0))
          pConfig->DetectionMode =
            VL53L1_DETECTION_DISTANCE_ONLY;
        else
          pConfig->DetectionMode =
            VL53L1_DETECTION_DISTANCE_OR_RATE;
      }
    }
  }


  return Status;
}




VL53L1_Error VL53L1::VL53L1_PerformOffsetPerVcselCalibration(int32_t CalDistanceMilliMeter)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  int32_t sum_ranging_range_A, sum_ranging_range_B;
  uint8_t offset_meas_range_A, offset_meas_range_B;
  int16_t Max, UnderMax, OverMax, Repeat;
  int32_t inloopcount;
  int32_t IncRounding;
  int16_t meanDistance_mm;
  VL53L1_RangingMeasurementData_t RangingMeasurementData;
  VL53L1_LLDriverData_t *pdev;
  uint8_t goodmeas;
  VL53L1_PresetModes currentMode;
  VL53L1_DistanceModes currentDist;
  VL53L1_DistanceModes DistMode[3] = {VL53L1_DISTANCEMODE_SHORT,
                                      VL53L1_DISTANCEMODE_MEDIUM, VL53L1_DISTANCEMODE_LONG
                                     };
  int16_t offsetA[3];
  int16_t offsetB[3];

  VL53L1_Error SmudgeStatus = VL53L1_ERROR_NONE;
  uint8_t smudge_corr_en, isc;



  pdev = VL53L1DevStructGetLLDriverHandle(Dev);

  smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;
  SmudgeStatus = VL53L1_dynamic_xtalk_correction_disable(Dev);

  pdev->customer.algo__part_to_part_range_offset_mm = 0;
  pdev->customer.mm_config__inner_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  pdev->customer.mm_config__outer_offset_mm = 0;
  memset(&pdev->per_vcsel_cal_data, 0, sizeof(pdev->per_vcsel_cal_data));

  Repeat = 0;
  Max = 2 * BDTable[
         VL53L1_TUNING_MAX_SIMPLE_OFFSET_CALIBRATION_SAMPLE_NUMBER];
  UnderMax = 1 + (Max / 2);
  OverMax = Max + (Max / 2);

  Status = VL53L1_GetPresetMode(&currentMode);
  Status = VL53L1_GetDistanceMode(&currentDist);

  while ((Repeat < 3) && (Status == VL53L1_ERROR_NONE)) {
    Status = VL53L1_SetDistanceMode(DistMode[Repeat]);
    Status = VL53L1_StartMeasurement();

    if (Status == VL53L1_ERROR_NONE) {
      VL53L1_WaitMeasurementDataReady();
      VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      VL53L1_ClearInterruptAndStartMeasurement();
    }

    inloopcount = 0;
    offset_meas_range_A = 0;
    sum_ranging_range_A = 0;
    offset_meas_range_B = 0;
    sum_ranging_range_B = 0;
    while ((Status == VL53L1_ERROR_NONE) && (inloopcount < Max) &&
           (inloopcount < OverMax)) {
      Status = VL53L1_WaitMeasurementDataReady();
      if (Status == VL53L1_ERROR_NONE) {
        Status = VL53L1_GetRangingMeasurementData(&RangingMeasurementData);
      }
      goodmeas = (RangingMeasurementData.RangeStatus ==
                  VL53L1_RANGESTATUS_RANGE_VALID);
      isc = pdev->ll_state.cfg_internal_stream_count;
      if ((Status == VL53L1_ERROR_NONE) && goodmeas) {
        if (isc & 0x01) {
          sum_ranging_range_A +=
            RangingMeasurementData.RangeMilliMeter;
          offset_meas_range_A++;
        } else {
          sum_ranging_range_B +=
            RangingMeasurementData.RangeMilliMeter;
          offset_meas_range_B++;
        }
        inloopcount = offset_meas_range_A +
                      offset_meas_range_B;
      }
      Status = VL53L1_ClearInterruptAndStartMeasurement();
    }


    if (inloopcount < UnderMax) {
      Status = VL53L1_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL;
    }

    VL53L1_StopMeasurement();


    if ((sum_ranging_range_A < 0) ||
        (sum_ranging_range_B < 0) ||
        (sum_ranging_range_A >
         ((int32_t) offset_meas_range_A * 0xffff)) ||
        (sum_ranging_range_B >
         ((int32_t) offset_meas_range_B * 0xffff))) {
      Status = VL53L1_WARNING_OFFSET_CAL_SIGMA_TOO_HIGH;
    }

    if ((Status == VL53L1_ERROR_NONE) &&
        (offset_meas_range_A > 0)) {
      IncRounding = offset_meas_range_A / 2;
      meanDistance_mm = (int16_t)
                        ((sum_ranging_range_A + IncRounding)
                         / offset_meas_range_A);
      offsetA[Repeat] = (int16_t)
                        CalDistanceMilliMeter - meanDistance_mm;
    }

    if ((Status == VL53L1_ERROR_NONE) &&
        (offset_meas_range_B > 0)) {
      IncRounding = offset_meas_range_B / 2;
      meanDistance_mm = (int16_t)
                        ((sum_ranging_range_B + IncRounding)
                         / offset_meas_range_B);
      offsetB[Repeat] = (int16_t)
                        CalDistanceMilliMeter - meanDistance_mm;
    }
    Repeat++;
  }

  if ((SmudgeStatus == VL53L1_ERROR_NONE) && (smudge_corr_en == 1)) {
    SmudgeStatus = VL53L1_dynamic_xtalk_correction_enable(Dev);
  }

  if (Status == VL53L1_ERROR_NONE) {
    pdev->per_vcsel_cal_data.short_a_offset_mm  = offsetA[0];
    pdev->per_vcsel_cal_data.short_b_offset_mm  = offsetB[0];
    pdev->per_vcsel_cal_data.medium_a_offset_mm = offsetA[1];
    pdev->per_vcsel_cal_data.medium_b_offset_mm = offsetB[1];
    pdev->per_vcsel_cal_data.long_a_offset_mm   = offsetA[2];
    pdev->per_vcsel_cal_data.long_b_offset_mm   = offsetB[2];
  }

  VL53L1_SetPresetMode(currentMode);
  VL53L1_SetDistanceMode(currentDist);


  return Status;
}

VL53L1_Error VL53L1::VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data)
{
  int  status;

  status = VL53L1_I2CWrite(Dev->I2cDevAddr, index, &data, 1);
  return status;
}

VL53L1_Error VL53L1::VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data)
{
  int  status;
  uint8_t buffer[2];

  buffer[0] = data >> 8;
  buffer[1] = data & 0x00FF;
  status = VL53L1_I2CWrite(Dev->I2cDevAddr, index, (uint8_t *)buffer, 2);
  return status;
}

VL53L1_Error VL53L1::VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data)
{
  int  status;
  uint8_t buffer[4];

  buffer[0] = (data >> 24) & 0xFF;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >>  8) & 0xFF;
  buffer[3] = (data >>  0) & 0xFF;
  status = VL53L1_I2CWrite(Dev->I2cDevAddr, index, (uint8_t *)buffer, 4);
  return status;
}

VL53L1_Error VL53L1::VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data)
{
  int  status;

  status = VL53L1_I2CRead(Dev->I2cDevAddr, index, data, 1);

  if (status) {
    return -1;
  }

  return 0;
}

VL53L1_Error VL53L1::VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data)
{
  int  status;
  uint8_t buffer[2] = {0, 0};

  status = VL53L1_I2CRead(Dev->I2cDevAddr, index, buffer, 2);
  if (!status) {
    *data = (buffer[0] << 8) + buffer[1];
  }
  return status;
}

VL53L1_Error VL53L1::VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data)
{
  int status;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = VL53L1_I2CRead(Dev->I2cDevAddr, index, buffer, 4);
  if (!status) {
    *data = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
  }
  return status;
}

VL53L1_Error VL53L1::VL53L1_UpdateByte(VL53L1_DEV Dev, uint16_t index, uint8_t AndData, uint8_t OrData)
{
  int  status;
  uint8_t buffer = 0;

  /* read data direct onto buffer */
  status = VL53L1_I2CRead(Dev->I2cDevAddr, index, &buffer, 1);
  if (!status) {
    buffer = (buffer & AndData) | OrData;
    status = VL53L1_I2CWrite(Dev->I2cDevAddr, index, &buffer, (uint16_t)1);
  }
  return status;
}

VL53L1_Error VL53L1::VL53L1_WriteMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int  status;

  status = VL53L1_I2CWrite(Dev->I2cDevAddr, index, pdata, (uint16_t)count);
  return status;
}

VL53L1_Error VL53L1::VL53L1_ReadMulti(VL53L1_DEV Dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  int status;

  status = VL53L1_I2CRead(Dev->I2cDevAddr, index, pdata, (uint16_t)count);

  return status;
}

VL53L1_Error VL53L1::VL53L1_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite)
{
#ifdef DEBUG_MODE
  Serial.print("Beginning transmission to ");
  Serial.println(((DeviceAddr) >> 1) & 0x7F);
#endif
  dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
#ifdef DEBUG_MODE
  Serial.print("Writing port number ");
  Serial.println(RegisterAddr);
#endif
  const uint8_t buffer[2] {RegisterAddr >> 8, RegisterAddr & 0xFF };
  dev_i2c->write(buffer, 2);
  for (int i = 0 ; i < NumByteToWrite ; i++) {
    dev_i2c->write(pBuffer[i]);
  }

  dev_i2c->endTransmission(true);
  return 0;
}

VL53L1_Error VL53L1::VL53L1_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead)
{
  int status = 0;
  //Loop until the port is transmitted correctly
  do {
#ifdef DEBUG_MODE
    Serial.print("Beginning transmission to ");
    Serial.println(((DeviceAddr) >> 1) & 0x7F);
#endif
    dev_i2c->beginTransmission(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)));
#ifdef DEBUG_MODE
    Serial.print("Reading port number ");
    Serial.println(RegisterAddr);
#endif
    const uint8_t buffer[2] {RegisterAddr >> 8, RegisterAddr & 0xFF };
    dev_i2c->write(buffer, 2);
    status = dev_i2c->endTransmission(false);
    //Fix for some STM32 boards
    //Reinitialize th i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
    if (status) {
      dev_i2c->end();
      dev_i2c->begin();
    }
#endif
    //End of fix
  } while (status != 0);

  dev_i2c->requestFrom(((uint8_t)(((DeviceAddr) >> 1) & 0x7F)), (byte) NumByteToRead);

  int i = 0;
  while (dev_i2c->available()) {
    pBuffer[i] = dev_i2c->read();
    i++;
  }

  return 0;
}

VL53L1_Error VL53L1::VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
  /* Returns current tick count in [ms] */

  VL53L1_Error status  = VL53L1_ERROR_NONE;

  //*ptick_count_ms = timeGetTime();
  *ptick_count_ms = 0;

  return status;
}

VL53L1_Error VL53L1::VL53L1_WaitUs(VL53L1_Dev_t *pdev, int32_t wait_us)
{
  /* Currently, the largest value that will produce an accurate delay is 16383.
   * This could change in future Arduino releases.
   * https://www.arduino.cc/reference/en/language/functions/time/delaymicroseconds/
   */
  if (wait_us > 16383) {
    VL53L1_WaitMs(pdev, wait_us / 1000);
    delayMicroseconds(wait_us % 1000);
  } else {
    delayMicroseconds(wait_us);
  }
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1::VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms)
{
  (void)pdev;
  delay(wait_ms);
  return VL53L1_ERROR_NONE;
}

VL53L1_Error VL53L1::VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t index, uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
  /*
   * Platform implementation of WaitValueMaskEx V2WReg script command
   *
   * WaitValueMaskEx(
   *          duration_ms,
   *          index,
   *          value,
   *          mask,
   *          poll_delay_ms);
   */

  VL53L1_Error status         = VL53L1_ERROR_NONE;
  uint32_t     start_time_ms = 0;
  uint32_t     current_time_ms = 0;
  uint32_t     polling_time_ms = 0;
  uint8_t      byte_value      = 0;
  uint8_t      found           = 0;



  /* calculate time limit in absolute time */

  VL53L1_GetTickCount(&start_time_ms);

  /* remember current trace functions and temporarily disable
   * function logging
   */


  /* wait until value is found, timeout reached on error occurred */

  while ((status == VL53L1_ERROR_NONE) &&
         (polling_time_ms < timeout_ms) &&
         (found == 0)) {

    if (status == VL53L1_ERROR_NONE)
      status = VL53L1_RdByte(
                 pdev,
                 index,
                 &byte_value);

    if ((byte_value & mask) == value) {
      found = 1;
    }

    if (status == VL53L1_ERROR_NONE  &&
        found == 0 &&
        poll_delay_ms > 0)
      status = VL53L1_WaitMs(
                 pdev,
                 poll_delay_ms);

    /* Update polling time (Compare difference rather than absolute to
       negate 32bit wrap around issue) */
    VL53L1_GetTickCount(&current_time_ms);
    polling_time_ms = current_time_ms - start_time_ms;

  }


  if (found == 0 && status == VL53L1_ERROR_NONE) {
    status = VL53L1_ERROR_TIME_OUT;
  }

  return status;
}


