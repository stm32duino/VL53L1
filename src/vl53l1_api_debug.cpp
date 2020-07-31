
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


VL53L1_Error VL53L1::VL53L1_decode_calibration_data_buffer(
	uint16_t                   buf_size,
	uint8_t                   *pbuffer,
	VL53L1_calibration_data_t *pdata)
{
	VL53L1_Error  status = VL53L1_ERROR_NONE;

	

	if (sizeof(VL53L1_calibration_data_t) > buf_size)
		return VL53L1_ERROR_COMMS_BUFFER_TOO_SMALL;

	memcpy(pdata, pbuffer, sizeof(VL53L1_calibration_data_t));

	

	return status;
}


VL53L1_Error VL53L1::VL53L1_get_nvm_debug_data(
	VL53L1_DEV                          Dev,
	VL53L1_decoded_nvm_data_t          *pdata)
{


	VL53L1_Error  status = VL53L1_ERROR_NONE;

	

	status = VL53L1_read_nvm(Dev, 0, pdata);

	

	return status;
}


VL53L1_Error VL53L1::VL53L1_get_histogram_debug_data(
	VL53L1_DEV                          Dev,
	VL53L1_histogram_bin_data_t        *pdata)
{


	VL53L1_Error  status = VL53L1_ERROR_NONE;

	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);

	


	memcpy(
		pdata,
		&(pdev->hist_data),
		sizeof(VL53L1_histogram_bin_data_t));

	

	return status;
}




VL53L1_Error VL53L1::VL53L1_get_additional_data(
	VL53L1_DEV                       Dev,
	VL53L1_additional_data_t        *pdata)
{


	VL53L1_Error  status = VL53L1_ERROR_NONE;

	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);

	



	pdata->preset_mode             = pdev->preset_mode;
	pdata->zone_preset             = pdev->zone_preset;
	pdata->measurement_mode        = pdev->measurement_mode;
	pdata->offset_calibration_mode = pdev->offset_calibration_mode;
	pdata->offset_correction_mode  = pdev->offset_correction_mode;
	pdata->dmax_mode               = pdev->dmax_mode;

	pdata->phasecal_config_timeout_us  = pdev->phasecal_config_timeout_us;
	pdata->mm_config_timeout_us        = pdev->mm_config_timeout_us;
	pdata->range_config_timeout_us     = pdev->range_config_timeout_us;
	pdata->inter_measurement_period_ms = pdev->inter_measurement_period_ms;
	pdata->dss_config__target_total_rate_mcps =
			pdev->dss_config__target_total_rate_mcps;



	status =
		VL53L1_get_histogram_debug_data(
			Dev,
			&(pdata->VL53L1_p_010));

	

	return status;
}




VL53L1_Error VL53L1::VL53L1_get_xtalk_debug_data(
	VL53L1_DEV                          Dev,
	VL53L1_xtalk_debug_data_t          *pdata)
{


	VL53L1_Error  status = VL53L1_ERROR_NONE;

	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);

	


	memcpy(
		&(pdata->customer),
		&(pdev->customer),
		sizeof(VL53L1_customer_nvm_managed_t));

	memcpy(
		&(pdata->xtalk_cfg),
		&(pdev->xtalk_cfg),
		sizeof(VL53L1_xtalk_config_t));

	memcpy(
		&(pdata->hist_data),
		&(pdev->hist_data),
		sizeof(VL53L1_histogram_bin_data_t));

	memcpy(
		&(pdata->xtalk_shapes),
		&(pdev->xtalk_shapes),
		sizeof(VL53L1_xtalk_histogram_data_t));

	memcpy(
		&(pdata->xtalk_results),
		&(pdev->xtalk_results),
		sizeof(VL53L1_xtalk_range_results_t));

	

	return status;
}


VL53L1_Error VL53L1::VL53L1_get_offset_debug_data(
	VL53L1_DEV                          Dev,
	VL53L1_offset_debug_data_t         *pdata)
{


	VL53L1_Error  status = VL53L1_ERROR_NONE;

	VL53L1_LLDriverData_t *pdev = VL53L1DevStructGetLLDriverHandle(Dev);

	


	memcpy(
		&(pdata->customer),
		&(pdev->customer),
		sizeof(VL53L1_customer_nvm_managed_t));

	memcpy(
		&(pdata->fmt_dmax_cal),
		&(pdev->fmt_dmax_cal),
		sizeof(VL53L1_dmax_calibration_data_t));

	memcpy(
		&(pdata->cust_dmax_cal),
		&(pdev->cust_dmax_cal),
		sizeof(VL53L1_dmax_calibration_data_t));

	memcpy(
		&(pdata->add_off_cal_data),
		&(pdev->add_off_cal_data),
		sizeof(VL53L1_additional_offset_cal_data_t));

	memcpy(
		&(pdata->offset_results),
		&(pdev->offset_results),
		sizeof(VL53L1_offset_range_results_t));

	

	return status;
}

