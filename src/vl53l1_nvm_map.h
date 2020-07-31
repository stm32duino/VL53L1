
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







#ifndef _VL53L1_NVM_MAP_H_
#define _VL53L1_NVM_MAP_H_


#ifdef __cplusplus
extern "C"
{
#endif




#define VL53L1_NVM__IDENTIFICATION__MODEL_ID 0x0008

#define VL53L1_NVM__IDENTIFICATION__MODULE_TYPE 0x000C

#define VL53L1_NVM__IDENTIFICATION__REVISION_ID 0x000D

#define VL53L1_NVM__IDENTIFICATION__MODULE_ID 0x000E

#define VL53L1_NVM__I2C_VALID 0x0010

#define VL53L1_NVM__I2C_SLAVE__DEVICE_ADDRESS 0x0011

#define VL53L1_NVM__EWS__OSC_MEASURED__FAST_OSC_FREQUENCY 0x0014

#define VL53L1_NVM__EWS__FAST_OSC_TRIM_MAX 0x0016

#define VL53L1_NVM__EWS__FAST_OSC_FREQ_SET 0x0017

#define VL53L1_NVM__EWS__SLOW_OSC_CALIBRATION 0x0018

#define VL53L1_NVM__FMT__OSC_MEASURED__FAST_OSC_FREQUENCY 0x001C

#define VL53L1_NVM__FMT__FAST_OSC_TRIM_MAX 0x001E

#define VL53L1_NVM__FMT__FAST_OSC_FREQ_SET 0x001F

#define VL53L1_NVM__FMT__SLOW_OSC_CALIBRATION 0x0020

#define VL53L1_NVM__VHV_CONFIG_UNLOCK 0x0028

#define VL53L1_NVM__REF_SELVDDPIX 0x0029

#define VL53L1_NVM__REF_SELVQUENCH 0x002A

#define VL53L1_NVM__REGAVDD1V2_SEL_REGDVDD1V2_SEL 0x002B

#define VL53L1_NVM__VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND 0x002C

#define VL53L1_NVM__VHV_CONFIG__COUNT_THRESH 0x002D

#define VL53L1_NVM__VHV_CONFIG__OFFSET 0x002E

#define VL53L1_NVM__VHV_CONFIG__INIT 0x002F

#define VL53L1_NVM__LASER_SAFETY__VCSEL_TRIM_LL 0x0030

#define VL53L1_NVM__LASER_SAFETY__VCSEL_SELION_LL 0x0031

#define VL53L1_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LL 0x0032

#define VL53L1_NVM__LASER_SAFETY__MULT_LL 0x0034

#define VL53L1_NVM__LASER_SAFETY__CLIP_LL 0x0035

#define VL53L1_NVM__LASER_SAFETY__VCSEL_TRIM_LD 0x0038

#define VL53L1_NVM__LASER_SAFETY__VCSEL_SELION_LD 0x0039

#define VL53L1_NVM__LASER_SAFETY__VCSEL_SELION_MAX_LD 0x003A

#define VL53L1_NVM__LASER_SAFETY__MULT_LD 0x003C

#define VL53L1_NVM__LASER_SAFETY__CLIP_LD 0x003D

#define VL53L1_NVM__LASER_SAFETY_LOCK_BYTE 0x0040

#define VL53L1_NVM__LASER_SAFETY_UNLOCK_BYTE 0x0044

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_0_ 0x0048

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_1_ 0x0049

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_2_ 0x004A

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_3_ 0x004B

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_4_ 0x004C

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_5_ 0x004D

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_6_ 0x004E

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_7_ 0x004F

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_8_ 0x0050

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_9_ 0x0051

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_10_ 0x0052

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_11_ 0x0053

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_12_ 0x0054

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_13_ 0x0055

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_14_ 0x0056

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_15_ 0x0057

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_16_ 0x0058

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_17_ 0x0059

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_18_ 0x005A

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_19_ 0x005B

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_20_ 0x005C

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_21_ 0x005D

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_22_ 0x005E

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_23_ 0x005F

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_24_ 0x0060

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_25_ 0x0061

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_26_ 0x0062

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_27_ 0x0063

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_28_ 0x0064

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_29_ 0x0065

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_30_ 0x0066

#define VL53L1_NVM__EWS__SPAD_ENABLES_RTN_31_ 0x0067

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_0_ 0x0068

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_1_ 0x0069

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_2_ 0x006A

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_3_ 0x006B

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_4_ 0x006C

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC1_5_ 0x006D

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_0_ 0x0070

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_1_ 0x0071

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_2_ 0x0072

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_3_ 0x0073

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_4_ 0x0074

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC2_5_ 0x0075

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_0_ 0x0078

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_1_ 0x0079

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_2_ 0x007A

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_3_ 0x007B

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_4_ 0x007C

#define VL53L1_NVM__EWS__SPAD_ENABLES_REF__LOC3_5_ 0x007D

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_0_ 0x0080

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_1_ 0x0081

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_2_ 0x0082

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_3_ 0x0083

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_4_ 0x0084

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_5_ 0x0085

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_6_ 0x0086

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_7_ 0x0087

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_8_ 0x0088

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_9_ 0x0089

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_10_ 0x008A

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_11_ 0x008B

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_12_ 0x008C

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_13_ 0x008D

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_14_ 0x008E

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_15_ 0x008F

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_16_ 0x0090

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_17_ 0x0091

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_18_ 0x0092

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_19_ 0x0093

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_20_ 0x0094

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_21_ 0x0095

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_22_ 0x0096

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_23_ 0x0097

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_24_ 0x0098

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_25_ 0x0099

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_26_ 0x009A

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_27_ 0x009B

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_28_ 0x009C

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_29_ 0x009D

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_30_ 0x009E

#define VL53L1_NVM__FMT__SPAD_ENABLES_RTN_31_ 0x009F

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_0_ 0x00A0

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_1_ 0x00A1

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_2_ 0x00A2

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_3_ 0x00A3

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_4_ 0x00A4

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC1_5_ 0x00A5

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_0_ 0x00A8

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_1_ 0x00A9

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_2_ 0x00AA

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_3_ 0x00AB

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_4_ 0x00AC

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC2_5_ 0x00AD

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_0_ 0x00B0

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_1_ 0x00B1

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_2_ 0x00B2

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_3_ 0x00B3

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_4_ 0x00B4

#define VL53L1_NVM__FMT__SPAD_ENABLES_REF__LOC3_5_ 0x00B5

#define VL53L1_NVM__FMT__ROI_CONFIG__MODE_ROI_CENTRE_SPAD 0x00B8

#define VL53L1_NVM__FMT__ROI_CONFIG__MODE_ROI_XY_SIZE 0x00B9

#define VL53L1_NVM__FMT__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD 0x00BC

#define VL53L1_NVM__FMT__REF_SPAD_MAN__REF_LOCATION 0x00BD

#define VL53L1_NVM__FMT__MM_CONFIG__INNER_OFFSET_MM 0x00C0

#define VL53L1_NVM__FMT__MM_CONFIG__OUTER_OFFSET_MM 0x00C2

#define VL53L1_NVM__FMT__ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x00C4

#define VL53L1_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x00C8

#define VL53L1_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS \
	0x00CA

#define VL53L1_NVM__FMT__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS \
	0x00CC

#define VL53L1_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0 0x00CE

#define VL53L1_NVM__FMT__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1 0x00CF

#define VL53L1_NVM__CUSTOMER_NVM_SPACE_PROGRAMMED 0x00E0

#define VL53L1_NVM__CUST__I2C_SLAVE__DEVICE_ADDRESS 0x00E4

#define VL53L1_NVM__CUST__REF_SPAD_APPLY__NUM_REQUESTED_REF_SPAD 0x00E8

#define VL53L1_NVM__CUST__REF_SPAD_MAN__REF_LOCATION 0x00E9

#define VL53L1_NVM__CUST__MM_CONFIG__INNER_OFFSET_MM 0x00EC

#define VL53L1_NVM__CUST__MM_CONFIG__OUTER_OFFSET_MM 0x00EE

#define VL53L1_NVM__CUST__ALGO__PART_TO_PART_RANGE_OFFSET_MM 0x00F0

#define VL53L1_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 0x00F4

#define VL53L1_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS \
	0x00F6

#define VL53L1_NVM__CUST__ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS \
	0x00F8

#define VL53L1_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_0 0x00FA

#define VL53L1_NVM__CUST__SPARE_HOST_CONFIG__NVM_CONFIG_SPARE_1 0x00FB

#define VL53L1_NVM__FMT__FGC__BYTE_0 0x01DC

#define VL53L1_NVM__FMT__FGC__BYTE_1 0x01DD

#define VL53L1_NVM__FMT__FGC__BYTE_2 0x01DE

#define VL53L1_NVM__FMT__FGC__BYTE_3 0x01DF

#define VL53L1_NVM__FMT__FGC__BYTE_4 0x01E0

#define VL53L1_NVM__FMT__FGC__BYTE_5 0x01E1

#define VL53L1_NVM__FMT__FGC__BYTE_6 0x01E2

#define VL53L1_NVM__FMT__FGC__BYTE_7 0x01E3

#define VL53L1_NVM__FMT__FGC__BYTE_8 0x01E4

#define VL53L1_NVM__FMT__FGC__BYTE_9 0x01E5

#define VL53L1_NVM__FMT__FGC__BYTE_10 0x01E6

#define VL53L1_NVM__FMT__FGC__BYTE_11 0x01E7

#define VL53L1_NVM__FMT__FGC__BYTE_12 0x01E8

#define VL53L1_NVM__FMT__FGC__BYTE_13 0x01E9

#define VL53L1_NVM__FMT__FGC__BYTE_14 0x01EA

#define VL53L1_NVM__FMT__FGC__BYTE_15 0x01EB

#define VL53L1_NVM__FMT__TEST_PROGRAM_MAJOR_MINOR 0x01EC

#define VL53L1_NVM__FMT__MAP_MAJOR_MINOR 0x01ED

#define VL53L1_NVM__FMT__YEAR_MONTH 0x01EE

#define VL53L1_NVM__FMT__DAY_MODULE_DATE_PHASE 0x01EF

#define VL53L1_NVM__FMT__TIME 0x01F0

#define VL53L1_NVM__FMT__TESTER_ID 0x01F2

#define VL53L1_NVM__FMT__SITE_ID 0x01F3

#define VL53L1_NVM__EWS__TEST_PROGRAM_MAJOR_MINOR 0x01F4

#define VL53L1_NVM__EWS__PROBE_CARD_MAJOR_MINOR 0x01F5

#define VL53L1_NVM__EWS__TESTER_ID 0x01F6

#define VL53L1_NVM__EWS__LOT__BYTE_0 0x01F8

#define VL53L1_NVM__EWS__LOT__BYTE_1 0x01F9

#define VL53L1_NVM__EWS__LOT__BYTE_2 0x01FA

#define VL53L1_NVM__EWS__LOT__BYTE_3 0x01FB

#define VL53L1_NVM__EWS__LOT__BYTE_4 0x01FC

#define VL53L1_NVM__EWS__LOT__BYTE_5 0x01FD

#define VL53L1_NVM__EWS__WAFER 0x01FD

#define VL53L1_NVM__EWS__XCOORD 0x01FE

#define VL53L1_NVM__EWS__YCOORD 0x01FF


#define VL53L1_NVM__FMT__OPTICAL_CENTRE_DATA_INDEX 0x00B8
#define VL53L1_NVM__FMT__OPTICAL_CENTRE_DATA_SIZE      4

#define VL53L1_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_INDEX 0x015C
#define VL53L1_NVM__FMT__CAL_PEAK_RATE_MAP_DATA_SIZE   56

#define VL53L1_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_INDEX 0x0194
#define VL53L1_NVM__FMT__ADDITIONAL_OFFSET_CAL_DATA_SIZE   8

#define VL53L1_NVM__FMT__RANGE_RESULTS__140MM_MM_PRE_RANGE 0x019C
#define VL53L1_NVM__FMT__RANGE_RESULTS__140MM_DARK 0x01AC
#define VL53L1_NVM__FMT__RANGE_RESULTS__400MM_DARK 0x01BC
#define VL53L1_NVM__FMT__RANGE_RESULTS__400MM_AMBIENT 0x01CC
#define VL53L1_NVM__FMT__RANGE_RESULTS__SIZE_BYTES         16







#ifdef __cplusplus
}
#endif

#endif

