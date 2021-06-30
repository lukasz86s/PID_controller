/*
 * vl53l0x.c
 *
 *  Created on: Jun 18, 2021
 *
 */
#include "vl53l0x.h"
#include <stdio.h>


uint16_t ioTimeout = 0; // if timeout == 0 is off
uint32_t g_measTimBudUs;

I2C_HandleTypeDef *i2c_1 = &hi2c1;;
//todo: remove hit2c from func , create file to register sending func
HAL_StatusTypeDef (*VL53L0X_REG_I2C_Mem_Write)(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) = HAL_I2C_Mem_Write;
HAL_StatusTypeDef (*VL53L0X_REG_I2C_Mem_Read)(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout) = HAL_I2C_Mem_Read;

// functions

/*private func------------------*/
bool setSignalRateLimit(float Mcps);
bool getSpadInfo(uint8_t *count, bool *type_is_aperture);
uint32_t getMeasurementTimingBudget(void);
void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
uint8_t getVcselPulsePeriod(vcselPeriodType type);
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
bool performSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
bool setMeasurementTimingBudget(uint32_t budget_us);
/*--------------------------*/


void vl53l0x_I2C_Write_Reg8(uint8_t addr, uint8_t value){
	VL53L0X_REG_I2C_Mem_Write(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, &value,1 , I2C_TIMEOUT);

}
void vl53l0x_I2C_Write_Reg16(uint8_t addr, uint16_t value){
	uint8_t temp[2];
	temp[0] =(value>>8) & 0xff; //hi byte
	temp[1] = value & 0xff;		// lo byte
	VL53L0X_REG_I2C_Mem_Write(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, temp,2 , I2C_TIMEOUT);

}

void vl53l0x_I2C_Write_Reg32(uint8_t addr, uint32_t value){
	uint8_t temp[4];
	temp[0] =(value>>24) & 0xff; //hi byte
	temp[1] =(value>>16) & 0xff;
	temp[2] =(value>>8) & 0xff;
	temp[3] = value & 0xff;	// lo byte
	VL53L0X_REG_I2C_Mem_Write(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, temp,4 , I2C_TIMEOUT);

}

void vl53l0x_I2C_Write_MultiReg(uint8_t addr, uint8_t *pData, uint8_t count){

	VL53L0X_REG_I2C_Mem_Write(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, pData, count, I2C_TIMEOUT);
}

uint8_t vl53l0x_I2C_Read_Reg8(uint8_t addr){
	uint8_t value;
	VL53L0X_REG_I2C_Mem_Read(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, &value  ,1 , I2C_TIMEOUT);
	return value;
}

uint8_t vl53l0x_I2C_Read_Reg8_Timeout(uint8_t addr, uint8_t timeOut){
	uint8_t value;
	VL53L0X_REG_I2C_Mem_Read(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, &value  ,1 , timeOut);
	return value;
}

uint16_t vl53l0x_I2C_Read_Reg16(uint8_t addr){
	uint16_t value = 0;
	uint8_t temp[2] = {0};
	VL53L0X_REG_I2C_Mem_Read(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, temp  ,2 , I2C_TIMEOUT);
	value |= (temp[0]<<8);
	value |= temp[1];
	return value;
}

uint32_t vl53l0x_I2C_Read_Reg32(uint8_t addr){
	uint32_t value = 0;
	uint8_t temp[4] = {0};
	VL53L0X_REG_I2C_Mem_Read(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, temp ,4 , I2C_TIMEOUT);
	value |= (temp[0]<<24);
	value |= (temp[1]<<16);
	value |= (temp[2]<<8);
	value |= temp[3];
	return value;
}
/* reading multiple data*/
void vl53l0x_I2C_Read_MultiReg(uint8_t addr, uint8_t *pData, uint8_t count){
	VL53L0X_REG_I2C_Mem_Read(i2c_1, ADDRESS_DEFAULT_SHIFTED, (uint16_t)(addr), 1, pData ,count , I2C_TIMEOUT);

}

uint8_t init_stop_var;		// read in intin used when starting measurment
/*public methods*/
bool vl53l0x_Init(bool io_2v8){


	if(io_2v8){
		vl53l0x_I2C_Write_Reg8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
				vl53l0x_I2C_Read_Reg8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
	}
	// set i2c standart mode
	vl53l0x_I2C_Write_Reg8(0x88, 0x00);
	vl53l0x_I2C_Write_Reg8(0x80, 0x01);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x00, 0x00);

	init_stop_var = vl53l0x_I2C_Read_Reg8(0x091);

	vl53l0x_I2C_Write_Reg8(0x00, 0x01);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x80, 0x00);

	//disable signal rate msrc bit 1 and signal rate pre range bit 4 limit checks

	vl53l0x_I2C_Write_Reg8(MSRC_CONFIG_CONTROL,
			vl53l0x_I2C_Read_Reg8(MSRC_CONFIG_CONTROL) | 0x12);
	// set rate signal limit Mcps( milion counts per second)
	setSignalRateLimit(0.25);

	vl53l0x_I2C_Write_Reg8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// staticInit
	uint8_t spad_count;
	bool  spad_type_is_aperture; //
	if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) return false;

	//The SPAD map
	uint8_t ref_spad_map[6];
	vl53l0x_I2C_Read_MultiReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// vl53l0x_set_reference_spad() in api --start

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	vl53l0x_I2C_Write_Reg8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;

	for (uint8_t i = 0; i < 48; i++)
	{
		if (i < first_spad_to_enable || spads_enabled == spad_count)
		{
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		}
		else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
		{
			spads_enabled++;
		}
	}

	vl53l0x_I2C_Read_MultiReg(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// VL53L0X_set_reference_spads() --end

	//VL53L0X_load_tuning_settings() --start

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x00, 0x00);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x09, 0x00);
	vl53l0x_I2C_Write_Reg8(0x10, 0x00);
	vl53l0x_I2C_Write_Reg8(0x11, 0x00);

	vl53l0x_I2C_Write_Reg8(0x24, 0x01);
	vl53l0x_I2C_Write_Reg8(0x25, 0xFF);
	vl53l0x_I2C_Write_Reg8(0x75, 0x00);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x4E, 0x2C);
	vl53l0x_I2C_Write_Reg8(0x48, 0x00);
	vl53l0x_I2C_Write_Reg8(0x30, 0x20);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x30, 0x09);
	vl53l0x_I2C_Write_Reg8(0x54, 0x00);
	vl53l0x_I2C_Write_Reg8(0x31, 0x04);
	vl53l0x_I2C_Write_Reg8(0x32, 0x03);
	vl53l0x_I2C_Write_Reg8(0x40, 0x83);
	vl53l0x_I2C_Write_Reg8(0x46, 0x25);
	vl53l0x_I2C_Write_Reg8(0x60, 0x00);
	vl53l0x_I2C_Write_Reg8(0x27, 0x00);
	vl53l0x_I2C_Write_Reg8(0x50, 0x06);
	vl53l0x_I2C_Write_Reg8(0x51, 0x00);
	vl53l0x_I2C_Write_Reg8(0x52, 0x96);
	vl53l0x_I2C_Write_Reg8(0x56, 0x08);
	vl53l0x_I2C_Write_Reg8(0x57, 0x30);
	vl53l0x_I2C_Write_Reg8(0x61, 0x00);
	vl53l0x_I2C_Write_Reg8(0x62, 0x00);
	vl53l0x_I2C_Write_Reg8(0x64, 0x00);
	vl53l0x_I2C_Write_Reg8(0x65, 0x00);
	vl53l0x_I2C_Write_Reg8(0x66, 0xA0);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x22, 0x32);
	vl53l0x_I2C_Write_Reg8(0x47, 0x14);
	vl53l0x_I2C_Write_Reg8(0x49, 0xFF);
	vl53l0x_I2C_Write_Reg8(0x4A, 0x00);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x7A, 0x0A);
	vl53l0x_I2C_Write_Reg8(0x7B, 0x00);
	vl53l0x_I2C_Write_Reg8(0x78, 0x21);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x23, 0x34);
	vl53l0x_I2C_Write_Reg8(0x42, 0x00);
	vl53l0x_I2C_Write_Reg8(0x44, 0xFF);
	vl53l0x_I2C_Write_Reg8(0x45, 0x26);
	vl53l0x_I2C_Write_Reg8(0x46, 0x05);
	vl53l0x_I2C_Write_Reg8(0x40, 0x40);
	vl53l0x_I2C_Write_Reg8(0x0E, 0x06);
	vl53l0x_I2C_Write_Reg8(0x20, 0x1A);
	vl53l0x_I2C_Write_Reg8(0x43, 0x40);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x34, 0x03);
	vl53l0x_I2C_Write_Reg8(0x35, 0x44);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x31, 0x04);
	vl53l0x_I2C_Write_Reg8(0x4B, 0x09);
	vl53l0x_I2C_Write_Reg8(0x4C, 0x05);
	vl53l0x_I2C_Write_Reg8(0x4D, 0x04);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x44, 0x00);
	vl53l0x_I2C_Write_Reg8(0x45, 0x20);
	vl53l0x_I2C_Write_Reg8(0x47, 0x08);
	vl53l0x_I2C_Write_Reg8(0x48, 0x28);
	vl53l0x_I2C_Write_Reg8(0x67, 0x00);
	vl53l0x_I2C_Write_Reg8(0x70, 0x04);
	vl53l0x_I2C_Write_Reg8(0x71, 0x01);
	vl53l0x_I2C_Write_Reg8(0x72, 0xFE);
	vl53l0x_I2C_Write_Reg8(0x76, 0x00);
	vl53l0x_I2C_Write_Reg8(0x77, 0x00);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x0D, 0x01);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x80, 0x01);
	vl53l0x_I2C_Write_Reg8(0x01, 0xF8);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x8E, 0x01);
	vl53l0x_I2C_Write_Reg8(0x00, 0x01);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x80, 0x00);

	  //VL53L0X_load_tuning_settings() --end

	 // "Set interrupt config to new sample ready"
	  // VL53L0X_SetGpioConfig() -- start

	 vl53l0x_I2C_Write_Reg8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	 vl53l0x_I2C_Write_Reg8(GPIO_HV_MUX_ACTIVE_HIGH, vl53l0x_I2C_Read_Reg8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	 vl53l0x_I2C_Write_Reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);
	 // VL53L0X_SetGpioConfig() -- end

	 g_measTimBudUs = getMeasurementTimingBudget();

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// VL53L0X_SetSequenceStepEnable() -- start

	 vl53l0x_I2C_Write_Reg8(SYSTEM_SEQUENCE_CONFIG, 0xE8);
	  //VL53L0X_SetSequenceStepEnable() --end

	// "Recalculate timing budget"
	setMeasurementTimingBudget(g_measTimBudUs);
	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin

	vl53l0x_I2C_Write_Reg8(SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!performSingleRefCalibration(0x40)) { return false; }

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	vl53l0x_I2C_Write_Reg8(SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!performSingleRefCalibration(0x00)) { return false; }

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	vl53l0x_I2C_Write_Reg8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// VL53L0X_PerformRefCalibration() end

	 return true;
}

/* private functions */

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    vl53l0x_I2C_Write_Reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    g_measTimBudUs = budget_us; // store for internal reuse
  }
  return true;
}
// Mcps - mega counts per second
bool setSignalRateLimit(float Mcps){

	if(Mcps < 0 || Mcps > 511.99)return false;
	vl53l0x_I2C_Write_Reg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, Mcps *(1<<7));
	return true;
}
/* Get feference SPAD count and type
 * based on vl53l0x_get_info_from_divice()*/
bool getSpadInfo(uint8_t *count, bool *type_is_aperture){

	uint8_t tmp;
	vl53l0x_I2C_Write_Reg8(0x80, 0x01);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x00, 0x00);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x06);
	vl53l0x_I2C_Write_Reg8(0x83, vl53l0x_I2C_Read_Reg8(0x83) | 0x04);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x07);
	vl53l0x_I2C_Write_Reg8(0x81, 0x01);

	vl53l0x_I2C_Write_Reg8(0x80, 0x01);

	vl53l0x_I2C_Write_Reg8(0x94, 0x6b);
	vl53l0x_I2C_Write_Reg8(0x83, 0x00);

	while(vl53l0x_I2C_Read_Reg8(0x83) == 0x00){ //while change
		// add timeout function
	}
	vl53l0x_I2C_Write_Reg8(0x83, 0x01);
	tmp = vl53l0x_I2C_Read_Reg8(0x92);

	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;

	vl53l0x_I2C_Write_Reg8(0x81, 0x00);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x06);
	vl53l0x_I2C_Write_Reg8(0x83, vl53l0x_I2C_Read_Reg8(0x83)  & ~0x04);
	vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
	vl53l0x_I2C_Write_Reg8(0x00, 0x01);

	vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
	vl53l0x_I2C_Write_Reg8(0x80, 0x00);

	return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(void)
{
	SequenceStepEnables enables;
	SequenceStepTimeouts timeouts;

	uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
	uint16_t const EndOverhead        = 960;
	uint16_t const MsrcOverhead       = 660;
	uint16_t const TccOverhead        = 590;
	uint16_t const DssOverhead        = 690;
	uint16_t const PreRangeOverhead   = 660;
	uint16_t const FinalRangeOverhead = 550;

	// "Start and end overhead times always present"
	uint32_t budget_us = StartOverhead + EndOverhead;

	getSequenceStepEnables(&enables);
	getSequenceStepTimeouts(&enables, &timeouts);

	if (enables.tcc)
	{
	budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	}

	if (enables.dss)
	{
	budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	}
	else if (enables.msrc)
	{
	budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	}

	if (enables.pre_range)
	{
	budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	}

	if (enables.final_range)
	{
	budget_us += (timeouts.final_range_us + FinalRangeOverhead);
	}

	g_measTimBudUs = budget_us; // store for internal reuse
	return budget_us;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = vl53l0x_I2C_Read_Reg8(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = vl53l0x_I2C_Read_Reg8(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(vl53l0x_I2C_Read_Reg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(vl53l0x_I2C_Read_Reg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(vl53l0x_I2C_Read_Reg8(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(vl53l0x_I2C_Read_Reg8(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}
// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
  vl53l0x_I2C_Write_Reg8(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP


  while ((vl53l0x_I2C_Read_Reg8(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    // add timeout func
  }

  vl53l0x_I2C_Write_Reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);

  vl53l0x_I2C_Write_Reg8(SYSRANGE_START, 0x00);

  return true;
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t vl53l0x_ReadRangeContinuousMillimeters( statInfo_t *extraStats ) {
  uint8_t tempBuffer[12];
  uint16_t temp;
  //todo: func ->startTimeout();
  while ((vl53l0x_I2C_Read_Reg8(RESULT_INTERRUPT_STATUS) & 0x07) == 0) { // wait for mesure complet
   // if (checkTimeoutExpired())
   // {
    //  g_isTimeout = true;
     // return 65535;
    //}
  }
  if( extraStats == 0 ){
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    temp = vl53l0x_I2C_Read_Reg16(RESULT_RANGE_STATUS + 10);
  } else {
    // Register map starting at 0x14
    //     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
    //    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
    //   0: Ranging status, uint8_t
    //   1: ???
    // 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
    //   4: 0 ?
    //   5: ???
    // 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
    // 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
    // A,B: uncorrected distance [mm], uint16_t
	vl53l0x_I2C_Read_MultiReg(0x14, tempBuffer, 12);
    extraStats->rangeStatus =  tempBuffer[0x00]>>3;
    extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
    extraStats->signalCnt   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
    extraStats->ambientCnt  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];
    temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
    extraStats->rawDistance = temp;
  }
  vl53l0x_I2C_Write_Reg8(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return temp;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t vl53l0x_ReadRangeSingleMillimeters( statInfo_t *extraStats ) {
  vl53l0x_I2C_Write_Reg8(0x80, 0x01);
  vl53l0x_I2C_Write_Reg8(0xFF, 0x01);
  vl53l0x_I2C_Write_Reg8(0x00, 0x00);
  vl53l0x_I2C_Write_Reg8(0x91, init_stop_var);
  vl53l0x_I2C_Write_Reg8(0x00, 0x01);
  vl53l0x_I2C_Write_Reg8(0xFF, 0x00);
  vl53l0x_I2C_Write_Reg8(0x80, 0x00);
  vl53l0x_I2C_Write_Reg8(SYSRANGE_START, 0x01);
  // "Wait until start bit has been cleared"
  //todo: function startTimeout();
  while (vl53l0x_I2C_Read_Reg8(SYSRANGE_START) & 0x01){
	  //todo: timeout
    //if (checkTimeoutExpired()){
     // g_isTimeout = true;
      //return 65535;
    //}
  }
  return vl53l0x_ReadRangeContinuousMillimeters( extraStats );
}
