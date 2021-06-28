/*
 * vl53l0x.h
 *
 *  Created on: Jun 18, 2021
 *      Author: fet
 */

#ifndef INC_VL53L0X_H_
#define INC_VL53L0X_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "i2c.h"

//#define UNUSED(x) (void)(x)
#define ADDRESS_DEFAULT 0x29 // default address 41 decimal
#define ADDRESS_DEFAULT_SHIFTED (ADDRESS_DEFAULT<<1)
#define I2C_TIMEOUT 50

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

typedef struct {
	uint8_t tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct {

	uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
	uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
	uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

// Additional info for one measurement
typedef struct{
  uint16_t rawDistance; //uncorrected distance  [mm],   uint16_t
  uint16_t signalCnt;   //Signal  Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t ambientCnt;  //Ambient Counting Rate [mcps], uint16_t, fixpoint9.7
  uint16_t spadCnt;     //Effective SPAD return count,  uint16_t, fixpoint8.8
  uint8_t  rangeStatus; //Ranging status (0-15)
} statInfo_t;
typedef enum{
	true = 1,
	false = 0
}bool;

typedef enum {
	VcselPeriodPreRange,
	VcselPeriodFinalRange
}vcselPeriodType;

enum regAddr{
		SYSRANGE_START                                = 0x00,

	      SYSTEM_THRESH_HIGH                          = 0x0C,
	      SYSTEM_THRESH_LOW                           = 0x0E,

	      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
	      SYSTEM_RANGE_CONFIG                         = 0x09,
	      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

	      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

	      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

	      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

	      RESULT_INTERRUPT_STATUS                     = 0x13,
	      RESULT_RANGE_STATUS                         = 0x14,

	      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
	      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
	      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
	      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
	      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

	      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

	      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

	      MSRC_CONFIG_CONTROL                         = 0x60,

	      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
	      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
	      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
	      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

	      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
	      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
	      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
	      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

	      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
	      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

	      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
	      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
	      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

	      SYSTEM_HISTOGRAM_BIN                        = 0x81,
	      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
	      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

	      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
	      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
	      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
	      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

	      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

	      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
	      IDENTIFICATION_MODEL_ID                     = 0xC0,
	      IDENTIFICATION_REVISION_ID                  = 0xC2,

	      OSC_CALIBRATE_VAL                           = 0xF8,

	      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
	      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

	      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
	      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
	      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
	      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

	      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

	      ALGO_PHASECAL_LIM                           = 0x30,
	      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};



void vl53l0x_I2C_Write_Reg8(uint8_t addr, uint8_t value);
void vl53l0x_I2C_Write_Reg16(uint8_t addr, uint16_t value);
void vl53l0x_I2C_Write_Reg32(uint8_t addr, uint32_t value);
void vl53l0x_I2C_Write_MultiReg(uint8_t addr, uint8_t *pData, uint8_t count);

uint8_t vl53l0x_I2C_Read_Reg8(uint8_t addr);
uint8_t vl53l0x_I2C_Read_Reg8_Timeout(uint8_t addr, uint8_t timeOut);
uint16_t vl53l0x_I2C_Read_Reg16(uint8_t addr);
uint32_t vl53l0x_I2C_Read_Reg32(uint8_t addr);
void vl53l0x_I2C_Read_MultiReg(uint8_t addr, uint8_t *pData, uint8_t count);

bool vl53l0x_Init(bool io_2v8);
uint16_t vl53l0x_ReadRangeContinuousMillimeters( statInfo_t *extraStats );
uint16_t vl53l0x_ReadRangeSingleMillimeters( statInfo_t *extraStats );

#endif /* INC_VL53L0X_H_ */
