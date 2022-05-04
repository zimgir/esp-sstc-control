#include <Arduino.h>

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/rtc_io_struct.h>

#include "config.h"
#include "utils.h"
#include "adc.h"

/**
 * @brief Copied from esp-idf source: components\hal\platform_port\include\hal\misc.h
 *
 * Macro to force a 32-bit read, modify, then write on a peripheral register
 *
 * Due to a GCC bug, the compiler may still try to optimize read/writes to peripheral register fields by using 8/16 bit
 * access, even if they are marked volatile (i.e., -fstrict-volatile-bitfields has no effect).
 *
 * For ESP chips, the peripheral bus only allows 32-bit read/writes. The following macro works around the compiler issue
 * by forcing a 32-bit read/modify/write.
 *
 * @note This macro should only be called on register fields of xxx_struct.h type headers, as it depends on the presence
 *       of a 'val' field of the register union.
 * @note Current implementation reads into a uint32_t instead of copy base_reg direclty to temp_reg. The reason being
 *       that C++ does not create a copy constructor for volatile structs.
 */
#define HAL_FORCE_MODIFY_U32_REG_FIELD(base_reg, reg_field, field_val) \
  {                                                                    \
    uint32_t temp_val = base_reg.val;                                  \
    typeof(base_reg) temp_reg;                                         \
    temp_reg.val = temp_val;                                           \
    temp_reg.reg_field = (field_val);                                  \
    (base_reg).val = temp_reg.val;                                     \
  }

/**
 * @brief Copied from esp-idf source: components\hal\platform_port\include\hal\misc.h
 *
 * Macro to force a 32-bit read on a peripheral register
 *
 * @note This macro should only be called on register fields of xxx_struct.h type headers. See description above for
 *       more details.
 * @note Current implementation reads into a uint32_t. See description above for more details.
 */
#define HAL_FORCE_READ_U32_REG_FIELD(base_reg, reg_field) ({ \
  uint32_t temp_val = base_reg.val;                          \
  typeof(base_reg) temp_reg;                                 \
  temp_reg.val = temp_val;                                   \
  temp_reg.reg_field;                                        \
})


void adc1_init()
{
  ESP_ERROR_CHECK(adc1_config_width(ADC_RESOLUTION));
}

void adc1_init_channel(adc1_channel_t channel, adc_atten_t atten)
{
  ESP_ERROR_CHECK(adc1_config_channel_atten(channel, atten));

  // call abstracted ADC API once to get sample for initital configuration
  adc1_get_raw(channel);

  // PROFILE_OP_TIMES(adc1_get_raw(SAMPLER_CHANNEL), 10);
  // PROFILE_OP_TIMES(adc1_fast_sample(SAMPLER_CHANNEL), 10);
}

/* Bypass OS managment and modify HW registers directly
IRAM_ATTR is applied so that function could be called from ISR if necessary */
uint16_t IRAM_ATTR adc1_fast_sample(adc1_channel_t channel)
{
  uint16_t raw_sample;

  // power ON ADC
  SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;

  // set forced SW control for ADC
  SENS.sar_read_ctrl.sar1_dig_force = 0;      // 1: Select digital control;       0: Select RTC control.
  SENS.sar_meas_start1.meas1_start_force = 1; // 1: SW control RTC ADC start;     0: ULP control RTC ADC start.
  SENS.sar_meas_start1.sar1_en_pad_force = 1; // 1: SW control RTC ADC bit map;   0: ULP control RTC ADC bit map;
  SENS.sar_touch_ctrl1.xpd_hall_force = 1;    // 1: SW control HALL power;        0: ULP FSM control HALL power.
  SENS.sar_touch_ctrl1.hall_phase_force = 1;  // 1: SW control HALL phase;        0: ULP FSM control HALL phase.

  // disable HALL sensor
  RTCIO.hall_sens.xpd_hall = 0;
  // channel is set in the  convert function
  SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD;
  // disable FSM, it's only used by the LNA.
  SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
  SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
  SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
  HAL_FORCE_MODIFY_U32_REG_FIELD(SENS.sar_meas_wait1, sar_amp_wait1, 1);
  HAL_FORCE_MODIFY_U32_REG_FIELD(SENS.sar_meas_wait1, sar_amp_wait2, 1);
  HAL_FORCE_MODIFY_U32_REG_FIELD(SENS.sar_meas_wait2, sar_amp_wait3, 1);

  // enable channel
  SENS.sar_meas_start1.sar1_en_pad = (1 << channel);

  // wait clear
  while (HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_slave_addr1, meas_status) != 0)
    ;

  // start signal
  SENS.sar_meas_start1.meas1_start_sar = 0;
  SENS.sar_meas_start1.meas1_start_sar = 1;

  // wait done
  while (HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas_start1, meas1_done_sar) == 0)
    ;

  // read result
  raw_sample = HAL_FORCE_READ_U32_REG_FIELD(SENS.sar_meas_start1, meas1_data_sar);

  return raw_sample;
}