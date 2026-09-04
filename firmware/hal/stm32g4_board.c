/*
 * firmware/hal/stm32g4_board.c
 * STM32G4 HAL glue (template)
 * NOTE: This is a template. You must configure CubeMX for your exact MCU
 * (e.g., STM32G474RE) and adjust timers/ADC channels/pins accordingly.
 */

#include "buckboost_control.h"
#include "stm32g4xx_hal.h"

// Example handles (declare in your main/board file or adapt names)
extern TIM_HandleTypeDef htim_pwm; // configured for PWM at 150 kHz with complementary outputs
extern ADC_HandleTypeDef hadc1;    // ADC for phase currents / vout / vin
extern DMA_HandleTypeDef hdma_adc1;

// Buffer for ADC readings (example layout)
// Layout and channels must be matched to CubeMX configuration
volatile float adc_phase_currents_buf[N_PHASES];
volatile float adc_vout_buf;
volatile float adc_vin_buf;

// These functions must be adapted to your ADC mapping and scaling
void pwm_set_phase_duty(float duty[N_PHASES]) {
    // Example: update CCR registers for TIM channels. This template assumes
    // each phase maps to a specific CCR register. Implement mapping for your board.
    // e.g., __HAL_TIM_SET_COMPARE(&htim_pwm, TIM_CHANNEL_1, (uint32_t)(duty[0]*htim_pwm.Init.Period));
}

void pwm_config_phase_shift(float phase_deg[N_PHASES]) {
    // Configure capture/compare preload with phase offsets if using multiple timers
}

float adc_read_vout(void) {
    // Return the latest converted Vout (apply scaling ADC->voltage)
    return (float)adc_vout_buf;
}
float adc_read_vin(void) {
    return (float)adc_vin_buf;
}
void adc_read_phase_currents(float i_phase[N_PHASES]) {
    for (int i = 0; i < N_PHASES; ++i) i_phase[i] = adc_phase_currents_buf[i];
}

/*
 * Recommended CubeMX notes:
 * - Configure TIMx for center-aligned or edge-aligned PWM at 150 kHz.
 * - Use complementary outputs for synchronous MOSFET pairs and insert deadtime.
 * - Use TIM TRGO to trigger ADC injected conversions centered in the switching window.
 * - Map ADC injected channels to sample each phase current; if too many channels,
 *   split into two injection sequences and trigger accordingly.
 * - Use DMA to transfer ADC results to memory and perform scaling in software.
 */
