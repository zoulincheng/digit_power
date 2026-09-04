#ifndef BUCKBOOST_CONTROL_H
#define BUCKBOOST_CONTROL_H

#include <stdint.h>

#define N_PHASES 6

typedef enum {
    MODE_IDLE = 0,
    MODE_BUCK,
    MODE_BOOST,
    MODE_BIDIR  // allow automatic direction based on setpoints
} bb_mode_t;

typedef enum {
    LOOP_MODE_AUTO = 0, // auto CV/CC switching
    LOOP_MODE_CV,
    LOOP_MODE_CC
} bb_loop_mode_t;

// Public API
void buckboost_init(void);

// Must be called periodically as follows:
// - buckboost_inner_loop_tick() at PWM frequency (150 kHz)
// - buckboost_voltage_loop_tick() at voltage loop frequency (~1 kHz)
void buckboost_inner_loop_tick(void);
void buckboost_voltage_loop_tick(void);

// Mode and setpoints
void buckboost_set_mode(bb_mode_t mode);
void buckboost_set_vout_ref(float vout);
void buckboost_set_iref(float iref);
void buckboost_set_loop_mode(bb_loop_mode_t mode);

// Limits and protection
void buckboost_set_limits(float vout_min, float vout_max,
                          float vin_min, float vin_max,
                          float per_phase_i_max, float total_low_i_max, float total_high_i_max);

// Hardware abstraction layer callbacks (implement in board code)
void pwm_set_phase_duty(float duty[N_PHASES]); // duty 0..1
void pwm_config_phase_shift(float phase_deg[N_PHASES]);

float adc_read_vout(void);
float adc_read_vin(void);
void adc_read_phase_currents(float i_phase[N_PHASES]);

// Fault & status
uint32_t buckboost_get_fault_flags(void);

#endif // BUCKBOOST_CONTROL_H
