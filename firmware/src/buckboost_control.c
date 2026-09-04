/*
 * firmware/src/buckboost_control.c
 * 6-phase bidirectional synchronous buck-boost control for STM32G4
 * Features:
 * - MODE: BUCK / BOOST / BIDIR
 * - CV/CC automatic switching (or forced CV/CC)
 * - DCM / CCM detection with hysteresis
 * - Per-phase and total current limits and protections
 * - Phase balancing
 *
 * Hardware-specific functions (pwm_set_phase_duty, adc_read_*) must be
 * provided by board-level code (STM32 HAL implementation files).
 */

#include "buckboost_control.h"
#include <string.h>
#include <math.h>

#define PWM_FREQ_HZ        150000.0f
#define TS_PWM             (1.0f / PWM_FREQ_HZ) // ~6.667 us
#define VOLTAGE_LOOP_HZ    1000.0f
#define TS_VOLTAGE         (1.0f / VOLTAGE_LOOP_HZ) // 1 ms

#define DUTY_MIN 0.0f
#define DUTY_MAX 0.95f

// DCM/CCM thresholds and hysteresis counts
#define DCM_THRES_A 1.0f
#define CCM_THRES_B 2.0f
#define DCM_CONFIRM_COUNT 3
#define CCM_CONFIRM_COUNT 3

// Protection defaults (can be overridden)
static float vout_min = 10.0f;
static float vout_max = 13.2f;
static float vin_min = 2.5f;
static float vin_max = 3.6f;
static float per_phase_i_limit = 60.0f;
static float total_low_i_limit = 360.0f;
static float total_high_i_limit = 100.0f;

// Controller structs
typedef struct { float kp; float ki; float integrator; float out_min; float out_max; } PI_Controller;
static PI_Controller vol_pi = { .kp = 0.2f, .ki = 80.0f, .integrator = 0.0f, .out_min = -100.0f, .out_max = 100.0f };
static PI_Controller cur_pi = { .kp = 0.01f, .ki = 300.0f, .integrator = 0.0f, .out_min = DUTY_MIN, .out_max = DUTY_MAX };

// State
static bb_mode_t current_mode = MODE_IDLE;
static bb_loop_mode_t loop_mode = LOOP_MODE_AUTO;
static float vout_ref = 12.0f;
static float iref_user = 0.0f; // if user forces CC
static float i_ref_global = 0.0f;
static float phase_duty[N_PHASES];
static float phase_currents[N_PHASES];
static uint32_t fault_flags = 0;

// DCM/CCM detection
static int dcm_counter = 0;
static int ccm_counter = 0;
static int is_ccm = 1; // start in CCM by default

// Utility
static float pi_update(PI_Controller *c, float err, float dt) {
    float up = c->kp * err;
    c->integrator += c->ki * err * dt;
    float u = up + c->integrator;
    if (u > c->out_max) { c->integrator -= c->ki * err * dt; u = c->out_max; }
    else if (u < c->out_min) { c->integrator -= c->ki * err * dt; u = c->out_min; }
    return u;
}

void buckboost_init(void) {
    memset(phase_duty, 0, sizeof(phase_duty));
    memset(phase_currents, 0, sizeof(phase_currents));
    current_mode = MODE_IDLE;
    loop_mode = LOOP_MODE_AUTO;
    fault_flags = 0;
}

void buckboost_set_mode(bb_mode_t mode) { current_mode = mode; }
void buckboost_set_vout_ref(float vout) { vout_ref = vout; }
void buckboost_set_iref(float iref) { iref_user = iref; }
void buckboost_set_loop_mode(bb_loop_mode_t m) { loop_mode = m; }

void buckboost_set_limits(float voutmin, float voutmax, float vinmin, float vinmax,
                          float per_phase_i_max, float total_low_i_max, float total_high_i_max) {
    vout_min = voutmin; vout_max = voutmax; vin_min = vinmin; vin_max = vinmax;
    per_phase_i_limit = per_phase_i_max; total_low_i_limit = total_low_i_max; total_high_i_limit = total_high_i_max;
}

uint32_t buckboost_get_fault_flags(void) { return fault_flags; }

// Inner loop: call at PWM frequency (e.g., from TIM interrupt)
void buckboost_inner_loop_tick(void) {
    // read per-phase currents
    adc_read_phase_currents(phase_currents);

    // compute totals and check per-phase limits
    float total_low_I = 0.0f; // low-side total
    float total_high_I = 0.0f; // for boost direction if measured separately; here approximate
    float max_phase_i = 0.0f;
    for (int i = 0; i < N_PHASES; ++i) {
        total_low_I += fabsf(phase_currents[i]);
        if (fabsf(phase_currents[i]) > max_phase_i) max_phase_i = fabsf(phase_currents[i]);
    }
    // DCM/CCM detection based on average phase current
    float avg_i = total_low_I / N_PHASES;
    if (avg_i <= DCM_THRES_A) { dcm_counter++; ccm_counter = 0; if (dcm_counter >= DCM_CONFIRM_COUNT) is_ccm = 0; }
    else if (avg_i > CCM_THRES_B) { ccm_counter++; dcm_counter = 0; if (ccm_counter >= CCM_CONFIRM_COUNT) is_ccm = 1; }

    // Protection checks
    if (max_phase_i > per_phase_i_limit) {
        fault_flags |= (1<<0); // per-phase overcurrent
        // apply soft limit by reducing i_ref_global
        i_ref_global *= 0.9f;
    }
    if (total_low_I > total_low_i_limit) {
        fault_flags |= (1<<1); // low-side total overcurrent
        i_ref_global *= 0.85f;
    }
    // Note: total_high_I detection requires high-side sensing; approximate with total_low_I here
    if (total_low_I > total_high_i_limit) {
        fault_flags |= (1<<2);
        i_ref_global *= 0.8f;
    }

    // Current control (inner)
    float err_i = i_ref_global - avg_i;
    float duty = pi_update(&cur_pi, err_i, TS_PWM);

    // Direction and mapping: in bidir mode, sign of i_ref_global can indicate direction
    // For simplicity we use same duty mapping; hardware switching pattern must realize buck/boost topology

    // distribute duties equally and apply phase balance
    for (int p = 0; p < N_PHASES; ++p) phase_duty[p] = duty;
    // small balancing based on per-phase current differences
    float avg = avg_i;
    const float k_adj = 0.0005f;
    for (int p = 0; p < N_PHASES; ++p) phase_duty[p] += k_adj * (avg - phase_currents[p]);

    // clamp
    for (int p = 0; p < N_PHASES; ++p) {
        if (phase_duty[p] < DUTY_MIN) phase_duty[p] = DUTY_MIN;
        if (phase_duty[p] > DUTY_MAX) phase_duty[p] = DUTY_MAX;
    }

    pwm_set_phase_duty(phase_duty);
}

// Voltage loop: lower rate (e.g., 1 kHz)
void buckboost_voltage_loop_tick(void) {
    float vout = adc_read_vout();
    float vin = adc_read_vin();

    // protections for voltages
    if (vout < vout_min || vout > vout_max) fault_flags |= (1<<3);
    if (vin < vin_min || vin > vin_max) fault_flags |= (1<<4);

    // Determine i_ref target based on loop mode
    if (loop_mode == LOOP_MODE_CC) {
        i_ref_global = iref_user;
    } else if (loop_mode == LOOP_MODE_CV) {
        float err_v = vout_ref - vout;
        float i_out = pi_update(&vol_pi, err_v, TS_VOLTAGE);
        if (i_out < 0) i_out = 0;
        i_ref_global = i_out; // outer loop produces current reference
    } else { // AUTO CV/CC
        // compute voltage-driven iref and compare to user iref (if provided)
        float err_v = vout_ref - vout;
        float i_from_v = pi_update(&vol_pi, err_v, TS_VOLTAGE);
        if (i_from_v < 0) i_from_v = 0;
        // if user set a manual iref > 0 we treat it as a limit
        if (iref_user > 0.0f) {
            // CV prioritized, but limit i_from_v by iref_user in CC situations
            if (i_from_v > iref_user) {
                // CC active
                i_ref_global = iref_user;
            } else {
                // CV active
                i_ref_global = i_from_v;
            }
        } else {
            i_ref_global = i_from_v;
        }
    }

    // Additional safety clamp
    if (i_ref_global < 0.0f) i_ref_global = 0.0f;

}
