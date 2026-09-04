# Python simulation: averaged bidirectional buck-boost multi-phase model

import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
fsw = 150000.0
Ts = 1.0/fsw
sim_time = 0.05
steps = int(sim_time / Ts)

# Hardware example params
V_low = 3.3
V_high = 12.0
Vout_ref = 12.0
Vin = 3.3

L_phase = 2.5e-6
C_out = 470e-6
R_load = 5.0
Nph = 6

# Controller params
Kp_v = 0.15
Ki_v = 80.0
Kp_i = 0.01
Ki_i = 300.0

# DCM/CCM thresholds
DCM_A = 1.0
CCM_B = 2.0
DCM_COUNT = 3
CCM_COUNT = 3

# State
iL = np.zeros(Nph)
vout = V_high * 0.5
int_v = 0.0
int_i = 0.0
is_ccm = True
dcm_ctr = 0
ccm_ctr = 0

def averaged_vl(duty, vin, vout):
    # Simplified averaged inductor voltage for bidirectional buck-boost
    # For demonstration only
    return duty*vin - (1.0-duty)*vout

# Logging
t_log = np.zeros(steps)
v_log = np.zeros(steps)
d_log = np.zeros(steps)
iavg_log = np.zeros(steps)
mode_log = np.zeros(steps)

for k in range(steps):
    t = k*Ts
    # voltage loop at 1 kHz
    if k % int(fsw/1000) == 0:
        err_v = Vout_ref - vout
        int_v += Ki_v * err_v * (1.0/1000.0)
        i_ref = Kp_v * err_v + int_v
        if i_ref < 0: i_ref = 0
    # average current
    i_avg = np.mean(iL)
    # DCM/CCM detection
    if i_avg <= DCM_A:
        dcm_ctr += 1
        ccm_ctr = 0
        if dcm_ctr >= DCM_COUNT:
            is_ccm = False
    elif i_avg > CCM_B:
        ccm_ctr += 1
        dcm_ctr = 0
        if ccm_ctr >= CCM_COUNT:
            is_ccm = True
    # inner current PI
    err_i = i_ref - i_avg
    int_i += Ki_i * err_i * Ts
    d = Kp_i * err_i + int_i
    d = max(0.0, min(0.95, d))
    # apply per-phase update
    vL = averaged_vl(d, Vin, vout)
    di = (vL / L_phase) * Ts
    iL += di
    # output capacitor
    i_out = np.sum(iL)
    dv = (i_out - vout/R_load) * (Ts / C_out)
    vout += dv
    # logging
    t_log[k] = t
    v_log[k] = vout
    d_log[k] = d
    iavg_log[k] = i_avg
    mode_log[k] = 1 if is_ccm else 0

# plots
plt.figure()
plt.subplot(3,1,1)
plt.plot(t_log, v_log)
plt.ylabel('Vout (V)')
plt.grid()
plt.subplot(3,1,2)
plt.plot(t_log, d_log)
plt.ylabel('Duty')
plt.grid()
plt.subplot(3,1,3)
plt.plot(t_log, iavg_log)
plt.plot(t_log, mode_log*3.0, '--')
plt.ylabel('Iavg (A) / CCM flag')
plt.xlabel('Time (s)')
plt.grid()
plt.show()
