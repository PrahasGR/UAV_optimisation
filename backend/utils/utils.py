
import math
PT_dBm = 30.0
f_Hz = 3.5e9
N_dBm = -100.0
Pmin_dBm = -70.0
hmin, hmax = 30.0, 100.0
zone_side = 600.0
a_param = 9.61
b_param = 0.16
eta_LoS = 1.0
eta_NLoS = 20.0
C_LIGHT = 3e8

def dbm_to_watt(dbm): return 10**(dbm/10.0)/1000.0

def watt_to_dbm(w): return 10*math.log10(w*1000.0) if w>0 else -1e9

def fspl_dB(d):
    if d <= 1e-9: d = 1e-9
    return 20.0 * math.log10((4.0 * math.pi * d * f_Hz) / C_LIGHT)

def elev_deg(h, rho):
    if rho <= 1e-9: rho = 1e-9
    return (180.0/math.pi) * math.atan2(h, rho)

def p_los(theta_deg):
    return 1.0 / (1.0 + a_param * math.exp(-b_param * (theta_deg - a_param)))