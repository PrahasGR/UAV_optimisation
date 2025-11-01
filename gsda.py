#!/usr/bin/env python3
"""
GSDA vs Distributed GA with analytic gradient (translation of cc.pdf eqs).
- Uses neighbor-only interference (eq. 12).
- Analytic gradient dRj/dx,dRj/dy,dRj/dh derived from pathloss/LoS model.
- Minimal, generic comments.
"""

import math, random, copy
import numpy as np

# ---------------- constants (paper defaults; change to exact Table II if you want)
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

# conversions
def dbm_to_watt(dbm): return 10**(dbm/10.0)/1000.0
def watt_to_dbm(w): return 10*math.log10(w*1000.0) if w>0 else -1e9
PT_w = dbm_to_watt(PT_dBm)
NOISE_w = dbm_to_watt(N_dBm)

# ---------------- radio model
def fspl_dB(d):
    if d <= 1e-9: d = 1e-9
    return 20.0 * math.log10((4.0 * math.pi * d * f_Hz) / C_LIGHT)

def elev_deg(h, rho):
    if rho <= 1e-9: rho = 1e-9
    return (180.0/math.pi) * math.atan2(h, rho)

def p_los(theta_deg):
    return 1.0 / (1.0 + a_param * math.exp(-b_param * (theta_deg - a_param)))

def expected_PL_dB_and_distance(drone, user):
    dx = drone[0]-user[0]; dy = drone[1]-user[1]; h = drone[2]
    rho = math.hypot(dx, dy)
    d = math.sqrt(rho*rho + h*h)
    fspl = fspl_dB(d)
    theta = elev_deg(h, rho)
    p = p_los(theta)
    PL = fspl + eta_NLoS + p * (eta_LoS - eta_NLoS)
    return PL, d, rho, theta, p

def rx_watt_from_drone(drone, user):
    PL,_,_,_,_ = expected_PL_dB_and_distance(drone, user)
    return PT_w * 10**(-PL/10.0)

# ---------------- grid / zones / neighbors ----------------
def grid_centers(rows, cols, zone=zone_side, origin=(0.0,0.0)):
    centers = []
    for r in range(rows):
        for c in range(cols):
            centers.append((origin[0] + (c+0.5)*zone, origin[1] + (r+0.5)*zone))
    return centers

def zone_of_point(pt, rows, cols, zone=zone_side, origin=(0.0,0.0)):
    x_rel = pt[0]-origin[0]; y_rel = pt[1]-origin[1]
    c = int(min(max(math.floor(x_rel/zone),0),cols-1))
    r = int(min(max(math.floor(y_rel/zone),0),rows-1))
    return r*cols + c

def neighbors(idx, rows, cols):
    r = idx//cols; c = idx%cols
    out=[]
    for dr,dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        rr,cc = r+dr, c+dc
        if 0<=rr<rows and 0<=cc<cols:
            out.append(rr*cols+cc)
    return out

# ---------------- assignments and objective ----------------
def user_assignments(users, rows, cols, centers=None):
    if centers is None: centers = grid_centers(rows, cols)
    assigns=[]
    for u in users:
        dists=[((u[0]-cx)**2 + (u[1]-cy)**2,j) for j,(cx,cy) in enumerate(centers)]
        assigns.append(min(dists)[1])
    return assigns

def user_capacity_bits(serving_idx, drone_positions, user, neigh_idxs):
    # per paper eq (11)-(13): SNR = Psig/(N + sum_{neighbors} P_neighbor)
    Psig_w = rx_watt_from_drone(drone_positions[serving_idx], user)
    interf = 0.0
    for j in neigh_idxs:
        if j==serving_idx: continue
        interf += rx_watt_from_drone(drone_positions[j], user)
    denom = NOISE_w + interf
    snr = Psig_w/denom if denom>0 else 0.0
    return math.log2(1.0 + snr)

def total_capacity(drone_positions, users, rows, cols, assigns=None):
    centers = grid_centers(rows, cols)
    if assigns is None: assigns = user_assignments(users, rows, cols, centers)
    s=0.0
    for i,u in enumerate(users):
        j=assigns[i]
        neigh = neighbors(j, rows, cols)
        s += user_capacity_bits(j, drone_positions, u, neigh)
    return s

# ---------------- analytic gradient of Rj wrt x_j,y_j,h_j ----------------
# Derivation summary used in code:
# Rj = sum_{i in Uj} log2(1+SNR_i)
# SNR_i = Psig_i / (N + sum_{n in N_j} Pn_i)
# For derivative wrt UAV j's own position, interference denominator does NOT depend on j (neighbors fixed),
# so dSNR/dx = (dPsig/dx) / denom
# Psig = PT_w * 10^{-PL/10} -> dPsig/dx = Psig * (-ln(10)/10) * dPL/dx
# PL = fspl(d) + eta_NLoS + pLoS*(eta_LoS-eta_NLoS)
# dPL/dx = dFSPL/dd * dd/drho * drho/dx  + dpLoS/dtheta * dtheta/drho * drho/dx * (eta_L - eta_N)
# where rho = ground projection distance, d = sqrt(rho^2 + h^2)

LN10 = math.log(10.0)
def analytic_grad_Fj(j_idx, drone_positions, users, rows, cols, assigns=None):
    centers = grid_centers(rows, cols)
    if assigns is None: assigns = user_assignments(users, rows, cols, centers)
    # gradient of Fj = -Rj
    gx = 0.0; gy = 0.0; gh = 0.0
    pos_j = drone_positions[j_idx]
    for i,u in enumerate(users):
        if assigns[i] != j_idx: continue
        # compute constituents
        PL, d, rho, theta, p = expected_PL_dB_and_distance(pos_j, u)
        # denom and Psig
        neigh = neighbors(j_idx, rows, cols)
        denom = NOISE_w
        for nn in neigh:
            if nn == j_idx: continue
            denom += rx_watt_from_drone(drone_positions[nn], u)
        Psig = PT_w * 10**(-PL/10.0)
        # SNR and derivative piece
        snr = Psig / denom if denom>0 else 0.0
        common_pref = (1.0 / math.log(2.0)) * (1.0 / (1.0 + snr))
        # derivatives: dPsig/dcoord = Psig * (-ln(10)/10) * dPL/dcoord
        # compute dPL/dcoord
        # dFSPL/dd = 20/(ln10) * 1/d
        dFSPL_dd = 20.0 / LN10 * (1.0 / d)  # dB per meter
        # dd/drho = rho/d
        if d == 0: dd_drho = 0.0
        else: dd_drho = rho / d
        dd_dx = 0.0; dd_dy = 0.0
        if rho > 1e-9:
            dd_drho_dx = (pos_j[0] - u[0]) / rho
            dd_drho_dy = (pos_j[1] - u[1]) / rho
        else:
            dd_drho_dx = 0.0; dd_drho_dy = 0.0
        # combine for FSPL part: dFSPL/dx = dFSPL/dd * dd/drho * drho/dx
        dFSPL_dx = dFSPL_dd * dd_drho * dd_drho_dx
        dFSPL_dy = dFSPL_dd * dd_drho * dd_drho_dy
        dFSPL_dh = dFSPL_dd * (h_part := (pos_j[2]/d)) * 0.0  # we'll handle h via dd/dh below
        # Actually dd/dh = h/d
        dd_dh = pos_j[2] / d if d>0 else 0.0
        dFSPL_dh = dFSPL_dd * dd_dh
        # pLoS derivative: dp/dtheta = a b e^{-b(theta-a)} / (1 + a e^{-b(theta-a)})^2
        z = a_param * math.exp(-b_param*(theta - a_param))
        dp_dtheta = b_param * z / ((1.0 + z)**2)
        # dtheta/drho = (180/pi) * d/d(rho) atan(h/rho) = (180/pi) * (-h / (rho^2 + h^2))
        dtheta_drho = (180.0/math.pi) * ( - pos_j[2] / (rho*rho + pos_j[2]*pos_j[2]) ) if (rho*rho + pos_j[2]*pos_j[2])>0 else 0.0
        # dp/drho = dp/dtheta * dtheta/drho
        dp_drho = dp_dtheta * dtheta_drho
        dp_dx = dp_drho * dd_drho_dx
        dp_dy = dp_drho * dd_drho_dy
        # dp/dh uses dtheta/dh = (180/pi) * (rho / (rho^2 + h^2))
        dtheta_dh = (180.0/math.pi) * ( rho / (rho*rho + pos_j[2]*pos_j[2]) ) if (rho*rho + pos_j[2]*pos_j[2])>0 else 0.0
        dp_dh = dp_dtheta * dtheta_dh
        # full dPL/dcoord = dFSPL/dcoord + dp/dcoord * (eta_L - eta_N)
        diff_eta = (eta_LoS - eta_NLoS)
        dPL_dx = dFSPL_dx + dp_dx * diff_eta
        dPL_dy = dFSPL_dy + dp_dy * diff_eta
        dPL_dh = dFSPL_dh + dp_dh * diff_eta
        # dPsig/dcoord
        factor = - (LN10 / 10.0)
        dPsig_dx = Psig * factor * dPL_dx
        dPsig_dy = Psig * factor * dPL_dy
        dPsig_dh = Psig * factor * dPL_dh
        # dSNR/dcoord = dPsig/dcoord / denom (interference independent of j)
        dSNR_dx = dPsig_dx / denom
        dSNR_dy = dPsig_dy / denom
        dSNR_dh = dPsig_dh / denom
        # contribution to gradient of Rj: dRj/dcoord = sum 1/ln2 * 1/(1+SNR) * dSNR/dcoord
        dR_dx = (1.0 / math.log(2.0)) * (1.0 / (1.0 + snr)) * dSNR_dx
        dR_dy = (1.0 / math.log(2.0)) * (1.0 / (1.0 + snr)) * dSNR_dy
        dR_dh = (1.0 / math.log(2.0)) * (1.0 / (1.0 + snr)) * dSNR_dh
        # accumulate for Fj = -Rj
        gx += -dR_dx
        gy += -dR_dy
        gh += -dR_dh
    return gx, gy, gh

# ---------------- projection
def project_to_zone(pos, center):
    half = zone_side/2.0
    x = min(max(pos[0], center[0]-half), center[0]+half)
    y = min(max(pos[1], center[1]-half), center[1]+half)
    h = min(max(pos[2], hmin), hmax)
    return (x,y,h)

# ---------------- GSDA (analytic grad) ----------------
def GSDA_analytic(init_positions, users, rows, cols,
                  K_outer=3, K_sub=(100,100,300),
                  s_vals=(1000.0,1000.0,300.0),
                  xi_matrix=None, verbose=False):
    J = len(init_positions)
    centers = grid_centers(rows, cols)
    drones = [tuple(p) for p in init_positions]
    Lk = [list(p) for p in drones]
    if xi_matrix is None:
        xi = np.zeros((J,J))
        for j in range(J):
            grp = [j] + neighbors(j, rows, cols)
            for i in grp: xi[j,i] = 1.0/len(grp)
    else:
        xi = np.array(xi_matrix, dtype=float)
    assigns = user_assignments(users, rows, cols, centers)
    for k in range(1, K_outer+1):
        if verbose: print("[GSDA] k=",k)
        Lk_prev = copy.deepcopy(Lk)
        for q in range(3):
            Kq = K_sub[q]
            initial = np.array([Lk[j][q] for j in range(J)], dtype=float)
            w_local = [initial.copy() for _ in range(J)]
            # step-size warmup counters
            F_prev = [None]*J
            for tau in range(1, Kq+1):
                new_w = [None]*J
                for j in range(J):
                    group = [j] + neighbors(j, rows, cols)
                    vq = np.zeros(J, dtype=float)
                    for i in group:
                        vq += xi[j,i] * w_local[i]
                    # build tilde vector for gradient eval
                    pos_for_grad = []
                    for jj in range(J):
                        coords = [None,None,None]
                        coords[q] = float(vq[jj])
                        for qq in range(3):
                            if qq==q: continue
                            if qq < q:
                                coords[qq] = Lk[jj][qq]
                            else:
                                coords[qq] = Lk_prev[jj][qq]
                        pos_for_grad.append(tuple(coords))
                    # analytic gradient of Fj at pos_for_grad wrt j's component q
                    gx, gy, gh = analytic_grad_Fj(j, pos_for_grad, users, rows, cols, assigns)
                    if q==0:
                        grad = gx
                        s1 = s_vals[0]; alpha = s1/(tau**2) if tau>1 else s1
                    elif q==1:
                        grad = gy
                        s2 = s_vals[1]; alpha = s2/(tau**2) if tau>1 else s2
                    else:
                        grad = gh
                        s3 = s_vals[2]; alpha = s3 / max(1, tau)
                    vq_prop = vq.copy()
                    vq_prop[j] = vq_prop[j] - alpha * grad
                    # project j component
                    tmp = list(Lk[j]); tmp[q] = float(vq_prop[j])
                    proj = project_to_zone(tuple(tmp), centers[j])
                    vq_prop[j] = proj[q]
                    # small Gibbs-like perturbation for exploration
                    if random.random() < 0.02:
                        vq_prop[j] += random.gauss(0, 1.0)
                        tmp = list(Lk[j]); tmp[q] = float(vq_prop[j])
                        proj = project_to_zone(tuple(tmp), centers[j])
                        vq_prop[j] = proj[q]
                    new_w[j] = vq_prop
                for j in range(J):
                    w_local[j] = np.array(new_w[j], dtype=float)
            # after Kq subiterations update Lk q-row
            avg_row = np.mean(np.vstack(w_local), axis=0)
            for j in range(J):
                Lk[j][q] = float(avg_row[j])
    final = [tuple(p) for p in Lk]
    cap = total_capacity(final, users, rows, cols)
    return final, cap

# ---------------- distributed GA (kept lighter) ----------------
def ga_optimize_single(j, positions, users, rows, cols, center,
                       pop=20, gens=20, mutp=0.2, sigma=12.0):
    half = zone_side/2.0
    popu = []
    for _ in range(pop):
        x = random.uniform(center[0]-half, center[0]+half)
        y = random.uniform(center[1]-half, center[1]+half)
        h = random.uniform(hmin, hmax)
        popu.append((x,y,h))
    assigns = user_assignments(users, rows, cols)
    def fitness(posj):
        pos = [tuple(p) for p in positions]
        pos[j] = posj
        return total_capacity(pos, users, rows, cols, assigns)
    scores = [fitness(ind) for ind in popu]
    for _ in range(gens):
        new=[]
        for _ in range(pop//2):
            a,b = random.sample(range(pop),2)
            p1 = popu[a] if scores[a]>=scores[b] else popu[b]
            c,d = random.sample(range(pop),2)
            p2 = popu[c] if scores[c]>=scores[d] else popu[d]
            child1 = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2)
            child2 = ((p1[0]+p2[0])/2 + random.gauss(0,1),
                      (p1[1]+p2[1])/2 + random.gauss(0,1),
                      (p1[2]+p2[2])/2 + random.gauss(0,1))
            def mutate(ind):
                x,y,h = ind
                if random.random()<mutp: x += random.gauss(0,sigma)
                if random.random()<mutp: y += random.gauss(0,sigma)
                if random.random()<mutp: h += random.gauss(0,sigma/2)
                return project_to_zone((x,y,h), center)
            new.append(mutate(child1)); new.append(mutate(child2))
        popu=new
        scores = [fitness(ind) for ind in popu]
    best = popu[int(np.argmax(scores))]
    return best, max(scores)

def distributed_GA(init_positions, users, rows, cols, KG=4, KpG=1, ga_pop=20, ga_gen=20):
    positions = [tuple(p) for p in init_positions]
    centers = grid_centers(rows, cols)
    for _ in range(KG):
        for j in range(len(positions)):
            best_pos = positions[j]; best_score = -1e12
            for _ in range(KpG):
                pos_found, score = ga_optimize_single(j, positions, users, rows, cols, centers[j],
                                                     pop=ga_pop, gens=ga_gen)
                if score > best_score:
                    best_score = score; best_pos = pos_found
            positions[j] = best_pos
    cap = total_capacity(positions, users, rows, cols)
    return positions, cap

# ---------------- demo ----------------
if __name__ == "__main__":
    random.seed(0); np.random.seed(0)
    rows = cols = 3
    J = rows*cols
    centers = grid_centers(rows, cols)
    total_side = zone_side * rows
    # fixed user layout (deterministic, reproducible). Replace with your list if desired.
    users = []
    # sample deterministic pseudo-random users but fixed seed above
    I = 100
    for _ in range(I):
        users.append((random.uniform(0,total_side), random.uniform(0,total_side)))
    init = [(cx,cy,100.0) for (cx,cy) in centers]

    print("Running GSDA (analytic gradient)...")
    gs_pos, gs_cap = GSDA_analytic(init, users, rows, cols,
                                   K_outer=10, K_sub=(150,150,400),
                                   s_vals=(3000.0,3000.0,900.0),
                                   verbose=True)
    print("GSDA capacity:", gs_cap)

    print("Running distributed GA (lighter budget)...")
    ga_pos, ga_cap = distributed_GA(init, users, rows, cols,
                                    KG=3, KpG=1, ga_pop=10, ga_gen=15)
    print("GA capacity:", ga_cap)

    print("\nComparison: GSDA=", gs_cap, " GA=", ga_cap)
    print("\nGSDA positions:")
    for p in gs_pos: print(p)
    print("\nGA positions:")
    for p in ga_pos: print(p)
