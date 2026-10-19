#!/usr/bin/env python3
"""Aviation Dynamics: turbofan aircraft definitions from published figures.

Each type below is a short specification of what its manufacturer and the
usual references publish (dimensions, wing area and sweep, weights, fuel,
engines, speed limits, flap settings, which systems it has). The generator
builds the full definition by laying that type out on the Top G's validated
arrangement (data/aircraft/top_g.json): the same wing planform family,
sections, flap and spoiler layout, tail volume coefficients, gear geometry
in % MAC and systems blocks, scaled to the type's own size, weights and
engines. Calibration (tools/jetgen/calibrate.mjs) then sets the few values
the published data pins down (the engine lapse to the maximum cruise speed,
the flap lift to the approach speed where one is published) and writes the
result.

    python3 tools/jetgen/jetgen.py            # all types
    python3 tools/jetgen/jetgen.py crj700     # one

Units in the specs: SI unless the key says otherwise (lb, kt, ft, lbf).
"""
import copy
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DATA = os.path.join(REPO, 'data', 'aircraft')
LB = 0.45359237
KT = 0.514444
FT = 0.3048

# The Top G (template) reference figures.
TOP = dict(L=30.41, R=1.32, half_span=14.76, wing_area=117.01, mtow=46992.0, empty=24000.0,
           thrust_kn=75.2, mac=4.4605, xle_mac=-14.8749, empty_cg_pct=27.466,
           wing_qc_root=-12.625, fwd_cg_pct=(7.289, 10.652), aft_cg_pct=36.434, mains_pct=45.40,
           vh=0.6879, vv=0.05771, ht_area=25.97, vt_area=18.92, fin_h=4.35)

# Top G fuselage stations [x, half-width, z top, z bottom] (FLU, nose tip x = 0):
# the nose to the end of the parallel section, then the tail cone.
NOSE = [[0.0, 0.04, 0.02, -0.12], [-0.35, 0.42, 0.22, -0.5], [-1.0, 0.82, 0.55, -0.88], [-1.9, 1.08, 0.88, -1.12],
        [-3.0, 1.26, 1.2, -1.26], [-4.4, 1.32, 1.32, -1.3]]
TAIL = [[-19.5, 1.32, 1.32, -1.3], [-21.8, 1.22, 1.3, -1.02], [-24.2, 1.0, 1.24, -0.5], [-26.6, 0.72, 1.14, 0.06],
        [-28.9, 0.4, 0.98, 0.45], [-30.41, 0.1, 0.76, 0.62]]

SPECS = {
    'phenom300': dict(
        display='Phenom 300E', manufacturer='Embraer', short='PHENOM 300',
        reference='Embraer EMB-505 Phenom 300E (2 x Pratt & Whitney Canada PW535E1), light business jet',
        L=15.64, R=0.86, span=16.2, wing_area=28.2, sweep=22.0, winglets=True, wing_x=0.47, t_tail=True,
        mtow_lb=18551, mlw_lb=17042, empty_lb=11583 - 375, mzfw_lb=14330, fuel_lb=5353, fuel_density=0.803,
        thrust_lbf=3478, bpr=2.6, tsfc=0.44, engines='aft', reversers=False,
        speedbrake=dict(detents=2), vmo=320, mmo=0.80, vle=250, flaps=[0, 8, 26, 37], vfe=[180, 170, 150],
        cruise=dict(ktas=464, fl=300), ceiling_ft=45000, vref=None,
        colors=dict(primary='#0f3a5c', accent='#c9a227'),
        windows=dict(count=6, first=-4.6, pitch=0.82, half_w=0.2, half_h=0.15),
        sources=[
            ('AVWEEK', 'Aviation Week, Aircraft Overview: Phenom 300E (PW535E1 3,478 lbf, MTOW 18,551 lb, take-off 3,209 ft at MTOW SL ISA).'),
            ('CDD', 'Conklin & de Decker, Embraer Phenom 300E (usable fuel 5,353 lb).'),
            ('SPEC', 'Dealer and reference listings (MLW 17,042 lb for the 300E, BOW 11,583 lb, span 53 ft 2 in, VFE 8 deg 180 / 26 deg 170 KIAS).'),
            ('AIN', 'AIN, Embraer addresses Phenom 300 glitches (2010): no thrust reversers; flap 3 26 deg, full flap 37 deg.'),
            ('TT', 'Twin & Turbine (Oct 2011): speedbrake fully open or closed; ground spoilers deploy automatically on touchdown and on a rejected take-off.'),
        ]),
    'cj4': dict(
        display='Citation CJ4', manufacturer='Cessna (Textron Aviation)', short='CJ4',
        reference='Cessna Citation CJ4 525C (2 x Williams International FJ44-4A), light business jet',
        L=16.26, R=0.82, span=15.49, wing_area=30.66, sweep=12.5, winglets=False, wing_x=0.47, t_tail=False,
        mtow_lb=17110, mlw_lb=15660, empty_lb=10280 - 375, mzfw_lb=12500, fuel_lb=5828, fuel_density=0.803,
        thrust_lbf=3621, bpr=3.3, tsfc=0.46, engines='aft', reversers=False,
        speedbrake=dict(detents=3), vmo=305, mmo=0.77, vle=250, flaps=[0, 15, 35], vfe=[200, 161],
        cruise=dict(ktas=451, fl=300), ceiling_ft=45000, vref=None,
        colors=dict(primary='#1d2a33', accent='#b6202a'),
        windows=dict(count=6, first=-4.7, pitch=0.85, half_w=0.18, half_h=0.15),
        sources=[
            ('TXT', 'Textron Aviation, Citation CJ4 product pages (wing area 330 sq ft).'),
            ('JETCRAFT', 'JetCraft, Cessna Citation CJ4 overview (FJ44-4A 3,621 lbf, MTOW 17,110 lb, MLW 15,660 lb, BOW 10,280 lb).'),
            ('GLOBALAIR', 'GlobalAir, Citation CJ4 Gen 2 specifications (fuel 5,828 lb).'),
            ('TT', 'Twin & Turbine (June 2015): speedbrakes on a lever by the throttles, ground spoilers after touchdown.'),
        ]),
    'learjet45': dict(
        display='Learjet 45', manufacturer='Learjet (Bombardier)', short='LEARJET 45',
        reference='Learjet 45 (2 x Honeywell TFE731-20AR), mid-size business jet',
        L=17.68, R=0.88, span=14.58, wing_area=28.95, sweep=13.0, winglets=True, wing_x=0.50, t_tail=True,
        mtow_lb=20500, mlw_lb=19200, empty_lb=13500, mzfw_lb=16000, fuel_lb=6062, fuel_density=0.803,
        thrust_lbf=3500, bpr=3.1, tsfc=0.41, engines='aft', reversers=True,
        speedbrake=dict(detents=3), vmo=330, mmo=0.81, vle=260, flaps=[0, 8, 20, 40], vfe=[250, 200, 150],
        cruise=dict(ktas=465, fl=330), ceiling_ft=51000, vref=None,
        colors=dict(primary='#2b2f36', accent='#c8102e'),
        windows=dict(count=7, first=-4.9, pitch=0.84, half_w=0.2, half_h=0.16),
        sources=[
            ('AEROPEDIA', 'Aeropedia, Learjet 45 (wing area 28.95 m2, 3,500 lbst TFE731-20).'),
            ('FLYRADIUS', 'FlyRadius, Bombardier Learjet 45 specifications (MTOW 20,500 lb, MLW 19,200 lb, fuel 6,062 lb).'),
        ]),
    'crj700': dict(
        display='CRJ700', manufacturer='Bombardier (now Mitsubishi)', short='CRJ700',
        reference='Bombardier CRJ700 (CL-600-2C10, 2 x GE CF34-8C5), 70-seat regional jet',
        L=32.51, R=1.35, span=23.24, wing_area=70.6, sweep=26.9, winglets=True, wing_x=0.46, t_tail=True,
        mtow_lb=75000, mlw_lb=67000, empty_lb=44245, mzfw_lb=62300, fuel_lb=19450, fuel_density=0.803,
        thrust_lbf=13790, bpr=5.0, tsfc=0.36, engines='aft', reversers=True,
        speedbrake=dict(detents=3), vmo=335, mmo=0.85, vle=220, flaps=[0, 1, 8, 20, 30, 45], vfe=[230, 230, 220, 185, 170],
        cruise=dict(ktas=473, fl=370), ceiling_ft=41000, vref=dict(kias=135, weight_lb=67000, flap=45),
        colors=dict(primary='#1e3f73', accent='#d7262e'),
        windows=dict(count=19, first=-5.6, pitch=0.82, half_w=0.15, half_h=0.2),
        sources=[
            ('FLYRADIUS', 'FlyRadius, Bombardier CRJ700 specifications (MTOW 75,000 lb ER, MLW 67,000 lb, fuel 19,450 lb).'),
            ('GLOBALAIR', 'GlobalAir, CRJ-700 specifications (operating weight 44,245 lb).'),
            ('WIKI', 'Wikipedia, Bombardier CRJ (CF34-8C5 13,790 lbf flat-rated, wing area 760 sq ft).'),
            ('FORUM', 'Airline-pilot forum quoting the CRJ-701ER handbook: Vref flaps 45 135 KIAS at 67,000 lb (secondary).'),
        ]),
    'b738': dict(
        display='737-800', manufacturer='Boeing', short='737-800',
        reference='Boeing 737-800 (2 x CFM International CFM56-7B26), narrow-body airliner',
        L=39.47, R=1.88, span=35.79, wing_area=124.6, sweep=25.0, winglets=True, wing_x=0.415, t_tail=False,
        mtow_lb=174200, mlw_lb=146300, empty_lb=91300, mzfw_lb=138300, fuel_l=26020, fuel_density=0.803,
        thrust_lbf=26300, bpr=5.1, tsfc=0.36, engines='wing', reversers=True,
        speedbrake=dict(detents=3), vmo=340, mmo=0.82, vle=270, flaps=[0, 1, 5, 15, 25, 30, 40], vfe=[250, 250, 200, 190, 175, 162],
        cruise=dict(ktas=460, fl=350), ceiling_ft=41000, vref=None,
        colors=dict(primary='#123a6b', accent='#e0a526'),
        windows=dict(count=26, first=-6.2, pitch=0.51, half_w=0.12, half_h=0.17),
        sources=[
            ('QANTAS', 'Qantas, Boeing 737-800 fact sheet (MTOW 79,015 kg, wing area 124.6 m2, fuel ~26,025 L, CFM56-7B26 116.99 kN).'),
            ('BOEING', 'Boeing 737 airplane characteristics for airport planning (dimensions, MLW 146,300 lb, OEW ~91,300 lb) as commonly reproduced.'),
        ]),
}


def mac_of(panels):
    """Area (both sides), MAC and the MAC's leading-edge x (structural) of a wing's non-winglet panels."""
    area = msum = xsum = 0.0
    for p in panels:
        if p.get('dihedral_deg', 0) > 45:
            continue
        qx, qy, _ = p['root_quarter_chord_m']
        b, c0, c1 = p['span_m'], p['root_chord_m'], p['tip_chord_m']
        sw = math.radians(p.get('sweep_deg', 0))
        n = 200
        for i in range(n):
            t = (i + 0.5) / n
            c = c0 + (c1 - c0) * t
            xle = (qx - b * t * math.tan(sw)) + 0.25 * c  # the quarter-chord line plus a quarter chord
            dy = b / n
            area += c * dy
            msum += c * c * dy
            xsum += xle * c * dy
    return 2 * area, msum / area, xsum / area


def r3(v):
    return [round(x, 4) for x in v]


def build(key, spec, top):
    d = copy.deepcopy(top)
    s = spec
    mtow, mlw, empty = s['mtow_lb'] * LB, s['mlw_lb'] * LB, s['empty_lb'] * LB
    mzfw = s['mzfw_lb'] * LB
    fuel_kg = s['fuel_lb'] * LB if 'fuel_lb' in s else s['fuel_l'] * s['fuel_density']
    fuel_l = fuel_kg / s['fuel_density']
    sL, sR = s['L'] / TOP['L'], s['R'] / TOP['R']
    sm = mtow / TOP['mtow']
    thrust_kn = s['thrust_lbf'] * 4.4482216 / 1000
    rn = 0.79 * math.sqrt(thrust_kn / TOP['thrust_kn'])  # nacelle radius ~ sqrt(thrust)

    # Fuselage: the nose and tail cone scale with the diameter, the parallel section takes the rest.
    tail_len = 10.91 * sR
    nose = [[x * sR, w * sR, zt * sR, zb * sR] for x, w, zt, zb in NOSE]
    tail0 = -(s['L'] - tail_len)
    tail = [[tail0 + (x - TAIL[0][0]) * sR, w * sR, zt * sR, zb * sR] for x, w, zt, zb in TAIL]
    tail[-1][0] = -s['L']
    stations = [r3(p) for p in nose + tail]
    xs = lambda x: x * sR if x > -4.4 else (tail0 + (x + 19.5) * sR if x < -19.5 else None)

    # ---- Wing: the Top G planform family at the type's span, area and sweep ----
    half = s['span'] / 2
    sb = half / (15.18 if s['winglets'] else 14.76)
    sc = (s['wing_area'] / 117.01) / sb
    wing = d['surfaces'][0]
    pin, pout, pwl = wing['panels']
    lam_in, lam_out = max(s['sweep'] - 6, 0.0), s['sweep']
    root_x = -s['wing_x'] * s['L']
    zw = 0.95 * sR
    pin.update(root_quarter_chord_m=r3([root_x, 0, zw]), span_m=round(4.2 * sb, 3), root_chord_m=round(6.9 * sc, 3),
               tip_chord_m=round(4.7 * sc, 3), sweep_deg=lam_in)
    qx1 = root_x - pin['span_m'] * math.tan(math.radians(lam_in))
    zk = zw - pin['span_m'] * math.tan(math.radians(3.0))
    pout.update(root_quarter_chord_m=r3([qx1, pin['span_m'], zk]), span_m=round(10.56 * sb, 3), root_chord_m=pin['tip_chord_m'],
                tip_chord_m=round(1.767 * sc, 3), sweep_deg=lam_out)
    qx2 = qx1 - pout['span_m'] * math.tan(math.radians(lam_out))
    zt = zk - pout['span_m'] * math.tan(math.radians(3.0))
    if s['winglets']:
        pwl.update(root_quarter_chord_m=r3([qx2, pin['span_m'] + pout['span_m'], zt]), span_m=round(0.42 * sb, 3),
                   root_chord_m=pout['tip_chord_m'], tip_chord_m=round(0.6 * sc, 3))
        wing['panels'] = [pin, pout, pwl]
    else:
        wing['panels'] = [pin, pout]
    flaps = s['flaps']
    fmax = flaps[-1]
    for p in wing['panels']:
        for c in p.get('controls', []):
            if c['channel'] == 'flap':
                c['max_positive_deg'] = fmax
                c['effectiveness'] = {'deflection_deg': [0, round(fmax * 0.26, 1), round(fmax * 0.51, 1), fmax], 'eta': [1.0, 0.96, 0.88, 0.72]}
                c['clmax_increment_ratio'] = s.get('flap_clmax_ratio', 0.9)
    S, mac, xle = mac_of(wing['panels'])
    d['reference'] = {'area_m2': round(s['wing_area'], 2), 'span_m': round(s['span'], 2), 'chord_m': round(mac, 3)}

    # ---- Tail: the Top G's volume coefficients at the type's arms ----
    ht, vt = d['surfaces'][1], d['surfaces'][2]
    htp, vtp = ht['panels'][0], vt['panels'][0]
    x_ht = tail0 + (-28.3501 + 19.5) * sR
    x_vt = tail0 + (-25.0 + 19.5) * sR
    l_h = (xle - 0.25 * mac) - (x_ht - 0.3)  # approx: tail MAC quarter chord ~0.3 m aft of its root's
    ht_area = TOP['vh'] * S * mac / l_h
    k_h = math.sqrt(ht_area / TOP['ht_area'])
    l_v = (xle - 0.25 * mac) - (x_vt - 1.6 * k_h)
    # The fin balances the fuselage too: the Top G's volume coefficient plus
    # what the extra fuselage destabilising moment needs (Munk's moment,
    # 2 (k2 - k1) Vol / (S b) per radian, against a fin lift slope of ~3/rad:
    # 0.6 x the extra Vol / (S b)). A long fuselage on a small wing (CRJ700,
    # 737-800) needs a larger fin: 0.083 and 0.077 here against ~0.08
    # published; with the Top G's 0.058 the CRJ's Dutch roll diverged slowly
    # below 140 KTAS.
    fus_ratio = 105.0 * sL * sR * sR / (S * s['span']) - 105.0 / (TOP['wing_area'] * 2 * TOP['half_span'])
    vv = TOP['vv'] + 0.6 * max(fus_ratio, 0.0)
    vt_area = vv * S * s['span'] / max(l_v, 1.0)
    k_v = math.sqrt(vt_area / TOP['vt_area'])
    vtp.update(root_quarter_chord_m=r3([x_vt, 0, -1.15 * sR]), span_m=round(4.35 * k_v, 3), root_chord_m=round(5.4 * k_v, 3),
               tip_chord_m=round(3.3 * k_v, 3))
    fin_top_z = -1.15 * sR - vtp['span_m']
    if s['t_tail']:
        x_ht = x_vt - vtp['span_m'] * math.tan(math.radians(vtp['sweep_deg'])) + 0.25 * (vtp['root_chord_m'] - vtp['tip_chord_m']) - 0.2 * k_h
        z_ht = fin_top_z + 0.05
        ht['panels'][0]['dihedral_deg'] = -2.0
    else:
        x_ht = tail0 + (-27.2 + 19.5) * sR
        z_ht = -0.25 * sR
        ht['panels'][0]['dihedral_deg'] = 6.0 if key == 'b738' else 2.0
    htp.update(root_quarter_chord_m=r3([x_ht, 0, z_ht]), span_m=round(5.3 * k_h, 3), root_chord_m=round(3.3 * k_h, 3),
               tip_chord_m=round(1.6 * k_h, 3))

    # ---- Mass, CG, stations, inertia ----
    m = d['mass']
    m['empty_kg'] = round(empty, 1)
    cg_x = xle - TOP['empty_cg_pct'] / 100 * mac
    m['empty_cg_m'] = r3([cg_x, 0.0, 0.15 * sR])
    ixx = 498000 * (empty / TOP['empty']) * (s['span'] / 30.36) ** 2
    iyy = 604000 * (empty / TOP['empty']) * (s['L'] / TOP['L']) ** 2
    izz = 1080000 * (empty / TOP['empty']) * (s['L'] ** 2 + s['span'] ** 2) / (TOP['L'] ** 2 + 30.36 ** 2)
    m['empty_inertia_kgm2'] = {'ixx': round(ixx), 'iyy': round(iyy), 'izz': round(izz), 'ixz': 0}
    m['max_ramp_kg'] = round(mtow + 0.004 * mtow, 1)
    m['max_takeoff_kg'] = round(mtow, 1)
    m['max_landing_kg'] = round(mlw, 1)
    m['max_zero_fuel_kg'] = round(mzfw, 1)
    payload = max(mzfw - empty - 170, 300)
    crew = 170 if key != 'b738' and key != 'crj700' else 190
    m['stations'] = [
        {'name': 'flight deck crew', 'position_m': r3([-3.6 * sR, 0.0, 0.2 * sR]), 'max_kg': 250, 'initial_kg': crew},
        {'name': 'cabin', 'position_m': r3([cg_x + 0.8 * mac * 0.3, 0.0, 0.2 * sR]), 'max_kg': round(0.85 * payload), 'initial_kg': round(0.35 * payload)},
        {'name': 'baggage', 'position_m': r3([tail0 + 0.5 * sR, 0.0, 0.3 * sR]), 'max_kg': round(0.15 * payload), 'initial_kg': round(0.05 * payload)},
    ]
    pct = lambda p: round(xle - p / 100 * mac, 3)
    m['cg_forward_limit'] = {'mass_kg': [round(empty), round(mtow)], 'x_m': [pct(TOP['fwd_cg_pct'][0]), pct(TOP['fwd_cg_pct'][1])]}
    m['cg_aft_limit'] = {'mass_kg': [round(empty), round(mtow)], 'x_m': [pct(TOP['aft_cg_pct']), pct(TOP['aft_cg_pct'])]}

    # ---- Fuselage aerodynamics and drag build-up (scaled with wetted area, nacelle size, wing area) ----
    sw = sL * sR
    fu, na = d['fuselages']
    fu.update(reference_point_m=r3([-0.46 * s['L'], 0, 0]), volume_m3=round(105.0 * sL * sR * sR, 1), crossflow_area_m2=round(74.0 * sL * sR, 1))
    dn = (rn / 0.79) ** 2
    eng_x = tail0 + (-25.9 + 19.5) * sR
    if s['engines'] == 'aft':
        eng_y = s['R'] + 1.15 * rn + 0.35 * sR
        eng_z = -0.55 * sR
    else:
        eng_y = 0.29 * half
        # Under the wing at that station: nozzle near the wing's leading edge, below it.
        t = eng_y / (pin['span_m'] + pout['span_m'])
        x_le_y = (root_x + 0.25 * pin['root_chord_m']) - eng_y * math.tan(math.radians(lam_out)) * 0.95
        eng_x = x_le_y - 0.4
        eng_z = zw + 0.75 * rn  # the nacelle's top just under the wing's lower surface, ahead of it
    na.update(reference_point_m=r3([eng_x + 1.5 * rn, 0, eng_z]), volume_m3=round(9.0 * dn * (rn / 0.79), 2), crossflow_area_m2=round(7.0 * dn, 2))
    k_drag = s.get('drag_scale', 1.0)
    d['drag_items'] = [
        {'name': 'fuselage', 'position_m': r3([-0.46 * s['L'], 0, 0]), 'drag_area_m2': round(0.62 * sw * k_drag, 4), 'critical_mach': 0.86},
        {'name': 'nacelles and pylons', 'position_m': r3([eng_x + 1.5 * rn, 0, eng_z]), 'drag_area_m2': round(0.3 * dn * k_drag, 4), 'critical_mach': 0.84},
        {'name': 'wing-body fairing, antennas, excrescences', 'position_m': r3([cg_x, 0, 0.6 * sR]), 'drag_area_m2': round(0.22 * S / 117.0 * k_drag, 4)},
        {'name': 'interference', 'position_m': r3([cg_x - 0.5, 0, 0.3 * sR]), 'drag_area_m2': round(0.1 * S / 117.0 * k_drag, 4)},
    ]

    # ---- Engines ----
    for i, e in enumerate(d['turbofans']):
        side = -1 if i == 0 else 1
        e['position_m'] = r3([eng_x, side * eng_y, eng_z])
        e['static_thrust_kn'] = round(thrust_kn, 2)
        e['bypass_ratio'] = s['bpr']
        e['tsfc_static_lb_per_lbf_h'] = s['tsfc']
        e['idle_fuel_flow_kg_per_h'] = round(280 * thrust_kn / TOP['thrust_kn'], 1)
        e['start_time_s'] = 35.0 if thrust_kn < 30 else 45.0
        e['acceleration_time_s'] = 5.0 if thrust_kn < 30 else 6.0
        e['lapse_mach_coefficient'] = s.get('lapse', 0.18)
        if s['reversers']:
            e['thrust_reverser'] = {'effectiveness': 0.4, 'transit_time_s': 2.0, 'reverse_n1': 0.8, 'stow_kias': 60,
                                    'cascade_forward_of_nozzle_m': round(1.5 * rn / 0.79, 2), 'cowl_radius_m': round(rn, 3)}
        else:
            e.pop('thrust_reverser', None)

    # ---- Fuel: two wing tanks (feeding their engines) ----
    for i, t in enumerate(d['fuel']['tanks']):
        side = -1 if i == 0 else 1
        t['position_m'] = r3([cg_x + 0.1 * mac, side * 0.35 * half, zw - 0.1])
        t['capacity_l'] = round(fuel_l / 2, 1)
        t['unusable_l'] = round(40 * fuel_l / 27328, 1)
        t['initial_l'] = round(0.45 * fuel_l / 2, 1)
    d['fuel']['density_kg_per_l'] = s['fuel_density']

    # ---- Gear: the nose leg forward, the mains at the Top G's % MAC; legs and wheels scaled ----
    kmass = mtow / TOP['mtow']
    kw = kmass ** (1 / 3)
    for g in d['gear']:
        nose_leg = g['name'] == 'nose'
        if nose_leg:
            g['attach_m'] = r3([-3.1 * sR if s['L'] < 25 else -3.1 * sR * 1.25, 0, 1.0 * sR])
        else:
            y = 2.25 * sR if s['engines'] == 'aft' else 2.86
            g['attach_m'] = r3([xle - TOP['mains_pct'] / 100 * mac, (-1 if 'left' in g['name'] else 1) * y, 1.0 * sR])
        g['wheel_radius_m'] = round(g['wheel_radius_m'] * kw, 3)
        if s['engines'] == 'wing':
            # Legs long enough for half a metre under the nacelles.
            g['length_m'] = round(eng_z + rn + 0.5 - g['attach_m'][2] - g['wheel_radius_m'], 3)
        else:
            g['length_m'] = round(1.05 * sR, 3)
        for k in ('stiffness_n_per_m', 'preload_n', 'damping_compression_ns_per_m', 'damping_rebound_ns_per_m',
                  'cornering_stiffness_n_per_rad', 'longitudinal_stiffness_n_per_m'):
            g[k] = round(g[k] * kmass)
        g['max_compression_m'] = round(g['max_compression_m'] * max(sR, 0.6), 3)
    # The nose tyre at the mains' cornering stiffness per newton of static load
    # (the Top G's nose tyre is stiffer per newton). Mass-scaled like the rest,
    # a shorter wheelbase brings the nose tyre's destabilising side force within
    # reach of the take-off speed: the small jets ground-looped near Vr.
    nose, mains = d['gear'][0], d['gear'][1:]
    xn, xm = nose['attach_m'][0], mains[0]['attach_m'][0]
    x_mid = pct((TOP['fwd_cg_pct'][1] + TOP['aft_cg_pct']) / 2)  # mid CG envelope at MTOW
    f_nose = (x_mid - xm) / (xn - xm)
    per_newton = mains[0]['cornering_stiffness_n_per_rad'] / (mtow * 9.80665 * (1 - f_nose) / len(mains))
    nose['cornering_stiffness_n_per_rad'] = round(per_newton * mtow * 9.80665 * f_nose)
    # Contacts.
    tip_y = s['span'] / 2
    for c in d['contacts']:
        n = c['name']
        x, y, z = c['position_m']
        if 'wingtip' in n:
            c['position_m'] = r3([qx2 - 0.25 * pout['tip_chord_m'], (1 if x and y > 0 else -1) * tip_y if y else 0, zt])
        elif 'nacelle' in n:
            c['position_m'] = r3([eng_x + 0.9 * rn, (1 if y > 0 else -1) * eng_y, eng_z + rn])
        elif 'fin' in n:
            c['position_m'] = r3([x_ht - 0.3, 0, fin_top_z])
        elif n == 'tail skid':
            c['position_m'] = r3([tail0 + (-24.8 + 19.5) * sR, 0, 0.05 * sR])
        else:
            c['position_m'] = r3([xs(x) if xs(x) is not None else x * sL, y * sR, z * sR])

    # ---- Controls, limits, speedbrake, systems ----
    ctl = d['controls']
    ctl['flap_detents_deg'] = flaps
    # Yaw damper, as these types have: washed-out yaw rate to the rudder.
    ctl['yaw_damper'] = {'gain_s': 2.0, 'reference_kias': 160, 'washout_s': 3.0, 'authority': 0.3}
    ctl['landing_gear']['drag_area_m2'] = round(2.0 * sw, 3)
    ctl['landing_gear']['vle_kias'] = s['vle']
    sb = ctl['speedbrake']
    sb['detents'] = s['speedbrake']['detents']
    sb['drag_area_m2'] = round(1.6 * S / 117.0, 3)
    sb['ground_spoilers'] = True
    lim = d['limits']
    lim.update(vne_kias=s['vmo'], vno_kias=s['vmo'], va_kias=round(0.65 * s['vmo']), vfe_kias=s['vfe'][-1], mmo=s['mmo'])
    lim['vfe_vs_flap'] = {'flap_deg': flaps[1:], 'kias': s['vfe']}
    asi = d['panel']['airspeed_indicator']
    asi['red_line_kias'] = s['vmo']
    asi['green_arc_kias'] = [asi['green_arc_kias'][0], s['vmo']]
    asi['yellow_arc_kias'] = [s['vmo'], s['vmo']]
    d['pitot_static']['pitot_position_m'] = r3([-2.2 * sR, -0.9 * sR, 0.2 * sR])
    d['pitot_static']['static_position_m'] = r3([-4.5 * sR, -1.25 * sR, 0.1 * sR])
    d['panel']['annunciators']['low_fuel_below_gal'] = round(300 * fuel_l / 27328)

    # ---- Identity, provenance ----
    d['id'] = key
    d['display_name'] = s['display']
    d['manufacturer'] = s['manufacturer']
    d['reference_aircraft'] = s['reference']
    d['description'] = (f"{s['display']}: generated by tools/jetgen from published figures (dimensions, wing area and sweep, weights, fuel, "
                        f"engines, speed limits, flap settings) on the Top G's validated arrangement, then calibrated (see provenance).")
    d['sources'] = [{'id': i, 'citation': c} for i, c in s['sources']] + [x for x in top['sources'] if x['id'] in ('MATTINGLY', 'ROSKAM', 'EST')]
    reverser_note = 'cascade thrust reversers (estimated effectiveness and schedule)' if s['reversers'] else 'no thrust reversers'
    d['provenance'] = [
        {'field': 'reference, mass.max_*, fuel, turbofans[*].static_thrust_kn', 'kind': 'published', 'source': s['sources'][0][0],
         'note': f"Span {s['span']} m, wing area {s['wing_area']} m2, length {s['L']} m; MTOW {s['mtow_lb']} lb, MLW {s['mlw_lb']} lb, "
                 f"empty {s['empty_lb']} lb (BOW less crew where only BOW is published), {s['thrust_lbf']} lbf a side (see sources)."},
        {'field': 'surfaces[0]', 'kind': 'estimated', 'source': 'EST',
         'note': f"The Top G's planform family (crank at 28 % semi-span, taper, washout, twist, sections, flap/aileron/spoiler layout) at this type's "
                 f"span, area and {s['sweep']} deg quarter-chord sweep{' with winglets' if s['winglets'] else ''}."},
        {'field': 'surfaces[1], surfaces[2]', 'kind': 'estimated', 'source': 'EST',
         'note': f"Tail areas from the Top G's volume coefficients at this type's arms (Vh 0.69; Vv {vv:.3f}: the Top G's 0.058 plus what this "
                 f"fuselage's extra destabilising moment needs); {'T-tail' if s['t_tail'] else 'low-set tail'}."},
        {'field': 'controls.yaw_damper', 'kind': 'estimated', 'source': 'EST',
         'note': 'Yaw damper (these types have one): washed-out yaw rate (3 s) to the rudder, 2 per rad/s at 160 KEAS scheduled with 1/EAS^2, 30 % authority.'},
        {'field': 'mass.empty_cg_m, mass.cg_*_limit, gear', 'kind': 'estimated', 'source': 'EST',
         'note': 'Empty CG 27 % MAC, CG limits 7-11 % to 36 % MAC, main gear at 45 % MAC (the Top G arrangement); inertia scaled by mass and size.'},
        {'field': 'drag_items', 'kind': 'estimated', 'source': 'ROSKAM', 'note': 'The Top G build-up scaled with fuselage wetted area, nacelle size and wing area.'},
        {'field': 'controls.speedbrake, turbofans[*].thrust_reverser', 'kind': 'published', 'source': s['sources'][-1][0],
         'note': f"Speedbrake with automatic ground spoilers; {reverser_note}."},
        {'field': 'limits, controls.flap_detents_deg', 'kind': 'published', 'source': s['sources'][-1][0],
         'note': f"VMO {s['vmo']} KIAS, MMO {s['mmo']}; flaps {flaps} with VFE {s['vfe']} (some flap placards estimated)."},
    ]
    if s.get('calibration'):
        d['provenance'].extend(s['calibration'])

    # ---- The 3-D model's shape and livery (renderers only) ----
    d['visual'] = {
        'short_name': s['short'],
        'fuselage_stations': stations,
        'nose_scale': round(sR, 4),
        'windows': s['windows'],
        'nacelle': {'radius': round(rn, 3), 'length': round(4.1 * rn / 0.79, 3), 'mount': s['engines']},
        'colors': s['colors'],
        'winglets': s['winglets'],
        # Body FLU (x forward, y left, z up) from the nose-tip datum.
        'camera': {'chase': round(max(25, 1.6 * s['L']), 1), 'target': r3([cg_x, 0, 0.6 * sR]), 'eye': r3([-3.4 * sR, 0.52 * sR, 0.42 * sR])},
    }
    return d


def main():
    top = json.load(open(os.path.join(DATA, 'top_g.json')))
    calib = {}
    cpath = os.path.join(HERE, 'calibration.json')
    if os.path.exists(cpath):
        calib = json.load(open(cpath))
    keys = sys.argv[1:] or list(SPECS)
    for k in keys:
        spec = dict(SPECS[k])
        spec.update(calib.get(k, {}))
        d = build(k, spec, top)
        path = os.path.join(DATA, f'{k}.json')
        with open(path, 'w') as f:
            f.write(json.dumps(d, indent=1, ensure_ascii=False) + '\n')
        print('wrote', os.path.relpath(path, REPO))


if __name__ == '__main__':
    main()
