#!/usr/bin/env python3
"""Generate the control-theory block diagrams (SVG) in this directory.

Usage: python3 gen.py   (requires: pip install schemdraw)

Output SVGs embed a prefers-color-scheme style block so one file serves
both GitHub themes.
"""
from pathlib import Path

import schemdraw

schemdraw.use('svg')
from schemdraw import dsp  # noqa: E402
import schemdraw.elements as elm  # noqa: E402

OUT = Path(__file__).parent
FS = 11        # base fontsize
FSS = 9.5      # small annotation fontsize
BH = 1.3       # box height

# one color code across every diagram, chosen to read on both github themes:
# gold = references, blue/green/amber = position/velocity/current loop,
# red = limits and protection, purple = estimator layer, gray = hardware
C_TRA = '#9a7b0a'
C_POS = '#1f6feb'
C_VEL = '#2da44e'
C_CUR = '#f59f00'
C_LIM = '#d4434b'
C_EST = '#8957e5'
C_HW = '#6e7781'

# dark-mode variants: each palette color brightens so it stays legible on a
# dark background; black ink flips to github's dark-theme foreground
DARK_MAP = {
    'black': '#c9d1d9',
    C_TRA: '#dfb317',
    C_POS: '#539bf5',
    C_VEL: '#57ab5a',
    C_CUR: '#f8b133',
    C_LIM: '#f47067',
    C_EST: '#b083f0',
    C_HW: '#909aa4',
}
# covers both emission forms: inline style (shapes) and bare attribute (text)
DARK_CSS = ('<style>@media (prefers-color-scheme:dark){' + ''.join(
    f'[style*="stroke:{c}"]{{stroke:{lc} !important}}'
    f'[style*="fill:{c}"]{{fill:{lc} !important}}'
    f'[stroke="{c}"]{{stroke:{lc} !important}}'
    f'[fill="{c}"]{{fill:{lc} !important}}'
    for c, lc in DARK_MAP.items()) + '}</style>')


def save(d, name):
    path = OUT / name
    d.save(str(path))
    svg = path.read_text()
    svg = svg.replace('>', '>' + DARK_CSS, 1)
    path.write_text(svg)
    print('wrote', name)


def sign(d, pt, txt, dx=0.0, dy=0.0):
    d += elm.Label(label=txt, fontsize=FSS).at((pt[0] + dx, pt[1] + dy))


def diagram():
    d = schemdraw.Drawing(show=False)
    d.config(fontsize=FS)
    return d


# ---------------------------------------------------------------- cascade
d = diagram()
d += elm.Arrow().right(1.2).label('goal', loc='top').color(C_TRA)
d += (traj := dsp.Box(w=2.0, h=BH).anchor('W').label('TRAJ').color(C_TRA))
d += elm.Arrow().at(traj.E).right(1.4).label('θ*, ω*, α*', loc='top', fontsize=FSS).color(C_TRA)
d += (pos := dsp.Box(w=2.2, h=BH).anchor('W').label('POSITION').color(C_POS))
d += elm.Arrow().at(pos.E).right(1.2).label('ω_ref', loc='top', fontsize=FSS).color(C_POS)
d += (vel := dsp.Box(w=2.2, h=BH).anchor('W').label('VELOCITY').color(C_VEL))
d += elm.Arrow().at(vel.E).right(1.2).label('i_cmd', loc='top', fontsize=FSS).color(C_VEL)
d += (lim := dsp.Box(w=1.8, h=BH).anchor('W').label('LIMITS').color(C_LIM))
d += elm.Arrow().at(lim.E).right(1.2).label('i*', loc='top', fontsize=FSS).color(C_LIM)
d += (cur := dsp.Box(w=2.2, h=BH).anchor('W').label('CURRENT').color(C_CUR))
d += elm.Arrow().at(cur.E).right(1.2).label('duty', loc='top', fontsize=FSS).color(C_CUR)
d += (pwm := dsp.Box(w=2.4, h=BH).anchor('W').label('PWM + DRV').color(C_HW))
d += elm.Arrow().at(pwm.E).right(1.2).color(C_HW)
d += (plant := dsp.Box(w=2.6, h=BH).anchor('W').label('motor + gear').color(C_HW))

mid_x = (pos.absanchors['W'][0] + cur.absanchors['E'][0]) / 2
d += (est := dsp.Box(w=13.0, h=BH).at((mid_x, -2.6)).anchor('N').label('ESTIMATOR STACK').color(C_EST))
# plant sensors -> estimator (right side entry)
pS = plant.absanchors['S']
eE = est.absanchors['E']
d += elm.Line().at(pS).down(pS[1] - eE[1]).color(C_HW)
d += elm.Arrow().left(pS[0] - eE[0]).label('shunt, terminal volts, encoders', loc='top', fontsize=FSS).color(C_HW)
# estimator outputs up to loops
for box, lbl in ((pos, 'θ̂'), (vel, 'ω̂'), (lim, 'T̂wind'), (cur, 'î, ω̂, V̂bus')):
    bS = box.absanchors['S']
    eT = est.absanchors['N']
    d += elm.Arrow().at((bS[0], eT[1])).up(bS[1] - eT[1]).label(lbl, loc='bottom', ofst=(0.32, -0.42), fontsize=FSS).color(C_EST)
save(d, 'cascade.svg')

# ---------------------------------------------------------- position loop
d = diagram()
d += elm.Arrow().right(1.4).label('θ*', loc='top').color(C_TRA)
d += (s1 := dsp.Sum().anchor('W').color(C_POS))
sign(d, s1.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s1.absanchors['SW'], '−', dx=-0.28, dy=-0.35)
d += elm.Arrow().at(s1.E).right(1.2).label('eθ', loc='top', fontsize=FSS).color(C_POS)
d += (kp := dsp.Box(w=1.8, h=BH).anchor('W').label('P').color(C_POS))
d += elm.Arrow().at(kp.E).right(1.2).color(C_POS)
d += (s2 := dsp.Sum().anchor('W').color(C_POS))
sign(d, s2.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s2.absanchors['N'], '+', dx=0.34, dy=0.35)
d += elm.Arrow().at(s2.E).right(1.2).color(C_POS)
d += (cl := dsp.Box(w=3.0, h=BH).anchor('W').label('clamp ±velocity_limit').color(C_POS))
d += elm.Arrow().at(cl.E).right(1.4).label('ω_ref', loc='top').color(C_POS)
# feedbacks
d += elm.Arrow().at((s1.absanchors['S'][0], s1.absanchors['S'][1] - 1.2)).up(1.2).label('θ̂', loc='bottom').color(C_EST)
d += elm.Arrow().at((s2.absanchors['N'][0], s2.absanchors['N'][1] + 1.2)).down(1.2).label('ω*  (ff from TRAJ)', loc='top').color(C_TRA)
d += elm.Label(label='tier 3: runs in the motor frame (θ̂_m, references ×N)', fontsize=FSS).at(
    (kp.absanchors['S'][0] + 2.0, s1.absanchors['S'][1] - 1.8))
save(d, 'position-loop.svg')

# ---------------------------------------------------------- velocity loop
d = diagram()
d += elm.Arrow().right(1.4).label('ω_ref', loc='top').color(C_POS)
d += (s1 := dsp.Sum().anchor('W').color(C_VEL))
sign(d, s1.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s1.absanchors['SW'], '−', dx=-0.28, dy=-0.35)
d += elm.Arrow().at(s1.E).right(1.2).label('eω', loc='top', fontsize=FSS).color(C_VEL)
d += (pi := dsp.Box(w=2.6, h=1.6).anchor('W').label('PI\n(anti-windup)').color(C_VEL))
d += elm.Arrow().at(pi.E).right(1.2).color(C_VEL)
d += (s2 := dsp.Sum().anchor('W').color(C_VEL))
sign(d, s2.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s2.absanchors['N'], '+', dx=0.34, dy=0.35)
d += elm.Arrow().at(s2.E).right(1.4).label('i_cmd', loc='top').color(C_VEL)
# feedback + FF
d += elm.Arrow().at((s1.absanchors['S'][0], s1.absanchors['S'][1] - 1.2)).up(1.2).label('ω̂', loc='bottom').color(C_EST)
d += (ff := dsp.Box(w=6.8, h=1.7).at((s2.absanchors['N'][0], s2.absanchors['N'][1] + 1.5)).anchor('S').label(
    'mechanical FF\n(J·α* + τ̂fric(ω_ref) + τcog(θ̂_m)) · 1/Kt').color(C_VEL))
d += elm.Arrow().at(ff.S).down(1.5).color(C_VEL)
d += elm.Arrow().at((ff.absanchors['W'][0] - 1.6, ff.absanchors['W'][1])).right(1.6).label('α*  (TRAJ)', loc='top', fontsize=FSS).color(C_TRA)
d += elm.Arrow().at((ff.absanchors['N'][0] - 1.6, ff.absanchors['N'][1] + 0.9)).down(0.9).label('ω_ref', loc='top', fontsize=FSS).color(C_POS)
d += elm.Arrow().at((ff.absanchors['E'][0] + 1.8, ff.absanchors['E'][1])).left(1.8).color(C_EST)
d += elm.Label(label='θ̂_m  (estimator)', fontsize=FSS, color=C_EST).at((ff.absanchors['E'][0] + 1.3, ff.absanchors['E'][1] + 0.45))
d += elm.Arrow(ls='--').at((pi.absanchors['S'][0] + 2.8, pi.absanchors['S'][1] - 1.3)).theta(155).length(3.0).label(
    'back-calc (from LIMITS)', loc='bottom', fontsize=FSS).color(C_LIM)
save(d, 'velocity-loop.svg')

# ----------------------------------------------------------------- limits
d = diagram()
d += elm.Arrow().right(1.4).label('i_cmd', loc='top').color(C_VEL)
d += (cl := dsp.Box(w=2.6, h=BH).anchor('W').label('clamp ±i_lim').color(C_LIM))
d += elm.Arrow().at(cl.E).right(1.4).label('i*', loc='top').color(C_LIM)
d += (mn := dsp.Box(w=11.0, h=1.6).at((cl.absanchors['S'][0], cl.absanchors['S'][1] - 2.6)).anchor('S').label(
    'i_lim = min( torque_limit,  derate(T̂wind),  stall policy,  endstop(θ̂) )').color(C_LIM))
d += elm.Arrow().at(mn.N).up(2.6 - 1.6).label('i_lim', loc='bottom', ofst=(0.35, -0.3), fontsize=FSS).color(C_LIM)
d += elm.Arrow().at((mn.absanchors['W'][0] - 2.4, mn.absanchors['W'][1])).right(2.4).label('T̂wind, θ̂, τ̂d', loc='top', ofst=(-0.3, 0.1), fontsize=FSS).color(C_EST)
d += elm.Arrow(ls='--').at(cl.N).up(1.0).label('back-calc -> VEL integrator', loc='top', fontsize=FSS).color(C_LIM)
save(d, 'limits.svg')

# ----------------------------------------------------------- current loop
d = diagram()
d += elm.Arrow().right(1.4).label('i*', loc='top').color(C_LIM)
d += (s1 := dsp.Sum().anchor('W').color(C_CUR))
sign(d, s1.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s1.absanchors['SW'], '−', dx=-0.28, dy=-0.35)
d += elm.Arrow().at(s1.E).right(1.0).label('ei', loc='top', fontsize=FSS).color(C_CUR)
d += (pi := dsp.Box(w=1.8, h=BH).anchor('W').label('PI').color(C_CUR))
d += elm.Arrow().at(pi.E).right(1.0).color(C_CUR)
d += (s2 := dsp.Sum().anchor('W').color(C_CUR))
sign(d, s2.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s2.absanchors['N'], '+', dx=0.34, dy=0.35)
d += elm.Arrow().at(s2.E).right(1.0).label('v_cmd', loc='top', fontsize=FSS).color(C_CUR)
d += (rc := dsp.Box(w=2.0, h=BH).anchor('W').label('× 1/V̂bus').color(C_CUR))
d += elm.Arrow().at(rc.E).right(1.0).color(C_CUR)
d += (dc := dsp.Box(w=2.8, h=BH).anchor('W').label('clamp ±duty_max').color(C_LIM))
d += elm.Arrow(ls='--').at(dc.S).down(1.1).label('back-calc -> PI integrator', loc='bottom', fontsize=FSS).color(C_LIM)
d += elm.Arrow().at(dc.E).right(1.0).label('duty', loc='top', fontsize=FSS).color(C_LIM)
d += (pw := dsp.Box(w=2.6, h=BH).anchor('W').label('PWM + decay').color(C_HW))
d += elm.Arrow().at(pw.E).right(1.2).label('to DRV', loc='top', fontsize=FSS).color(C_HW)
d += elm.Arrow().at((s1.absanchors['S'][0], s1.absanchors['S'][1] - 1.2)).up(1.2).label('î', loc='bottom').color(C_EST)
d += elm.Arrow().at((s2.absanchors['N'][0], s2.absanchors['N'][1] + 1.2)).down(1.2).label('Ke·ω̂  (BEMF decoupling)', loc='top').color(C_EST)
d += elm.Arrow().at((rc.absanchors['N'][0], rc.absanchors['N'][1] + 1.2)).down(1.2).label('V̂bus', loc='top').color(C_EST)
d += elm.Arrow(ls='--').at(pw.S).down(1.1).label('sampling-window command -> ADC scheduler', loc='bottom', fontsize=FSS).color(C_HW)
save(d, 'current-loop.svg')

# ----------------------------------------------------------- dual encoder
d = diagram()
d += elm.Line().right(1.6).label('θ*_out', loc='top').color(C_TRA)
d += (dot := elm.Dot(radius=0.06).color(C_TRA))
d += elm.Arrow().at(dot.center).right(1.0).color(C_TRA)
d += (nx := dsp.Box(w=1.4, h=BH).anchor('W').label('× N').color(C_TRA))
d += elm.Arrow().at(nx.E).right(2.8).color(C_TRA)
d += (s1 := dsp.Sum().anchor('W').color(C_TRA))
sign(d, s1.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s1.absanchors['SW'], '+', dx=-0.3, dy=-0.4)
d += elm.Arrow().at(s1.E).right(1.2).label('θ*_m', loc='top', fontsize=FSS).color(C_TRA)
d += (fc := dsp.Box(w=4.6, h=1.6).anchor('W').label('fast cascade\n(motor frame, IR θ̂_m ω̂_m)'))
d += elm.Arrow().at(fc.E).right(1.2).label('to PWM', loc='top', fontsize=FSS).color(C_HW)
# trim path
dc = dot.absanchors['center']
d += (s2 := dsp.Sum().at((dc[0], -3.0)).anchor('center').color(C_POS))
sign(d, s2.absanchors['NW'], '+', dx=-0.28, dy=0.15)
sign(d, s2.absanchors['SW'], '−', dx=-0.3, dy=-0.4)
d += elm.Arrow().at(dc).down(3.0 - 0.5).color(C_TRA)
d += elm.Arrow().at(s2.E).right(1.2).color(C_POS)
d += (tr := dsp.Box(w=3.2, h=1.6).anchor('W').label('Ki_trim / s\n(slow, ~1 Hz)').color(C_POS))
trE = tr.absanchors['E']
s1S = s1.absanchors['S']
d += elm.Line().at(trE).right(s1S[0] - trE[0]).color(C_POS)
d += elm.Arrow().at((s1S[0], trE[1])).up(s1S[1] - trE[1]).color(C_POS)
d += elm.Label(label='trim', fontsize=FSS).at((s1S[0] + 0.45, trE[1] + 0.9))
d += elm.Arrow().at((s2.absanchors['S'][0], s2.absanchors['S'][1] - 1.2)).up(1.2).label('θ̂_out', loc='bottom').color(C_EST)
d += elm.Label(label='the ONLY output-frame integrator', fontsize=FSS).at((tr.absanchors['S'][0] + 0.6, tr.absanchors['S'][1] - 0.9))
# backlash estimator, clear of the trim row
d += (bl := dsp.Box(w=5.6, h=1.6).at((fc.absanchors['E'][0] - 2.8, -5.8)).anchor('N').label(
    'backlash estimator\nb̂ = hysteresis of (θ̂_m/N − θ̂_out)').color(C_EST))
d += elm.Arrow().at((bl.absanchors['W'][0] - 2.4, bl.absanchors['W'][1])).right(2.4).label('θ̂_m, θ̂_out', loc='top', fontsize=FSS).color(C_EST)
d += elm.Arrow(ls='--').at(bl.E).right(1.3).label('b̂ -> trim deadband,\ncontact-detect mask', loc='right', fontsize=FSS).color(C_EST)
save(d, 'dual-encoder.svg')

# ------------------------------------------------------- sensor frontends
d = diagram()
rows = [
    ('shunt ADC', [('window gate', 2.4), ('− offset₀', 2.0), ('× gain_cal', 2.2)], 'î', '', 0.0),
    ('pot ADC', [('oversample ×16', 2.9), ('LUT_pot', 1.9)], 'θ_out', 'tier 1', -5.2),
    ('mag sin/cos', [('ellipse correct', 2.8), ('atan2 CORDIC', 2.6), ('LUT_lin', 1.9)], 'θ_out', 'tier 2', -7.1),
    ('IR ×2 ADC', [('per-unit curve LUT', 3.3), ('fine angle + segment', 3.6)], 'θ_m', 'tier 3', -9.0),
]
for src, boxes, out, tier, y in rows:
    d.here = (0, y)
    d += elm.Arrow().right(2.4).label(src, loc='top').color(C_HW)
    for lbl, w in boxes:
        d += (b := dsp.Box(w=w, h=1.1).anchor('W').label(lbl).color(C_EST))
        d += elm.Arrow().at(b.E).right(1.0).color(C_EST)
    d += elm.Label(label=out, color=C_EST).at((d.here[0] + 0.35, d.here[1]))
    if tier:
        d += elm.Label(label=tier, fontsize=FSS).at((d.here[0] + 0.55, d.here[1] + 0.42))
# terminal chain: v-diff plus the V̂bus branch (no separate supply divider -
# a driven-high terminal sits at the bus rail)
d.here = (0, -1.9)
d += elm.Arrow().right(2.4).label('terminal ADC ×2', loc='top').color(C_HW)
d += (pa := dsp.Box(w=2.6, h=1.1).anchor('W').label('PWM average').color(C_EST))
d += elm.Arrow().at(pa.E).right(1.0).color(C_EST)
d += (df := dsp.Box(w=1.4, h=1.1).anchor('W').label('diff').color(C_EST))
d += elm.Arrow().at(df.E).right(1.0).color(C_EST)
d += (gc := dsp.Box(w=2.2, h=1.1).anchor('W').label('× gain_cal').color(C_EST))
d += elm.Arrow().at(gc.E).right(1.0).color(C_EST)
d += elm.Label(label='v̂_diff', color=C_EST).at((d.here[0] + 0.35, d.here[1]))
bx = pa.absanchors['E'][0] + 0.5
d += elm.Dot(radius=0.06).at((bx, -1.9)).color(C_EST)
d += elm.Line().at((bx, -1.9)).down(1.4).color(C_EST)
d += elm.Arrow().at((bx, -3.3)).right(0.9).color(C_EST)
d += (og := dsp.Box(w=3.0, h=1.1).anchor('W').label('on-phase gate').color(C_EST))
d += elm.Arrow().at(og.E).right(1.0).color(C_EST)
d += (lpf := dsp.Box(w=1.4, h=1.1).anchor('W').label('LPF').color(C_EST))
d += elm.Arrow().at(lpf.E).right(1.0).color(C_EST)
d += elm.Label(label='V̂bus', color=C_EST).at((d.here[0] + 0.4, d.here[1]))
d += elm.Label(label='(driven-high terminal = bus rail)', fontsize=FSS).at((lpf.absanchors['E'][0] + 2.3, -4.35))
save(d, 'sensor-frontends.svg')

# --------------------------------------------------- bemf + thermometry
d = diagram()
d += elm.Arrow().right(1.6).label('v̂_diff', loc='top').color(C_EST)
d += (s1 := dsp.Sum().anchor('W').color(C_EST))
sign(d, s1.absanchors['NW'], '+', dx=-0.25, dy=0.1)
sign(d, s1.absanchors['SW'], '−', dx=-0.28, dy=-0.4)
sign(d, s1.absanchors['SE'], '−', dx=0.3, dy=-0.4)
d += elm.Arrow().at(s1.E).right(1.2).label('ê', loc='top', fontsize=FSS).color(C_EST)
d += (ke := dsp.Box(w=1.8, h=BH).anchor('W').label('× 1/Ke').color(C_EST))
d += elm.Arrow().at(ke.E).right(1.0).color(C_EST)
d += (lp := dsp.Box(w=1.5, h=BH).anchor('W').label('LPF').color(C_EST))
d += elm.Arrow().at(lp.E).right(1.6).label('ω̂_bemf', loc='top').color(C_EST)
dse = s1.absanchors['SE']
d += elm.Arrow().at((dse[0] + 0.6, dse[1] - 0.85)).theta(125).length(1.05).color(C_EST)
d += elm.Label(label='L·dî/dt  (drop when PWM-averaged)', fontsize=FSS).at((dse[0] + 3.0, dse[1] - 1.1))
# thermometry chain below
ty = -3.4
d.here = (0.2, ty)
d += (gt := dsp.Box(w=4.4, h=1.6).anchor('W').label('gate: |î| high AND\n(|ω̂| ≈ 0 OR encoder ω̂)').color(C_EST))
d += elm.Arrow().at(gt.E).right(1.0).color(C_EST)
d += (ir := dsp.Box(w=4.2, h=1.6).anchor('W').label('slow IIR\nR̂ = (v̂_diff − Ke·ω̂) / î').color(C_EST))
d += elm.Arrow().at(ir.E).right(1.0).color(C_EST)
d += (tw := dsp.Box(w=4.2, h=1.6).anchor('W').label('T̂wind =\nT₀ + (R̂/R₀ − 1)/0.00393').color(C_EST))
d += elm.Arrow().at(tw.E).right(1.4).label('-> derate()', loc='top', fontsize=FSS).color(C_LIM)
d += elm.Arrow(ls='--').at((gt.absanchors['S'][0], gt.absanchors['S'][1] - 1.0)).up(1.0).label(
    'cal-time anchor: (R₀, T₀)', loc='bottom', fontsize=FSS).color(C_HW)
# R̂ feedback up to the BEMF sum
irN = ir.absanchors['N']
s1S = s1.absanchors['S']
d += elm.Line().at(irN).up(0.8).color(C_EST)
d += elm.Label(label='R̂(T)·î', fontsize=FSS, color=C_EST).at((irN[0] + 0.65, irN[1] + 0.4))
d += elm.Line().at((irN[0], irN[1] + 0.8)).left(irN[0] - s1S[0]).color(C_EST)
d += elm.Arrow().at((s1S[0], irN[1] + 0.8)).up(s1S[1] - irN[1] - 0.8).color(C_EST)
d += elm.Label(label='timescale separation resolves the R̂ <-> ω̂_bemf circularity: R̂ moves in seconds, ω̂_bemf in milliseconds',
               fontsize=FSS).at((tw.absanchors['S'][0] - 2.0, tw.absanchors['S'][1] - 1.0))
save(d, 'bemf-thermometry.svg')

# --------------------------------------------------------- kalman fusion
d = diagram()
d += elm.Arrow().right(1.6).label('î', loc='top').color(C_EST)
d += (pr := dsp.Box(w=5.2, h=2.0).anchor('W').label('predict (model)\nω̂ += (Kt·î − τ̂fric − τ̂d)·Ts/J\nθ̂ += ω̂·Ts').color(C_EST))
d += elm.Arrow().at(pr.E).right(1.4).color(C_EST)
d += (co := dsp.Box(w=5.2, h=2.0).anchor('W').label('correct\n[θ̂, ω̂, τ̂d] += [L1, L2, L3]·innov\ninnov = θ_meas − θ̂').color(C_EST))
d += elm.Arrow().at(co.E).right(1.5).label('θ̂, ω̂, τ̂d', loc='top').color(C_EST)
# measurement inputs
coN = co.absanchors['N']
d += elm.Arrow().at((coN[0] - 1.2, coN[1] + 1.9)).down(1.9).label(
    'θ_meas  (pot·N / mag / IR)', loc='top', fontsize=FSS).color(C_EST)
d += elm.Arrow(ls='--').at((coN[0] + 1.2, coN[1] + 0.85)).down(0.85).color(C_EST)
d += elm.Label(label='ω̂_bemf  (small weight)', fontsize=FSS, color=C_EST).at((coN[0] + 3.1, coN[1] + 0.8))
# state feedback
coS = co.absanchors['S']
prS = pr.absanchors['S']
d += elm.Line().at(coS).down(1.0).color(C_EST)
d += elm.Line().at((coS[0], coS[1] - 1.0)).left(coS[0] - prS[0]).label('state feedback', loc='top', fontsize=FSS).color(C_EST)
d += elm.Arrow().at((prS[0], coS[1] - 1.0)).up(1.0).color(C_EST)
d += elm.Label(label='L1..L3 fixed — steady-state gains computed offline at cal (identified J, friction, sensor noise)',
               fontsize=FSS).at(((prS[0] + coS[0]) / 2 + 0.6, coS[1] - 1.75))
d += elm.Arrow(ls='--').at((co.absanchors['E'][0] + 0.4, co.absanchors['E'][1] - 0.55)).right(1.1).label(
    'τ̂d -> contact detect', loc='bottom', ofst=(0.2, -0.25), fontsize=FSS).color(C_EST)
save(d, 'kalman-fusion.svg')

# ------------------------------------------------ putting it all together
d = diagram()
d += elm.Arrow().right(0.9).label('goal', loc='top', fontsize=FSS).color(C_TRA)
d += (traj := dsp.Box(w=1.5, h=1.1).anchor('W').label('TRAJ').color(C_TRA))
d += elm.Arrow().at(traj.E).right(0.7).label('θ*', loc='top', fontsize=FSS).color(C_TRA)
d += (sp := dsp.Sum().anchor('W').color(C_POS))
sign(d, sp.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, sp.absanchors['SW'], '−', dx=-0.26, dy=-0.34)
d += elm.Arrow().at(sp.E).right(0.5).color(C_POS)
d += (kp := dsp.Box(w=1.2, h=1.1).anchor('W').label('P').color(C_POS))
d += elm.Arrow().at(kp.E).right(0.5).color(C_POS)
d += (sp2 := dsp.Sum().anchor('W').color(C_POS))
sign(d, sp2.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, sp2.absanchors['N'], '+', dx=-0.28, dy=0.3)
d += elm.Arrow().at(sp2.E).right(0.5).color(C_POS)
d += (clw := dsp.Box(w=1.5, h=1.1).anchor('W').label('clamp\n±ω_lim', fontsize=FSS).color(C_POS))
d += elm.Arrow().at(clw.E).right(0.8).label('ω_ref', loc='bottom', fontsize=FSS).color(C_POS)
d += (sv := dsp.Sum().anchor('W').color(C_VEL))
sign(d, sv.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, sv.absanchors['SW'], '−', dx=-0.26, dy=-0.34)
d += elm.Arrow().at(sv.E).right(0.5).color(C_VEL)
d += (piv := dsp.Box(w=1.2, h=1.1).anchor('W').label('PI').color(C_VEL))
d += elm.Arrow().at(piv.E).right(0.5).color(C_VEL)
d += (sv2 := dsp.Sum().anchor('W').color(C_VEL))
sign(d, sv2.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, sv2.absanchors['N'], '+', dx=-0.28, dy=0.3)
d += elm.Arrow().at(sv2.E).right(0.8).label('i_cmd', loc='top', fontsize=FSS).color(C_VEL)
d += (cli := dsp.Box(w=1.7, h=1.1).anchor('W').label('clamp\n±i_lim', fontsize=FSS).color(C_LIM))
d += elm.Arrow().at(cli.E).right(0.7).label('i*', loc='top', fontsize=FSS).color(C_LIM)
d += (si := dsp.Sum().anchor('W').color(C_CUR))
sign(d, si.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, si.absanchors['SW'], '−', dx=-0.26, dy=-0.34)
d += elm.Arrow().at(si.E).right(0.5).color(C_CUR)
d += (pic := dsp.Box(w=1.2, h=1.1).anchor('W').label('PI').color(C_CUR))
d += elm.Arrow().at(pic.E).right(0.5).color(C_CUR)
d += (si2 := dsp.Sum().anchor('W').color(C_CUR))
sign(d, si2.absanchors['NW'], '+', dx=-0.22, dy=0.08)
sign(d, si2.absanchors['SW'], '+', dx=-0.26, dy=-0.34)
d += elm.Arrow().at(si2.E).right(0.5).color(C_CUR)
d += (rc := dsp.Box(w=1.7, h=1.1).anchor('W').label('× 1/V̂bus', fontsize=FSS).color(C_CUR))
d += elm.Arrow().at(rc.E).right(0.6).color(C_CUR)
d += (dcl := dsp.Box(w=1.6, h=1.1).anchor('W').label('clamp\n±duty', fontsize=FSS).color(C_LIM))
d += elm.Arrow().at(dcl.E).right(0.6).label('duty', loc='top', fontsize=FSS).color(C_LIM)
d += (pwm := dsp.Box(w=1.9, h=1.1).anchor('W').label('PWM + DRV', fontsize=FSS).color(C_HW))
d += elm.Arrow().at(pwm.E).right(0.6).color(C_HW)
d += (plant := dsp.Box(w=2.0, h=1.1).anchor('W').label('motor + gear', fontsize=FSS).color(C_HW))

# feedforward row
LANE1 = 1.75
LANE2 = 2.75
tN = traj.absanchors['N']
p2N = sp2.absanchors['N']
d += elm.Line().at(tN).up(LANE1 - tN[1]).color(C_TRA)
d += elm.Dot(radius=0.06).at((tN[0], LANE1)).color(C_TRA)
d += elm.Line().at((tN[0], LANE1)).right(p2N[0] - tN[0]).label('ω*', loc='top', fontsize=FSS).color(C_TRA)
d += elm.Arrow().at((p2N[0], LANE1)).down(LANE1 - p2N[1]).color(C_TRA)
v2N = sv2.absanchors['N']
d += (ffb := dsp.Box(w=4.4, h=1.1).at((v2N[0], 1.25)).anchor('S').label(
    'mechanical feedforward\n(J·α* + τ̂fric + τ̂cog) / Kt', fontsize=FSS).color(C_VEL))
d += elm.Arrow().at(ffb.S).down(1.25 - v2N[1]).color(C_VEL)
ffN = ffb.absanchors['N']
ffW = ffb.absanchors['W']
# alpha* from TRAJ, over the top lane
d += elm.Line().at((tN[0], LANE1)).up(LANE2 - LANE1).color(C_TRA)
d += elm.Line().at((tN[0], LANE2)).right((ffN[0] - 1.0) - tN[0]).label('α*', loc='top', fontsize=FSS).color(C_TRA)
d += elm.Arrow().at((ffN[0] - 1.0, LANE2)).down(LANE2 - ffN[1]).color(C_TRA)
# omega_ref tapped off the position-loop output
tapx = clw.absanchors['E'][0] + 0.4
d += elm.Dot(radius=0.06).at((tapx, 0)).color(C_POS)
d += elm.Line().at((tapx, 0)).up(ffW[1]).color(C_POS)
d += elm.Arrow().at((tapx, ffW[1])).right(ffW[0] - tapx).color(C_POS)
# theta_m branch is wired after the theta-hat riser exists (see below)

# useful anchor columns
spS = sp.absanchors['S']
svS = sv.absanchors['S']
siS = si.absanchors['S']
s2S = si2.absanchors['S']
rS = rc.absanchors['S']
cS = cli.absanchors['S']
pS = plant.absanchors['S']

# sensor feeder from the plant
FEED = -3.0
FE_C = -5.0                  # front-end row centerline
FE_N = FE_C + 0.55
FE_S = FE_C - 0.55
JOG = -6.4                   # i-hat distribution lane between the rows
OB_C = -7.8                  # observer row centerline
shx = siS[0]                 # shunt FE sits under the current error sum
tvx = shx + 2.6
posx = svS[0] + 1.75         # clear of the omega-hat riser at svS
d += elm.Line().at(pS).down(pS[1] - FEED).label('raw signals', loc='bottom', ofst=(0.95, 0.5), fontsize=FSS).color(C_HW)
d += (posfe := dsp.Box(w=3.0, h=1.1).at((posx, FE_C)).anchor('center').label(
    'position FE\npot / mag / IR + LUT', fontsize=FSS).color(C_EST))
d += (shfe := dsp.Box(w=2.0, h=1.1).at((shx, FE_C)).anchor('center').label('shunt FE', fontsize=FSS).color(C_EST))
d += (tvfe := dsp.Box(w=2.6, h=1.1).at((tvx, FE_C)).anchor('center').label('terminal V FE\n(v̂_diff, V̂bus)', fontsize=FSS).color(C_EST))
d += elm.Line().at((pS[0], FEED)).left(pS[0] - posx).color(C_HW)
for xdrop in (shx - 0.75, tvx):
    d += elm.Dot(radius=0.06).at((xdrop, FEED)).color(C_HW)
    d += elm.Arrow().at((xdrop, FEED)).down(FEED - FE_N).color(C_HW)
d += elm.Arrow().at((posx, FEED)).down(FEED - FE_N).color(C_HW)

# observers row
d += (bemf := dsp.Box(w=4.2, h=1.2).at((shx + 0.2, OB_C)).anchor('center').label(
    'BEMF observer +\nwinding thermometry', fontsize=FSS).color(C_EST))
d += (kf := dsp.Box(w=4.2, h=1.2).at((svS[0], OB_C)).anchor('center').label(
    'Kalman fusion\n[θ̂, ω̂, τ̂d]', fontsize=FSS).color(C_EST))
bN = bemf.absanchors['N']
kN = kf.absanchors['N']

# i-hat: shunt FE up to the current sum, down to the observers, jog into KF
d += elm.Arrow().at((shx, FE_N)).up(siS[1] - FE_N).color(C_EST)
d += elm.Label(label='î', fontsize=FSS, color=C_EST).at((shx + 0.35, -1.95))
d += elm.Line().at((shx, FE_S)).down(FE_S - JOG).color(C_EST)
d += elm.Dot(radius=0.06).at((shx, JOG)).color(C_EST)
d += elm.Arrow().at((shx, JOG)).down(JOG - bN[1]).color(C_EST)
d += elm.Line().at((shx, JOG)).left(shx - (kN[0] - 0.9)).color(C_EST)
d += elm.Arrow().at((kN[0] - 0.9, JOG)).down(JOG - kN[1]).color(C_EST)
# v-diff: terminal FE down into bemf
VD = FE_S - 0.55
d += elm.Line().at((tvx, FE_S)).down(FE_S - VD).color(C_EST)
d += elm.Line().at((tvx, VD)).left(tvx - (shx + 0.85)).label('v̂_diff', loc='bottom', fontsize=FSS).color(C_EST)
d += elm.Arrow().at((shx + 0.85, VD)).down(VD - bN[1]).color(C_EST)
# theta_meas: position FE down into KF
d += elm.Arrow().at((posx, FE_S)).down(FE_S - kN[1]).color(C_EST)
d += elm.Label(label='θ_meas', fontsize=FSS, color=C_EST).at((posx + 0.9, FE_S - 0.4))
# omega_bemf: bemf into KF
d += elm.Arrow().at(bemf.W).left(bemf.absanchors['W'][0] - kf.absanchors['E'][0]).label('ω̂_bemf', loc='top', fontsize=FSS).color(C_EST)
# Twind up into the clamp
twx = bemf.absanchors['W'][0] + 0.5
d += elm.Line().at((twx, bN[1])).up(bN[1] * -1 - 1.85).color(C_EST)
d += elm.Line().at((twx, -1.85)).left(twx - cS[0]).color(C_EST)
d += elm.Arrow().at((cS[0], -1.85)).up(1.85 + cS[1]).color(C_EST)
d += elm.Label(label='T̂wind', fontsize=FSS, color=C_EST).at((cS[0] + 0.62, -1.2))
d += elm.Label(label='i_lim = min(torque_limit,\nderate, stall, endstop(θ̂))', fontsize=FSS, color=C_LIM).at((cS[0] - 3.0, -2.35))
# theta-hat: KF left and up into the position sum
kW = kf.absanchors['W']
d += elm.Line().at(kW).left(kW[0] - spS[0]).color(C_EST)
d += elm.Arrow().at((spS[0], kW[1])).up(spS[1] - kW[1]).color(C_EST)
d += elm.Label(label='θ̂', fontsize=FSS, color=C_EST).at((spS[0] + 0.3, -1.95))
# theta_m into the mech FF - branch of theta-hat, routed around the outside
LANE3 = 3.45
d += elm.Dot(radius=0.06).at((spS[0], -0.75)).color(C_EST)
d += elm.Line().at((spS[0], -0.75)).left(spS[0] + 0.5).color(C_EST)
d += elm.Line().at((-0.5, -0.75)).up(LANE3 + 0.75).color(C_EST)
d += elm.Line().at((-0.5, LANE3)).right(ffN[0] + 1.0 + 0.5).color(C_EST)
d += elm.Arrow().at((ffN[0] + 1.0, LANE3)).down(LANE3 - ffN[1]).color(C_EST)
d += elm.Label(label='θ̂_m', fontsize=FSS, color=C_EST).at((ffN[0] + 1.6, ffN[1] + 0.5))
# omega-hat: KF up into the velocity sum, branch right for BEMF decoupling
KLANE = -1.5
d += elm.Line().at((svS[0], kN[1])).up(KLANE - kN[1]).color(C_EST)
d += elm.Dot(radius=0.06).at((svS[0], KLANE)).color(C_EST)
d += elm.Arrow().at((svS[0], KLANE)).up(svS[1] - KLANE).color(C_EST)
d += elm.Label(label='ω̂', fontsize=FSS, color=C_EST).at((svS[0] + 0.3, -1.0))
d += elm.Line().at((svS[0], KLANE)).right(s2S[0] - svS[0]).color(C_EST)
d += elm.Arrow().at((s2S[0], KLANE)).up(s2S[1] - KLANE).color(C_EST)
d += elm.Label(label='Ke·ω̂', fontsize=FSS, color=C_EST).at((s2S[0] + 0.55, -1.0))
# vbus: second output of the terminal FE, out the east side and up into
# the reciprocal block (keeps the riser clear of the Ke-omega label)
tvE = tvfe.absanchors['E']
d += elm.Line().at(tvE).right(rS[0] - tvE[0]).color(C_EST)
d += elm.Arrow().at((rS[0], tvE[1])).up(rS[1] - tvE[1]).color(C_EST)
d += elm.Label(label='V̂bus', fontsize=FSS, color=C_EST).at((rS[0] + 0.55, -1.0))
# disturbance torque out
d += elm.Arrow(ls='--').at(kf.S).down(0.7).label('τ̂d (load estimate)', loc='bottom', fontsize=FSS).color(C_EST)
save(d, 'everything.svg')

print('done')
