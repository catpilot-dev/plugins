"""BMW E9x/E8x lateral controller — Direct Tracking of the Arc reference.

Hook target for controls.lat_controller_init (plugin.json): patches lac.update
with this controller when the lane_keeping plugin (the Arc reference) is
loaded; otherwise the stock lateral controller stays in place. Reference doc:
LATERAL_CONTROLLER.md (current law at the top, dated history below).

The law, per deviceMotion tick (20 Hz) and per CAN tick (100 Hz):
  delta_err = atan(kappa_ref*L) - atan(kappa_meas*L)        front-wheel angle (rad)
  P_nm      = T_CAP_SLOPE_BASE * kappa_scale * v^2 * delta_err
  a_ref     = v_true^2 * kappa_ref                          reference lateral accel
  ff_nm     = curve_ff_nm(a_ref)                            steady curve torque
  demand    = ff_nm + P_nm (+ sl_bias)
Every CONTROL_HORIZON_S/2 a decision takes the demand through t_cap, relax-dwell
(a deep measured curve holds its torque through a reference dip), the noise
guard and STEP_MAX, and the torque ramps to it over the next half horizon; the
free-play take-up closes the servo lash on each change of push direction.

Lateral acceleration is bounded by speedlimitd through v, never by draining
torque here: this controller tracks the commanded curvature and never gives
up in a turn (routes 326, 385, 2ba, 3ce, 3cf: every mid-turn drain ran the car
wide). The P law reversing on overshoot is the correction.

Loaded by the plugin registry under its canonical name (manifest id + module
path). Never import it as `bmw.latcontroller` from other modules: a second
module instance would carry its own state dict.
"""
import os
import sys
from collections import deque

# The plugin root must be importable for `from bmw.values import ...` whatever
# the load order (this file lives in bmw/).
_PLUGIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PLUGIN_DIR not in sys.path:
  sys.path.insert(0, _PLUGIN_DIR)

import numpy as np

# Gain reference speed floor (m/s). The torque law references max(v_true, floor)
# so the v^2 gain stops collapsing at low speed; 10 km/h leaves walking-pace
# commands below the ~2.75 Nm breakaway (routes 411/413 seg 2, user ruling).
V_GAIN_FLOOR = 2.778

# LKA regime: DCC cannot engage below 30 km/h, so v_true < LKA_MAX_V is lateral-
# only driving. Its two behaviours (relax-dwell length, unwind step boost)
# blend in over [LKA_MAX_V, LKA_BLEND_V] because the plant does not step at
# 30 km/h
# (route 41b: a hard threshold switched the law mid-turn).
LKA_MAX_V = 8.5            # m/s (30 km/h) -- lower edge of the blend band
LKA_BLEND_V = 12.0         # m/s (43 km/h) -- upper edge of the blend band (user choice)
UNWIND_STEP_GAIN = 3.0     # step-cap multiplier while shedding torque in LKA: stiction
                           # holds the freed wheel there, so a fast release is safe (route 417)


def gain_reference_speed(v_true):
  """Speed the torque parameters (gains and caps) reference; identity above V_GAIN_FLOOR."""
  return max(float(v_true), V_GAIN_FLOOR)


def curve_ff_nm(a_ref):
  """Steady curve torque (Nm) for a reference lateral accel a_ref (m/s^2, + = right)."""
  mag = max(abs(float(a_ref)) - CF_AY0, 0.0)
  return float(np.clip(CF_GAIN * np.sign(a_ref) * mag, -CF_MAX_NM, CF_MAX_NM))

# Controller timing horizon (s): the decision cadence and the ramp spread are
# each half of it. Fixed, independent of lat_delay: a 0.95 s horizon stretched
# the cadence to ~500 ms of added lag (route 4d8).
CONTROL_HORIZON_S = 0.65

# Slope bias (SlopeCompensation switch; docs/superpowers/specs/2026-10-19-bmw-
# slope-bias-design.md). P alone needs a standing error to hold the steady
# torque a straight road asks for (route 4e9: ~0.08 Nm, the car +0.15 m right
# of the plan), so sl_bias integrates the P term on straights and is added to
# the P target everywhere. The learn gate is the driving situation only, never
# the error's sign: anti-windup must never block the unwind (37e34b7).
SL_TAU_S = 20.0            # integration time constant on the P term (s)
SL_BIAS_MAX_NM = 0.25      # route 4ea: roads with roll >= 2.3 deg need 0.20-0.25; well below breakaway
SL_MIN_V = 40 / 3.6        # learn only above 40 km/h (m/s)
SL_KAPPA_DES = 1e-3        # straight: |kappa_ref| below this (1/m)
SL_KAPPA_MEAS = 1.5e-3     # and |kappa_meas| below this (1/m)
SL_AY_MAX = 0.15           # and |a_ref| below this (m/s^2): gentle bends charged the bias on route 4ec

# Curve integrator (CurveIntegrator switch, default off; LATERAL_CONTROLLER.md
# "Curve integrator"). In a steady curve P only supplies the torque the
# feed-forward leaves out by holding a standing error, and rack friction grows
# with load (fitted breakaway ~0.33 + 0.67*|a_y| Nm), so the car settles wide:
# understeer +0.1..+0.7e-3 below 90 km/h on every drive, right bends worse than
# left (route 515: +0.73 vs -0.03e-3). ci_nm integrates delta_err in curves and
# is added to the demand. It always unwinds (no freeze: the July hold-bias
# froze on overshoot and ratcheted to its cap, 37e34b7), leaks away outside
# curves, resets on an S-curve sign flip and on disengage, and pauses in lane
# changes. Its gate is the complement of the slope-bias learn gate.
CI_AY_ON = SL_AY_MAX       # integrate only at |a_ref| >= this (m/s^2)
CI_MIN_V = 40 / 3.6        # and above 40 km/h (m/s)
CI_KI = 75.0               # Nm per (rad of delta_err * s): 0.3e-3 under -> ~0.3 Nm in ~4 s. Half the
                           # first cut: the (unvalidated) bench traded understeer for 0.1-0.5 Hz weave
                           # about linearly in CI_KI (150: -0.15e-3 / +70 %, 75: -0.09e-3 / +35 %)
CI_MAX_NM = 0.5            # cap (Nm), about one breakaway band
CI_TAU_LEAK_S = 1.5        # decay time constant outside the gate (s)

# Curve feed-forward (docs/superpowers/specs/2026-10-20-bmw-curve-feedforward-
# and-audit-design.md). Above ~0.3 m/s^2 a curve needs steady torque this
# hydraulic rack's stiction does not supply (440 steady windows on 4e9-4ec:
# ~1.5 Nm per m/s^2 beyond ~0.25); P alone paid for it with a standing error
# (4ec seg 22: the car drifted 0.8 m to the outside of a 1 m/s^2 bend). The
# feed-forward stays below the measured need on purpose: P closes the rest
# and rarely has to remove feed-forward torque (replay 4e9-4ec: 14 % of
# steady windows, by at most 0.8 Nm). Raised 2026-11-01 (route 50f, lash
# 21 -> 3.4 deg after the clamped column gear): the measured steady need is
# 1.43 Nm per m/s^2 - 0.16 (4f7: 1.38 - 0.22); the old 1.0 / 0.3 supplied ~45 %
# at 0.6 m/s^2, so mild curves and lane changes waited on P's standing error
# (car at ~0.7 s lag, 0.6-0.9 m outward on mild curves). 1.4 / 0.2 gives ~80 %
# and stays under the need at every a_ref.
CF_AY0 = 0.2               # m/s^2 deadband: below it stiction holds the angle at zero torque; above SL_AY_MAX
CF_GAIN = 1.4              # Nm per m/s^2 beyond the deadband (measured need ~1.4)
CF_MAX_NM = 2.0            # cap (Nm); t_cap and STEER_MAX still apply

# Direct Tracking (docs/superpowers/specs/2026-10-18-bmw-direct-tracking-
# design.md) is the only law: no rest state, sign-guarded reversals, free-play
# take-up. It tracks the Arc reference from lane_keeping; raw kappa_des on a
# deadzone-free tracker is the documented-unsafe case, so without that plugin
# the hook leaves the stock controller in place (ruling R10). The registry
# inserts a module into sys.modules BEFORE executing it and leaves it there
# when the import raises, so presence alone is not enough: HOOK_READY is the
# module's last statement, set only when it loaded fully.
_LK_REGISTER_MODULE = 'plugins.lane_keeping.register'


def _lane_keeping_ready():
  return getattr(sys.modules.get(_LK_REGISTER_MODULE), 'HOOK_READY', False) is True


def on_lat_controller_init(result, lac, CP):
  """Install the Direct Tracking controller as lac.update.

  Front-wheel-angle error (rear-axle bicycle model, L = CP.wheelbase):
    delta_err = atan(kappa_ref*L) - atan(kappa_meas*L),  kappa_meas = yaw rate / v_true

  Demand, every deviceMotion tick, on the full error:
    P_nm   = T_CAP_SLOPE_BASE * kappa_scale(|kappa_ref|) * v^2 * delta_err
    ff_nm  = curve_ff_nm(v_true^2 * kappa_ref)
    demand = ff_nm + P_nm (+ sl_bias)
  Decision, every CONTROL_HORIZON_S/2 while active:
    target = demand, clipped to t_cap, bridged by relax-dwell in a deep
             measured curve, sign changes gated by the noise guard, moved at
             most STEP_MAX from the current torque.
  The torque ramps to the target over the next half horizon, one CAN tick at
  a time; the free-play take-up (bmw/takeup.py) owns the torque while it
  crosses the servo lash on a change of push direction. Sub-breakaway targets
  are commanded as they are: on this hydraulic rack a torque too small to
  move the column is, in effect, a hold.

  Inactive: no decision runs and torque/target/ramp are zero, so each
  engagement ramps from 0 on its first deviceMotion tick.
  """
  import math
  from openpilot.cereal import log
  from openpilot.cereal import messaging
  from bmw.values import CarControllerParams as CCP
  from bmw.takeup import SignGuard, ContactLearner, FreePlayTakeUp

  # Ruling R10: Direct needs the Arc reference. Without lane_keeping the stock
  # lateral controller stays in place.
  if not _lane_keeping_ready():
    try:
      from openpilot.common.swaglog import cloudlog
      cloudlog.warning("bmw latcontroller: lane_keeping not loaded, stock lateral controller in use")
    except Exception:
      pass
    return result

  # Plugin params are never read per tick: a file read on controlsd's 100 Hz
  # thread can cost a control frame under eMMC contention. The import is at
  # function scope so a missing config.py (install.sh copies it in a later
  # step) defaults the switch instead of failing the hook module.
  try:
    from config import read_plugin_param
  except Exception:
    pass
  # SlopeCompensation is read at each engagement edge in update() (a rare
  # event, not a per-tick read), so the user can A/B by re-engaging.
  # Default off; a failed read (incl. read_plugin_param unbound) is off.

  # Decision cadence (deviceMotion ticks) and CAN-rate ramp spread: both are half
  # of CONTROL_HORIZON_S, so two decision-and-ramp cycles fit one horizon and
  # each ramp completes as the next decision lands. Panda enforces wire-rate.
  DT_DEVICE_MOTION = 0.05                        # one deviceMotion tick (s)
  DT_CAN_TICK = 0.01                        # one CAN tick at 100 Hz (s)
  ACTION_CADENCE_TICKS_FALLBACK = 5         # state-init only: primes the first decision
  # Plant-inversion gain, linear tire regime: tau_Nm = T_CAP_SLOPE * v^2 * delta
  # drives both the target and the authority cap
  #   t_cap = min(STEER_MAX, T_CAP_BASE_NM + T_CAP_SLOPE_BASE*kappa_scale*v^2*|delta_ref|).
  # kappa_scale raises the gain from 1.5 on straights and gentle bends to 3.0 at
  # |kappa_ref| >= 0.02, where the planner must be chased. Near centre the rack
  # breaks free at ~0.45 Nm, so the curvature error needed to move it scales
  # with 1/gain (route 4ed: 1.0 -> 1.25). 1.25 -> 1.5 (2026-11-09, routes
  # 530/531): mild bends still ran ~0.4e-3 wide; the closed-loop bench cuts that
  # 0.05e-3 with no added 0.1-0.3 Hz straight sway, where a feed-forward or
  # integral term on the reference adds sway. 1.5 rang near-straight on route
  # 326 under the pre-Direct controller: on-car check = straight sway and
  # torque reversals.
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01, 0.02]        # |kappa_ref| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.5, 2.5, 3.0]           # scale factor on T_CAP_SLOPE_BASE

  # Per-decision torque step cap (frac): human-style gradual steering, 1.2 Nm
  # per decision up to 54 km/h (~4 Nm/s) tapering to 0.6 Nm at 100 km/h, where
  # lane margin is consumed faster (routes 385 seg 27, 39b seg 18).
  STEP_MAX_V  = [15.0, 28.0]     # vEgo breakpoints (m/s)
  STEP_MAX_BP = [0.10, 0.05]     # per-decision torque step cap (frac)
  # During a lane change the cap is the flat low-speed value at every speed:
  # the manoeuvre is planned and the model commits to it, and on the taper the
  # torque trailed the demand (routes 50f/510 at 77-87 km/h: 2.5-2.7 Nm built
  # against 4.3-5.6 Nm, the car ran ~0.2 s behind the plan and the model added
  # correcting lobes = the wheel swing). Never above STEP_MAX_BP[0].
  LANE_CHANGE_STATES = ('laneChangeStarting', 'laneChangeFinishing')
  # Yaw predictor, lane changes only: the P law's measured curvature adds the
  # curvature the wheel has put in over the last YAW_PRED_TICKS deviceMotion ticks
  # (0.3 s = the wheel -> yaw lag, routes 4f7-511), which the yaw has not shown
  # yet. The rack holds the wheel where it stops (route 511 seg 6: 2 -> 12 deg
  # under <= 1.6 Nm, held at 0 Nm), so the yaw-only P reversed ~0.3 s after the
  # overshoot was already in. Only the difference enters: a standing wheel
  # angle (alignment offset, a curve) adds nothing, and the yaw stays the
  # steady-state reference. YAW_PRED_SR: wheel deg per front-wheel deg incl.
  # understeer, fitted on routes 510/511 (24.8-25.4).
  YAW_PRED_TICKS = 6
  YAW_PRED_SR = 25.0

  # Relax-dwell: in a measured deep curve an overshoot-side error must persist
  # this long before the decision may command below the current torque. It
  # bridges the model's reference dips mid-hairpin (route 3a0 seg 8; on
  # 4e9-4ec nine dips of 9-21 % for 0.15-0.55 s, holding 1.9-3.8 Nm). The
  # feed-forward cannot bridge them: it sits at its cap in a hairpin.
  RELAX_DWELL_TICKS = 20      # deviceMotion ticks (1.0 s at 20 Hz)
  RELAX_DWELL_KAPPA = 0.010   # |kappa_meas| defining a deep curve (1/m)

  # Rear-axle bicycle-model wheelbase (m), for kappa <-> delta conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['deviceMotion', 'drivingModelData'])
  _learner = ContactLearner()

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each decision (cadence = CONTROL_HORIZON_S/2)
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / spread_frames
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS_FALLBACK,  # primed so first deviceMotion tick fires cadence immediately (no engagement gap)
    'action': 'init',          # debug: ramp / relax_dwell / idle / take_up
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad), what controller acts on
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'relax_ticks': 0,             # consecutive deviceMotion ticks of overshoot-side error while deep in a curve
    # Direct Tracking.
    'was_active': False,     # previous tick's `active`, for the engagement edge
    'guard': SignGuard(),    # noise guard on P-target sign reversals
    'learner': _learner,     # servo-column contacts, learned every drive
    'takeup': FreePlayTakeUp(_learner, CCP.STEER_MAX),   # lash take-up
    'tu_resume': 'init',     # decision label under 'take_up', restored when it ends
    'tu_demand': 0.0,        # last decision's target before STEP_MAX (take-up size gate)
    'a_ref': 0.0,            # reference lateral accel v_true^2 * kappa_ref (m/s^2)
    'ff_nm': 0.0,            # curve feed-forward (Nm)
    'demand_nm': 0.0,        # ff_nm + P_nm (+ sl_bias): what the decision targets
    'slope': False,          # SlopeCompensation latched at the engagement edge
    'sl_bias': 0.0,          # learned straight-road torque (Nm); kept across engagements, never persisted
    'sl_learn': False,       # learn gate open this deviceMotion tick (telemetry)
    'curve_int': False,      # CurveIntegrator latched at the engagement edge
    'ci_nm': 0.0,            # curve integrator torque (Nm), added to the demand
    'lane_change': False,    # model runs a lane change this deviceMotion tick (LANE_CHANGE_STATES)
    'wheel_hist': deque(maxlen=YAW_PRED_TICKS + 1),  # wheel angle (deg) at the last deviceMotion ticks
    'kappa_pred': 0.0,       # yaw-predictor curvature added to the P law's measured (1/m)
  }

  def update(active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = 11

    # Engagement edge: latch the SlopeCompensation switch (read here only, so
    # the law never changes while pushing); the guard starts uncommitted.
    if active and not state['was_active']:
      try:
        state['slope'] = read_plugin_param('bmw_e9x_e8x', 'SlopeCompensation', '') == '1'
      except Exception:
        state['slope'] = False
      try:
        state['curve_int'] = read_plugin_param('bmw_e9x_e8x', 'CurveIntegrator', '') == '1'
      except Exception:
        state['curve_int'] = False
      state['ci_nm'] = 0.0
      state['guard'].reset()
    elif not active:
      state['slope'] = False
      state['curve_int'] = False
      state['ci_nm'] = 0.0
      # Inactive: the law does not run on the state. Torque, target and ramp
      # are zeroed so the next engagement ramps from 0 under STEP_MAX on its
      # first deviceMotion tick instead of jumping to a torque built while the
      # output was 0 (the rack never received it). A take-up in flight ends.
      state.update(torque=0.0, target_frac=0.0, ramp_step=0.0, ramp_frames=0)
      state['takeup'].reset()
      if state['action'] == 'take_up':
        state['action'] = state['tu_resume']
    state['was_active'] = active

    # steeringAngleDeg carries a constant alignment offset (~-1.58 deg): the
    # take-up reads angle deltas only, so it never needs to know it.
    # getattr: CS is a stub in some test paths.
    _angle = float(getattr(CS, 'steeringAngleDeg', 0.0))

    _sm.update(0)
    lp = _sm['deviceMotion']
    motion_updated = _sm.updated['deviceMotion']

    # deviceMotion tick (20 Hz): measured and desired every tick, a decision every
    # action_cadence_ticks (6 ticks, 300 ms: >= 2.5 tau of the rack's ~130 ms response,
    # so the plant has answered the previous step). CAN tick (100 Hz): ramp.
    if motion_updated:
      # Cadence and ramp length, both half of CONTROL_HORIZON_S; spread_frames
      # is derived from the cadence (5 CAN ticks per deviceMotion tick) so the ramp
      # ends exactly when the next decision lands (6 ticks = 30 frames = 300 ms).
      half_horizon = 0.5 * CONTROL_HORIZON_S
      action_cadence_ticks = max(1, round(half_horizon / DT_DEVICE_MOTION))
      spread_frames = action_cadence_ticks * int(DT_DEVICE_MOTION / DT_CAN_TICK)

      # No filtering here: desired_curvature is already the lane_keeping Arc
      # reference, low-passed upstream. This controller tracks it faithfully.
      state['desired'] = float(desired_curvature)

      # Two speeds (user ruling): v_true divides the yaw rate — kappa_meas must
      # use the actual speed or the loop settles at the wrong curvature at low
      # speed; v is the gain reference (P, t_cap, step_max),
      # floored at V_GAIN_FLOOR so v^2 never collapses toward zero.
      v_true = max(float(lp.velocityDevice.x) if _sm.seen['deviceMotion'] else CS.vEgo, 1.0)
      v = gain_reference_speed(v_true)
      state['measured'] = float(lp.angularVelocityDevice.z) / v_true

      # Lane change (model state) gates the step cap and the yaw predictor.
      state['lane_change'] = bool(
        _sm.seen['drivingModelData']
        and str(_sm['drivingModelData'].meta.laneChangeState) in LANE_CHANGE_STATES)
      _hist = state['wheel_hist']
      _hist.append(_angle)
      state['kappa_pred'] = 0.0
      if state['lane_change'] and len(_hist) == _hist.maxlen:
        # angle + = left, kappa + = right.
        state['kappa_pred'] = -math.radians(_hist[-1] - _hist[0]) / (YAW_PRED_SR * L)

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan((state['measured'] + state['kappa_pred']) * L)
      delta_err_raw = delta_des - delta_meas

      delta_err = delta_err_raw
      state['delta_err'] = delta_err

      # Curvature scale for the P term and caps, keyed on |kappa_ref|.
      kappa_scale = float(np.interp(abs(state['desired']),
                                    T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP))

      # Curve feed-forward on the reference lateral accel, from the TRUE speed
      # (the gain reference floor would invent 0.39 m/s^2 at walking pace).
      a_ref = v_true * v_true * state['desired']
      state['a_ref'] = a_ref
      state['ff_nm'] = curve_ff_nm(a_ref)
      _p_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err

      # Slope bias learning (see SL_TAU_S): integrate the P term alone on
      # straights. The gate never looks at the error's sign.
      state['sl_learn'] = bool(
        state['slope'] and active and not getattr(CS, 'steeringPressed', False)
        and v_true > SL_MIN_V
        and abs(state['desired']) < SL_KAPPA_DES and abs(state['measured']) < SL_KAPPA_MEAS
        and abs(a_ref) < SL_AY_MAX
        and state['takeup'].state != 'taking_up')
      if state['sl_learn']:
        state['sl_bias'] = float(np.clip(state['sl_bias'] + (_p_nm / SL_TAU_S) * DT_DEVICE_MOTION,
                                         -SL_BIAS_MAX_NM, SL_BIAS_MAX_NM))

      # Curve integrator: integrate delta_err in a curve, leak it away outside.
      # A reference that changes sides (S-curve) drops what the last bend
      # needed at once rather than integrating it down through the next one.
      if state['curve_int'] and active:
        if state['ci_nm'] * state['desired'] < 0.0:
          state['ci_nm'] = 0.0
        if (abs(a_ref) >= CI_AY_ON and v_true >= CI_MIN_V and not state['lane_change']
            and not getattr(CS, 'steeringPressed', False)):
          state['ci_nm'] = float(np.clip(state['ci_nm'] + CI_KI * delta_err * DT_DEVICE_MOTION,
                                         -CI_MAX_NM, CI_MAX_NM))
        else:
          state['ci_nm'] *= math.exp(-DT_DEVICE_MOTION / CI_TAU_LEAK_S)

      # The demand the decision targets.
      state['demand_nm'] = (state['ff_nm'] + _p_nm + (state['sl_bias'] if state['slope'] else 0.0)
                            + state['ci_nm'])

      # The noise guard judges reversal persistence on the 20 Hz sign of the
      # whole demand, as an error-equivalent (Direct spec §3): on delta_err
      # alone a steady small error against the feed-forward or the bias would
      # hold the target at 0 and drop them.
      state['guard'].observe(state['demand_nm'] / (T_CAP_SLOPE_BASE * kappa_scale * v * v))

      # Relax-dwell: the model's reference dips mid-hairpin while the measured
      # curvature holds (route 3a0 seg 8: 40-50 % for ~1 s; on the Arc
      # reference, 4e9-4ec: 9-21 % for 0.15-0.55 s). Following the dip sheds
      # the curve torque — the feed-forward is at its cap in a hairpin and P
      # drops by kappa_scale·v²·L·Δκ — and SAT flings the wheel out of the
      # turn; the rebuild under STEP_MAX takes 1-2 s. So while deep in a
      # MEASURED curve, an overshoot-side error must persist RELAX_DWELL_TICKS
      # before the decision may command below the current torque. A sign flip
      # of kappa_ref (S-curve) disarms it at once. Below LKA_MAX_V stiction
      # holds a surrendered wheel, so the dwell is off there and its length
      # grows over the blend band (route 417: a full-length dwell at an
      # intersection exit fought the driver's return of the wheel).
      deep_relax = (v_true >= LKA_MAX_V
                    and abs(state['measured']) > RELAX_DWELL_KAPPA
                    and state['desired'] * state['measured'] > 0.0
                    and delta_err * delta_des < 0.0)
      state['relax_ticks'] = state['relax_ticks'] + 1 if deep_relax else 0
      dwell_ticks = float(np.interp(v_true, [LKA_MAX_V, LKA_BLEND_V],
                                    [0.0, float(RELAX_DWELL_TICKS)]))
      dwelling = 0 < state['relax_ticks'] <= dwell_ticks

      state['tick_count'] += 1
      # Expire transient action labels once their ramp completes, so telemetry
      # occupancy is not over-attributed; holds keep their label.
      if state['ramp_frames'] == 0 and state['action'] in ('ramp', 'relax_dwell'):
        state['action'] = 'idle'

      if active and state['tick_count'] >= action_cadence_ticks:
        state['tick_count'] = 0

        # The demand of this deviceMotion tick, on the FULL error — no on-target
        # band, a sub-friction target is in effect a hold (Direct spec §2).
        _tu_demand = None   # the decision's demand before the step cap (take-up size gate)
        target_frac = state['demand_nm'] / CCP.STEER_MAX
        state['action'] = 'ramp'
        # Authority cap, v^2*|delta_ref|-scaled and clipped at STEER_MAX:
        # straights stay near T_CAP_BASE_NM, tight turns can reach the panda
        # limit (transient over-envelope; speedlimitd bleeds v).
        t_cap_nm = min(CCP.STEER_MAX,
                       T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
        t_cap_frac = t_cap_nm / CCP.STEER_MAX
        target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))
        # Relax-dwell bridge: during the dwell an overshoot-side reference dip
        # may not command below the current torque (within t_cap).
        if dwelling:
          target_frac = math.copysign(min(abs(state['torque']), t_cap_frac), state['torque'])
          state['action'] = 'relax_dwell'
        # Noise guard (Direct spec §3): a sign change of the target commands 0
        # until the opposite error sign has persisted GUARD_PERSIST_S — ~0.5 Nm
        # of noise torque at 25 m/s is enough to sweep the servo across the
        # lash, and 0 is itself a hold. Exempt while relax-dwell bridges
        # (ruling R3): its target keeps the current torque's sign.
        if not dwelling:
          target_frac = state['guard'].allow(target_frac)
        # The take-up sizes the reversal on this demand, not on the step-capped
        # target: a zero crossing under STEP_MAX can land a few hundredths
        # past zero and the next decision is already past it.
        _tu_demand = target_frac
        # Per-decision step cap: move at most STEP_MAX toward the target, let
        # the plant respond one cadence, re-measure and step again (route 385
        # seg 27: single 8.3 Nm decisions drove 150 deg/s wheel bursts).
        step_max = float(np.interp(v, STEP_MAX_V, STEP_MAX_BP))
        if state['lane_change']:
          step_max = STEP_MAX_BP[0]
        # Fast unwind in the LKA regime, only while the step moves torque
        # toward zero: releasing carries no abrupt-application hazard below
        # 30 km/h (SAT cannot fling the freed wheel, stiction holds it) and a
        # ~3.9 Nm hold otherwise takes ~1 s to shed against the driver's return
        # of the wheel (route 417). Not gated on curvature: kappa_meas collapses
        # through RELAX_DWELL_KAPPA partway through an exit.
        if (target_frac - state['torque']) * state['torque'] < 0.0:
          step_max *= float(np.interp(v_true, [LKA_MAX_V, LKA_BLEND_V],
                                      [UNWIND_STEP_GAIN, 1.0]))
        step = float(np.clip(target_frac - state['torque'], -step_max, step_max))
        target_frac = float(np.clip(state['torque'] + step, -t_cap_frac, t_cap_frac))

        state['target_frac'] = target_frac
        state['tu_demand'] = target_frac if _tu_demand is None else _tu_demand
        state['ramp_step'] = (target_frac - state['torque']) / spread_frames
        state['ramp_frames'] = spread_frames

    # Contact learning (100 Hz, observe only): the servo-column gap under the
    # torque commanded last tick (state['torque'] has not advanced yet).
    # Guarded (ruling R12): an exception here must never break lateral.
    _gap = float(getattr(CS, 'steeringAngleOffsetDeg', 0.0))
    try:
      state['learner'].observe(_gap, state['torque'] * CCP.STEER_MAX,
                               float(getattr(CS, 'steeringRateDeg', 0.0)),
                               ok=active and not getattr(CS, 'steeringPressed', False))
    except Exception:
      pass

    # Free-play take-up (Direct spec §3), per CAN tick. Old-side torque has
    # been shed under STEP_MAX; when the ramp crosses zero toward an unseated
    # flank, a TAKEUP_NM step closes the lash (the only un-capped jump), then
    # the remaining ramp re-aims at target_frac from that level.
    _ramp_held = False
    tu = state['takeup']
    _pressed = bool(getattr(CS, 'steeringPressed', False))
    # Driver override: end the take-up and resume the held ramp (as on
    # handover) — driver-driven SZL motion must not confirm a contact and
    # hand over against the driver. No new take-up fires while pressed.
    if tu.state == 'taking_up' and _pressed:
      tu.reset()
      if state['ramp_frames'] > 0:
        state['ramp_step'] = (state['target_frac'] - state['torque']) / state['ramp_frames']
    _was_taking = tu.state == 'taking_up'
    if state['action'] != 'take_up':
      state['tu_resume'] = state['action']
    # Only a ramp headed to a non-zero target on its own side is "on the way
    # to the new side": a shed to 0 can land a float epsilon past zero.
    _heading = state['ramp_step'] if (state['ramp_frames'] > 0
                                      and state['target_frac'] * state['ramp_step'] > 0.0) else 0.0
    # Guarded like the learner: a take-up fault drops back to the plain ramp,
    # never out of lateral.
    try:
      _tu_frac = tu.step(state['torque'], _heading, state['target_frac'], _gap, _angle,
                         blocked=_pressed,
                         demand_frac=state['tu_demand'])
    except Exception:
      tu.reset()
      _tu_frac = None
    if _tu_frac is not None:
      state['torque'] = _tu_frac
      state['action'] = 'take_up'
      _ramp_held = True
    else:
      if state['action'] == 'take_up':
        state['action'] = state['tu_resume']
      if _was_taking and state['ramp_frames'] > 0:
        # Handover on the take-up side with a step-capped target below the
        # take-up level but a demand at/above it: hold the level rather than
        # ramp down off the flank just closed; the next decision steps up
        # from it. A reversed or released target still re-aims normally.
        _t = state['torque']
        if (state['target_frac'] * _t > 0.0 and abs(state['target_frac']) < abs(_t)
            and state['tu_demand'] * _t > 0.0 and abs(state['tu_demand']) >= abs(_t)):
          state['target_frac'] = _t
        state['ramp_step'] = (state['target_frac'] - state['torque']) / state['ramp_frames']

    # Apply the per-frame ramp step; panda enforces wire-rate (STEER_DELTA_UP)
    # downstream. Held while a take-up commands the torque.
    if state['ramp_frames'] > 0 and not _ramp_held:
      state['torque'] = float(np.clip(state['torque'] + state['ramp_step'], -1.0, 1.0))
      state['ramp_frames'] -= 1
      # A ramp that completes on a 0 target ends at exactly 0.0: (0 - t)/n
      # applied n times lands at +/-eps about half the time, and +eps on the
      # new side makes the later genuine reversal fail take-up's torque*s <= 0
      # test, so the take-up is silently lost.
      if state['ramp_frames'] == 0 and state['target_frac'] == 0.0:
        state['torque'] = 0.0

    err = state['desired'] - state['measured']  # for logging only
    output = 0.0 if not active else float(np.clip(state['torque'], -1.0, 1.0))

    pid_log.actualLateralAccel = float(state['measured'])
    pid_log.desiredLateralAccel = float(state['desired'])
    pid_log.error = float(err)
    pid_log.active = active
    pid_log.output = float(output)
    pid_log.saturated = bool(abs(output) > 0.99)

    # Telemetry at deviceMotion rate (20 Hz): most fields only change on deviceMotion
    # ticks. The exact field set is pinned by test_payload_exact_key_set and
    # listed in LATERAL_CONTROLLER.md §9.
    if motion_updated:
      try:
        if state['lat_pub'] is None:
          from openpilot.selfdrive.plugins.plugin_bus import PluginPub
          state['lat_pub'] = PluginPub('bmw_lat_control')
        _contacts = state['learner'].contacts()
        payload = {
          'desired': float(state['desired']),
          'measured': float(state['measured']),
          'err': float(err),
          'delta_err': float(state['delta_err']),             # what the P law acts on
          'target_frac': float(state['target_frac']),
          'ramp_step': float(state['ramp_step']),
          'ramp_frames': int(state['ramp_frames']),
          'action': state['action'],
          'torque': float(state['torque']),
          'output': float(output),
          'vEgo': float(CS.vEgo),
          'active': active,
          'ff_nm': float(state['ff_nm']),                       # curve feed-forward (Nm)
          'a_ref': float(state['a_ref']),                       # reference lateral accel (m/s^2)
          'relax_ticks': int(state['relax_ticks']),
          # Steering-side signals for drive analysis.
          'angle': float(getattr(CS, 'steeringAngleDeg', 0.0)),            # wheel angle, deg, + = LEFT (BMW)
          'servo_gap': float(getattr(CS, 'steeringAngleOffsetDeg', 0.0)),  # column - servo angle, deg: slack/windup
          'steer_rate': float(getattr(CS, 'steeringRateDeg', 0.0)),
          'pressed': bool(getattr(CS, 'steeringPressed', False)),
          'lat_delay': float(lat_delay) if lat_delay else 0.0,             # s, as given by controlsd
          # Take-up (Direct spec §5). Contacts 0.0 = not known yet this drive.
          'tu_state': state['takeup'].state,                  # right / left (seated) / taking_up / unknown
          'c_right': float(_contacts[0]) if _contacts else 0.0,   # gap under right torque, deg
          'c_left': float(_contacts[1]) if _contacts else 0.0,    # gap under left torque, deg
          'tu_fires': int(state['takeup'].fires),             # cumulative take-ups
          'tu_ms': float(state['takeup'].last_ms),            # last take-up duration
          'tu_timeouts': int(state['takeup'].timeouts),       # cumulative take-ups ended by TAKEUP_TIMEOUT_S
          'tu_guarded': int(state['guard'].suppressed),       # cumulative reversals held at 0 by the noise guard
          'sl_bias': float(state['sl_bias']),                  # learned straight-road torque (Nm)
          'sl_learn': bool(state['sl_learn']),                 # slope-bias learn gate open
          'kappa_pred': float(state['kappa_pred']),
          'ci_nm': float(state['ci_nm']),                      # curve integrator torque (Nm, 0 when off)            # yaw predictor, added to measured in the P law (1/m)
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result
