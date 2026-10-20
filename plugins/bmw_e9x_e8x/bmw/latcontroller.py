"""BMW E9x/E8x lateral controller — Direct Tracking of the Arc reference.

Hook target for controls.lat_controller_init (plugin.json): patches lac.update
with this controller when the lane_keeping plugin (the Arc reference) is
loaded; otherwise the stock lateral controller stays in place. Reference doc:
LATERAL_CONTROLLER.md (current law at the top, dated history below).

The law, per livePose tick (20 Hz) and per CAN tick (100 Hz):
  delta_err = atan(kappa_ref*L) - atan(kappa_meas*L)        front-wheel angle (rad)
  P_nm      = T_CAP_SLOPE_BASE * kappa_scale * v^2 * delta_err  (+ sl_bias)
Every CONTROL_HORIZON_S/2 a decision takes P through the curve hold-floor,
relax-dwell, the noise guard, STEP_MAX, t_cap and the stall-breakaway block,
and the torque ramps to it over the next half horizon; the free-play take-up
closes the servo lash on each change of push direction.

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

# Curve hold gate on commanded lateral accel v^2*|kappa_ref| (m/s^2): below [0]
# stiction holds the wheel at zero torque; at/above [1] the standing torque is
# kept — self-aligning torque beats stiction there (routes 380/384, 3ca seg 23).
HOLD_AY_BP = [0.5, 0.9]


def hold_factor(v_ego, kappa_des_abs):
  """Curve-hold blend factor [0, 1] on commanded lateral accel; pure function."""
  return float(np.interp(v_ego * v_ego * kappa_des_abs, HOLD_AY_BP, [0.0, 1.0]))


# Stall/breakaway v2 (StallBreakaway switch; route 3f2 seg 10 is the event).
# A stalled rack that releases sweeps the wheel far past the command while the
# controller still pushes on a stale error. The machine ARMs on a frozen wheel
# while ramping, notes the BREAKAWAY (where and which way it moved) and TRIPs
# when the wheel has travelled SB_TRIP_DISP_DEG past that point, in that
# direction, at >= SB_TRIP_RATE_DPS, outside a deep curve: it sheds the push
# and blocks same-side re-pushes for SB_BLOCK_TICKS. Everything is a
# difference of angle samples, so no absolute angle or curvature sign is used.
ANGLE_QUANTUM_DEG = 0.04395   # steerAngleDeg LSB
SB_FROZEN_TICKS = 40          # 0.4 s @100 Hz, span < 2 quanta = frozen (arm)
SB_MOVE_TICKS = 20            # 0.2 s window: breakaway + rate measurement
SB_MOVE_QUANTA = 3            # >= 3 quanta advance = breakaway (creep is 1-2)
SB_EPISODE_TICKS = 200        # 2 s max episode after breakaway
SB_TRIP_DISP_DEG = 2.0        # wheel travel past the breakaway point (user ruling)
SB_TRIP_RATE_DPS = 30.0       # sweep speed over the 0.2 s window: ordinary corrections
                              # stay below 30 deg/s on 3f4, the 3f2 release ran 31-70
SB_TRIP_KAPPA_MAX = 0.010     # |kappa_ref| above this: SAT self-arrests a release, stay out
SB_TRIP_MIN_TORQUE = 0.12     # frac (~1.4 Nm): a release without windup has nothing to shed
                              # (8 of 9 benign trips on 3fa/3fb were corner-exit unwinds at
                              # 0.007-0.088; the 3f2 event carried 0.293)
SB_SHED_FRAMES = 10           # drain torque -> 0 over 100 ms on trip
SB_BLOCK_TICKS = 50           # 0.5 s same-side push suppression after trip
SB_SETTLED_RAD = 0.001        # episode ends once |delta_err| is within the error noise floor (rad)

# Gain reference speed floor (m/s). The torque law references max(v_true, floor)
# so the v^2 gain stops collapsing at low speed; 10 km/h leaves walking-pace
# commands below the ~2.75 Nm breakaway (routes 411/413 seg 2, user ruling).
V_GAIN_FLOOR = 2.778

# LKA regime: DCC cannot engage below 30 km/h, so v_true < LKA_MAX_V is lateral-
# only driving. Its two behaviours (relax-dwell length, unwind step boost) blend
# in over [LKA_MAX_V, LKA_BLEND_V] because the plant does not step at 30 km/h
# (route 41b: a hard threshold switched the law mid-turn).
LKA_MAX_V = 8.5            # m/s (30 km/h) -- lower edge of the blend band
LKA_BLEND_V = 12.0         # m/s (43 km/h) -- upper edge of the blend band (user choice)
UNWIND_STEP_GAIN = 3.0     # step-cap multiplier while shedding torque in LKA: stiction
                           # holds the freed wheel there, so a fast release is safe (route 417)


def gain_reference_speed(v_true):
  """Speed the torque parameters (gains and caps) reference; identity above V_GAIN_FLOOR."""
  return max(float(v_true), V_GAIN_FLOOR)

# Controller timing horizon (s): the decision cadence and the ramp spread are
# each half of it. Fixed, independent of lat_delay: a 0.95 s horizon stretched
# the cadence to ~500 ms of added lag (route 4d8) and shrank hold_cap below SAT.
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

  Decision, every CONTROL_HORIZON_S/2 while active, on the full error:
    P_nm   = T_CAP_SLOPE_BASE * kappa_scale(|kappa_ref|) * v^2 * delta_err  (+ sl_bias)
    target = P, trimmed from or floored at the curve hold (hold_f * torque,
             sign-guarded, capped at hold_cap), bridged by relax-dwell in a
             deep curve, clipped to t_cap, sign changes gated by the noise
             guard, moved at most STEP_MAX from the current torque, zeroed on
             a same-side push during a stall-breakaway block.
  The torque ramps to the target over the next half horizon, one CAN tick at
  a time; the free-play take-up (bmw/takeup.py) owns the torque while it
  crosses the servo lash on a change of push direction. Sub-breakaway targets
  are commanded as they are: on this hydraulic rack a torque too small to
  move the column is, in effect, a hold.

  Inactive: no decision runs and torque/target/ramp are zero, so each
  engagement ramps from 0 on its first livePose tick.
  """
  import math
  from cereal import log
  from cereal import messaging
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

  # Plugin params are read once here, never per tick: a file read on
  # controlsd's 100 Hz thread can cost a control frame under eMMC contention,
  # and controlsd is onroad-only, so every drive re-reads them. The import is
  # at function scope so a missing config.py (install.sh copies it in a later
  # step) defaults the switch instead of failing the hook module. StallBreakaway
  # defaults off; '1' enables, at the next drive start.
  try:
    from config import read_plugin_param
    _stall_v2_on = read_plugin_param('bmw_e9x_e8x', 'StallBreakaway', '') == '1'
  except Exception:
    _stall_v2_on = False
  # SlopeCompensation is read at each engagement edge in update() instead (a
  # rare event, not a per-tick read), so the user can A/B by re-engaging.
  # Default off; a failed read (incl. read_plugin_param unbound) is off.

  # Decision cadence (livePose ticks) and CAN-rate ramp spread: both are half
  # of CONTROL_HORIZON_S, so two decision-and-ramp cycles fit one horizon and
  # each ramp completes as the next decision lands. Panda enforces wire-rate.
  DT_LIVEPOSE = 0.05                        # one livePose tick (s)
  DT_CAN_TICK = 0.01                        # one CAN tick at 100 Hz (s)
  ACTION_CADENCE_TICKS_FALLBACK = 5         # state-init only: primes the first decision
  # Plant-inversion gain, linear tire regime: tau_Nm = T_CAP_SLOPE * v^2 * delta
  # drives both the target and the authority cap
  #   t_cap = min(STEER_MAX, T_CAP_BASE_NM + T_CAP_SLOPE_BASE*kappa_scale*v^2*|delta_ref|).
  # kappa_scale raises the gain from 1.0 on straights (1.5 rang near-straight,
  # route 326) to 3.0 at |kappa_ref| >= 0.02, where the planner must be chased.
  T_CAP_BASE_NM = 2.0
  T_CAP_SLOPE_BASE = 1.0
  T_CAP_SCALE_KAPPA = [0.001, 0.01, 0.02]        # |kappa_ref| breakpoints (1/m)
  T_CAP_SCALE_BP    = [1.0, 2.5, 3.0]           # scale factor on T_CAP_SLOPE_BASE

  # Reference lateral drift (m) the hold cap is sized at: the P value for this
  # drift over the horizon bounds the held torque (route 385 seg 27).
  HOLD_CAP_DRIFT_M = 0.10

  # Per-decision torque step cap (frac): human-style gradual steering, 1.2 Nm
  # per decision up to 54 km/h (~4 Nm/s) tapering to 0.6 Nm at 100 km/h, where
  # lane margin is consumed faster (routes 385 seg 27, 39b seg 18).
  STEP_MAX_V  = [15.0, 28.0]     # vEgo breakpoints (m/s)
  STEP_MAX_BP = [0.10, 0.05]     # per-decision torque step cap (frac)

  # Relax-dwell: in a measured deep curve an overshoot-side error must persist
  # this long before the decision may command below the current torque — it
  # bridges the model's ~1 s mid-hairpin reference dips (route 3a0 seg 8).
  RELAX_DWELL_TICKS = 20      # livePose ticks (1.0 s at 20 Hz)
  RELAX_DWELL_KAPPA = 0.010   # |kappa_meas| defining a deep curve (1/m)

  # Rear-axle bicycle-model wheelbase (m), for kappa <-> delta conversion.
  L = float(CP.wheelbase)

  _sm = messaging.SubMaster(['livePose'])
  _learner = ContactLearner()

  state = {
    'torque': 0.0,             # current commanded torque fraction (advances by ramp_step each CAN tick)
    'target_frac': 0.0,        # plant-inversion target set each decision (cadence = CONTROL_HORIZON_S/2)
    'ramp_step': 0.0,          # per-frame torque increment = (target − torque) / spread_frames
    'ramp_frames': 0,          # CAN frames left in current ramp
    'tick_count': ACTION_CADENCE_TICKS_FALLBACK,  # primed so first livePose tick fires cadence immediately (no engagement gap)
    'action': 'init',          # debug: ramp / relax_dwell / idle / take_up
    'delta_err': 0.0,          # debug: front-wheel-angle error (rad), what controller acts on
    'lat_pub': None,
    'desired': 0.0, 'measured': 0.0,
    'hold_f': 0.0,                # debug: curve hold factor [0,1] (gated on v²·|kappa_ref|)
    'hold_cap': 0.0,              # debug: cap on held torque (P value at the HOLD_CAP_DRIFT_M reference drift, frac)
    'relax_ticks': 0,             # consecutive livePose ticks of overshoot-side error while deep in a curve
    # Stall/breakaway v2 (see the SB_* constants at module scope).
    'sb_ring': deque(maxlen=SB_FROZEN_TICKS + 1),  # 100 Hz steerAngleDeg, active only
    'sb_state': 0,          # 0=idle, 1=armed(stall), 2=breakaway episode
    'sb_brk_angle': 0.0,    # steerAngleDeg at breakaway (displacement reference)
    'sb_dir': 0.0,          # +1/-1: breakaway MOTION direction (self-referenced)
    'sb_arm_kappa': 0.0,    # |kappa_ref| LATCHED at ARM (internal, not telemetry)
    'sb_episode_ticks': 0,  # countdown in state 2
    'sb_block': 0,          # same-side push suppression countdown after a trip
    'sb_trips': 0,          # cumulative (telemetry)
    # Direct Tracking.
    'was_active': False,     # previous tick's `active`, for the engagement edge
    'guard': SignGuard(),    # noise guard on P-target sign reversals
    'learner': _learner,     # servo-column contacts, learned every drive
    'takeup': FreePlayTakeUp(_learner, CCP.STEER_MAX),   # lash take-up
    'tu_resume': 'init',     # decision label under 'take_up', restored when it ends
    'tu_demand': 0.0,        # last decision's target before STEP_MAX (take-up size gate)
    'slope': False,          # SlopeCompensation latched at the engagement edge
    'sl_bias': 0.0,          # learned straight-road torque (Nm); kept across engagements, never persisted
    'sl_learn': False,       # learn gate open this livePose tick (telemetry)
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
      state['guard'].reset()
    elif not active:
      state['slope'] = False
      # Inactive: the law does not run on the state. Torque, target and ramp
      # are zeroed so the next engagement ramps from 0 under STEP_MAX on its
      # first livePose tick instead of jumping to a torque built while the
      # output was 0 (the rack never received it). A take-up in flight ends.
      state.update(torque=0.0, target_frac=0.0, ramp_step=0.0, ramp_frames=0)
      state['takeup'].reset()
      if state['action'] == 'take_up':
        state['action'] = state['tu_resume']
    state['was_active'] = active

    # steeringAngleDeg carries a constant alignment offset (~-1.58 deg): the
    # stall/breakaway machine reads deltas only, so it never needs to know it.
    # getattr: CS is a stub in some test paths.
    _angle = float(getattr(CS, 'steeringAngleDeg', 0.0))

    # ---- Stall/breakaway v2 state machine (100 Hz) -----------------------
    # Runs before the livePose branch; a same-tick cadence decision can
    # overwrite a trip's shed, which the shed-rate enforcement below undoes —
    # unless that decision is a counter-steer, which always wins.
    if active:
      state['sb_ring'].append(_angle)
    else:
      state['sb_ring'].clear(); state['sb_state'] = 0; state['sb_block'] = 0
    if state['sb_block'] > 0:
      state['sb_block'] -= 1
      # Shed-rate enforcement: a cadence decision inside the shed window
      # recomputes ramp_step over spread_frames, stretching the 100 ms drain
      # ~6x, and the 3f2 event peaked 0.30 s after the trip — drain rate is
      # the point. While the standing torque is still a same-side push and the
      # ramp is not draining at least that fast, re-assert the fast drain.
      # Torque is opposite in sign to angle, so the push behind the sb_dir
      # motion has torque * (-sb_dir) > 0. A ramp headed to the opposite side
      # is a counter-steer: it may stop pushing, it must never be zeroed.
      _to_counter = state['target_frac'] * (-state['sb_dir']) < 0.0
      if state['torque'] * (-state['sb_dir']) > 0.0 and not _to_counter:
        _drain = abs(state['torque']) / SB_SHED_FRAMES
        _draining = (state['ramp_frames'] > 0
                     and state['ramp_step'] * state['torque'] < 0.0
                     and abs(state['ramp_step']) >= _drain)
        if not _draining:
          state['target_frac'] = 0.0
          state['ramp_step'] = -state['torque'] / SB_SHED_FRAMES
          state['ramp_frames'] = SB_SHED_FRAMES
    if _stall_v2_on and active:
      ring = state['sb_ring']
      if state['sb_state'] == 0:
        # ARM: a full 0.4 s ring inside 2 quanta while pushing = the rack is
        # not following. Common and harmless on its own (926 arms/85 min on
        # 3f4); arming does nothing but latch the in-corner curvature, so a
        # corner-exit decay cannot sneak the episode under the deep-curve gate.
        if state['action'] == 'ramp' and len(ring) == ring.maxlen \
           and (max(ring) - min(ring)) < 2 * ANGLE_QUANTUM_DEG:
          state['sb_state'] = 1
          state['sb_arm_kappa'] = abs(state['desired'])
      elif state['sb_state'] == 1:
        if state['action'] != 'ramp':
          state['sb_state'] = 0
        elif len(ring) > SB_MOVE_TICKS \
             and abs(ring[-1] - ring[-1 - SB_MOVE_TICKS]) >= SB_MOVE_QUANTA * ANGLE_QUANTUM_DEG:
          # BREAKAWAY: the stalled rack moved. Latch where, and which way —
          # observed, not derived from any sign convention.
          state['sb_state'] = 2
          state['sb_episode_ticks'] = SB_EPISODE_TICKS
          state['sb_brk_angle'] = ring[-1]
          state['sb_dir'] = 1.0 if ring[-1] >= ring[-1 - SB_MOVE_TICKS] else -1.0
      elif state['sb_state'] == 2:
        state['sb_episode_ticks'] -= 1
        # Rate signed by the breakaway direction: positive means advancing the
        # way it broke free, the only motion a release produces (an unsigned
        # rate tripped on a fast snap BACK toward the breakaway point).
        _rate = (ring[-1] - ring[-1 - SB_MOVE_TICKS]) * state['sb_dir'] / (SB_MOVE_TICKS * DT_CAN_TICK) \
                if len(ring) > SB_MOVE_TICKS else 0.0
        # The episode also ends once the error has settled (last livePose value).
        if state['sb_episode_ticks'] <= 0 or abs(state['delta_err']) <= SB_SETTLED_RAD:
          state['sb_state'] = 0
        # The curvature gate is applied both instantaneous and latched-at-arm:
        # on 3fb kappa_ref decayed 0.019 -> 0.0096 during a hairpin exit and a
        # plain unwind read as a release. SB_TRIP_MIN_TORQUE: no windup, nothing
        # to shed.
        elif state['action'] != 'relax_dwell' \
             and abs(state['torque']) >= SB_TRIP_MIN_TORQUE \
             and abs(state['desired']) <= SB_TRIP_KAPPA_MAX \
             and state['sb_arm_kappa'] <= SB_TRIP_KAPPA_MAX \
             and (ring[-1] - state['sb_brk_angle']) * state['sb_dir'] >= SB_TRIP_DISP_DEG \
             and _rate >= SB_TRIP_RATE_DPS:
          # TRIP: shed the push to 0 over SB_SHED_FRAMES (same-side decrease
          # with the wheel already moving that way) and block same-side
          # re-pushes so the stale-error P law cannot rebuild the surplus. The
          # shed is sign-gated: standing counter-steer (or zero torque) is kept
          # — the surplus to dump is same-side torque — and the trip is still
          # recorded (sb_trips) because it happened.
          if state['torque'] * (-state['sb_dir']) > 0.0:
            state['target_frac'] = 0.0
            state['ramp_step'] = -state['torque'] / SB_SHED_FRAMES
            state['ramp_frames'] = SB_SHED_FRAMES
            state['sb_block'] = SB_BLOCK_TICKS
          state['sb_trips'] += 1
          state['sb_state'] = 0
    else:
      state['sb_state'] = 0


    _sm.update(0)
    lp = _sm['livePose']
    livepose_updated = _sm.updated['livePose']

    # livePose tick (20 Hz): measured and desired every tick, a decision every
    # action_cadence_ticks (6 ticks, 300 ms: >= 2.5 tau of the rack's ~130 ms response,
    # so the plant has answered the previous step). CAN tick (100 Hz): ramp.
    if livepose_updated:
      # Cadence and ramp length, both half of CONTROL_HORIZON_S; spread_frames
      # is derived from the cadence (5 CAN ticks per livePose tick) so the ramp
      # ends exactly when the next decision lands (6 ticks = 30 frames = 300 ms).
      half_horizon = 0.5 * CONTROL_HORIZON_S
      action_cadence_ticks = max(1, round(half_horizon / DT_LIVEPOSE))
      spread_frames = action_cadence_ticks * int(DT_LIVEPOSE / DT_CAN_TICK)

      # No filtering here: desired_curvature is already the lane_keeping Arc
      # reference, low-passed upstream. This controller tracks it faithfully.
      state['desired'] = float(desired_curvature)

      # Two speeds (user ruling): v_true divides the yaw rate — kappa_meas must
      # use the actual speed or the loop settles at the wrong curvature at low
      # speed; v is the gain reference (P, t_cap, hold_cap, hold gate, step_max),
      # floored at V_GAIN_FLOOR so v^2 never collapses toward zero.
      v_true = max(float(lp.velocityDevice.x) if _sm.seen['livePose'] else CS.vEgo, 1.0)
      v = gain_reference_speed(v_true)
      state['measured'] = float(lp.angularVelocityDevice.z) / v_true

      # Curve hold factor on commanded a_y (see HOLD_AY_BP): in a curve the
      # torque that holds the wheel on target approximates the self-aligning
      # torque, which exceeds rack stiction; letting go there unwinds the wheel
      # (route 380/384: a 0.6 Hz limit cycle). Re-derived from state['torque']
      # every decision: no learning state, no ratchet.
      hold_f = hold_factor(v, abs(state['desired']))
      state['hold_f'] = hold_f

      # Front-wheel-angle error (rear-axle bicycle model).
      delta_des = math.atan(state['desired'] * L)
      delta_meas = math.atan(state['measured'] * L)
      delta_err_raw = delta_des - delta_meas

      delta_err = delta_err_raw
      state['delta_err'] = delta_err

      # Look-ahead distance for the hold cap, on the fixed horizon.
      lookahead_m = v * CONTROL_HORIZON_S

      # Curvature scale for the P term and caps, keyed on |kappa_ref|.
      kappa_scale = float(np.interp(abs(state['desired']),
                                    T_CAP_SCALE_KAPPA, T_CAP_SCALE_BP))

      # Slope bias learning (see SL_TAU_S): integrate the P term alone on
      # straights. The gate never looks at the error's sign.
      state['sl_learn'] = bool(
        state['slope'] and active and not getattr(CS, 'steeringPressed', False)
        and v_true > SL_MIN_V
        and abs(state['desired']) < SL_KAPPA_DES and abs(state['measured']) < SL_KAPPA_MEAS
        and state['takeup'].state != 'taking_up'
        and state['sb_block'] == 0 and state['sb_state'] != 2)   # 1 = armed only: harmless
      if state['sl_learn']:
        _p_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err
        state['sl_bias'] = float(np.clip(state['sl_bias'] + (_p_nm / SL_TAU_S) * DT_LIVEPOSE,
                                         -SL_BIAS_MAX_NM, SL_BIAS_MAX_NM))

      # The noise guard judges reversal persistence on the 20 Hz error sign,
      # not the cadence target (Direct spec §3). With the slope bias latched it
      # observes the biased demand (P + sl_bias, as an error) — the sign that
      # allow() later judges; on delta_err alone a steady small error against
      # the bias would hold the target at 0 and drop the bias.
      if state['slope']:
        state['guard'].observe(delta_err + state['sl_bias']
                               / (T_CAP_SLOPE_BASE * kappa_scale * v * v))
      else:
        state['guard'].observe(delta_err)

      # Held torque for the curve hold: "keep what you have", sign-guarded
      # (never hold torque opposing the commanded curve — on a rack with no
      # self-centering that drives the wrong way, route 385 seg 27) and capped
      # at the P value for HOLD_CAP_DRIFT_M over the horizon (anything above
      # what P would command while almost on target is arrival momentum, not
      # holding torque; steady SAT measures 0.08-0.15 frac).
      hold_cap = (T_CAP_SLOPE_BASE * kappa_scale * v * v
                  * (2.0 * HOLD_CAP_DRIFT_M * L / (lookahead_m ** 2))) / CCP.STEER_MAX
      state['hold_cap'] = hold_cap
      held = hold_f * state['torque']
      if held * delta_des < 0.0:
        held = 0.0
      held_target = float(np.clip(held, -hold_cap, hold_cap))

      # Relax-dwell: the model's reference dips 40-50 % for ~1 s mid-hairpin
      # while the measured curvature holds (route 3a0 seg 8). Following the dip
      # surrenders held torque and SAT flings the wheel ~20 deg out of the
      # turn; rebuilding under STEP_MAX takes 1.5-2 s. So while deep in a
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

        # Plant-inversion target: the steady-state aligning torque for
        # delta_err, on the FULL error — no on-target band, a sub-friction
        # target is in effect a hold (Direct spec §2).
        #   tau_Nm = T_CAP_SLOPE_BASE * kappa_scale * v^2 * delta_err
        _tu_demand = None   # the decision's demand before the step cap (take-up size gate)
        target_nm = T_CAP_SLOPE_BASE * kappa_scale * v * v * delta_err
        if state['slope']:
          target_nm += state['sl_bias']
        target_frac = target_nm / CCP.STEER_MAX
        state['action'] = 'ramp'
        # Curve hold trim (ruling R8): an in-band overshoot gives a small P
        # opposing the held torque; taken alone it would drop the curve torque
        # a full STEP_MAX per decision (1.8 Nm held -> 1.06 Nm). P trims down
        # from the hold instead: held_target + P. A large overshoot still
        # carries the sum past zero. On straights held_target is 0.
        _trim_hold = (hold_f > 0.0
                      and target_frac * held_target < 0.0)
        if _trim_hold:
          target_frac = held_target + target_frac
        # Authority cap, v^2*|delta_ref|-scaled and clipped at STEER_MAX:
        # straights stay near T_CAP_BASE_NM, tight turns can reach the panda
        # limit (transient over-envelope; speedlimitd bleeds v).
        t_cap_nm = min(CCP.STEER_MAX,
                       T_CAP_BASE_NM + T_CAP_SLOPE_BASE * kappa_scale * v * v * abs(delta_des))
        t_cap_frac = t_cap_nm / CCP.STEER_MAX
        target_frac = float(np.clip(target_frac, -t_cap_frac, t_cap_frac))
        # Hold-floor: a same-direction push never commands less than the held
        # torque — keep holding while adding trim (route 393: a P below the
        # holding level let SAT unwind the wheel and grow the very error being
        # corrected). Opposite-sign P is untouched, so torque still drops the
        # moment the error flips sides. Not after an R8 trim, which is below
        # the hold by design.
        if not _trim_hold and target_frac * held_target > 0.0:
          target_frac = math.copysign(max(abs(target_frac), abs(held_target)), target_frac)
        # Relax-dwell bridge: during the dwell an overshoot-side reference dip
        # may not command below the current (capped) torque.
        if dwelling:
          target_frac = math.copysign(min(abs(state['torque']), hold_cap), state['torque'])
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
        # Post-trip block: no same-side re-push while a release settles;
        # counter-steer passes (the controller may stop pushing, never give up
        # correcting). sb_dir is the wheel's motion in angle space (+ = left)
        # and torque is opposite in sign to angle, so the push behind it is
        # target_frac * (-sb_dir) > 0. Gated on 'ramp': a relax-dwell hold of
        # the current torque is not a push and must not be zeroed.
        if state['action'] == 'ramp' and state['sb_block'] > 0 \
           and target_frac * (-state['sb_dir']) > 0.0:
          target_frac = 0.0

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
    # A stall-breakaway block owns the torque (its shed): end the take-up.
    if tu.state == 'taking_up' and state['sb_block'] > 0:
      tu.reset()
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
                         blocked=state['sb_block'] > 0 or _pressed,
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

    # Telemetry at livePose rate (20 Hz): most fields only change on livePose
    # ticks. The exact field set is pinned by test_payload_exact_key_set and
    # listed in LATERAL_CONTROLLER.md §9.
    if livepose_updated:
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
          'hold_f': float(state['hold_f']),
          'hold_cap': float(state['hold_cap']),
          'relax_ticks': int(state['relax_ticks']),
          'sb_state': int(state['sb_state']),                 # stall/breakaway v2: 0=idle, 1=armed, 2=episode
          'sb_trips': int(state['sb_trips']),                 # cumulative trips this drive
          'sb_block': int(state['sb_block']),                 # same-side push suppression ticks left
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
        }
        state['lat_pub'].send(payload)
      except Exception:
        pass

    return -output, 0.0, pid_log

  lac.update = update
  return result
