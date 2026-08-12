import math
from bmw.rack_motion import (RackMotion, ANGLE_LSB_DEG, WINDOW_S,
                             MOTION_THRESHOLD_DEG_S, TORQUE_TO_ANGLE_SIGN)


def _feed(rm, rate_deg_s, duration_s=0.30, dt=0.01, start_angle=0.0, quantise=True):
    """Drive the observer with a constant-rate ramp, optionally LSB-quantised."""
    t = 0.0
    angle = start_angle
    while t <= duration_s + 1e-9:
        a = angle
        if quantise:
            a = round(a / ANGLE_LSB_DEG) * ANGLE_LSB_DEG
        rm.update(t, a)
        t += dt
        angle += rate_deg_s * dt
    return rm


def test_rate_nan_before_window_fills():
    rm = RackMotion()
    rm.update(0.0, 1.0)
    rm.update(0.01, 1.0)
    assert math.isnan(rm.rate_deg_s)


def test_constant_rate_recovered_within_resolution():
    rm = _feed(RackMotion(), 20.0)
    assert abs(rm.rate_deg_s - 20.0) < 1.0


def test_stationary_reads_zero_rate():
    rm = _feed(RackMotion(), 0.0)
    assert abs(rm.rate_deg_s) < 0.55


def test_constant_offset_cancels_exactly_without_quantisation():
    """The algorithm is offset-immune: differencing removes any constant.

    Quantisation is off here so this tests the property itself. With it on,
    an offset that is not a whole number of LSBs (-1.58 / 0.0879 = 17.97)
    shifts the sampling phase, so cancellation is exact only in exact
    arithmetic — see the companion test below.
    """
    a = _feed(RackMotion(), 12.0, start_angle=0.0, quantise=False).rate_deg_s
    b = _feed(RackMotion(), 12.0, start_angle=-1.58, quantise=False).rate_deg_s
    assert abs(a - b) < 1e-9


def test_constant_offset_cancels_within_quantisation_noise():
    """With LSB quantisation the residual is bounded by the noise floor."""
    a = _feed(RackMotion(), 12.0, start_angle=0.0).rate_deg_s
    b = _feed(RackMotion(), 12.0, start_angle=-1.58).rate_deg_s
    assert abs(a - b) < 0.55          # ANGLE_LSB_DEG / WINDOW_S


def test_is_moving_threshold():
    assert not _feed(RackMotion(), 1.0).is_moving()
    assert _feed(RackMotion(), 5.0).is_moving()


def test_is_moving_with_torque_requires_matching_direction():
    # Negative torque commands LEFT; LEFT is POSITIVE steering angle.
    rm = _feed(RackMotion(), +20.0)          # wheel moving left
    assert rm.is_moving_with_torque(-0.20)   # left torque -> agrees
    assert not rm.is_moving_with_torque(+0.20)  # right torque -> disagrees


def test_zero_torque_never_counts_as_moving_with_torque():
    rm = _feed(RackMotion(), +20.0)
    assert not rm.is_moving_with_torque(0.0)


def test_reset_clears_history():
    rm = _feed(RackMotion(), 20.0)
    rm.reset()
    assert math.isnan(rm.rate_deg_s)


def test_stale_samples_are_evicted():
    rm = RackMotion()
    _feed(rm, 40.0, duration_s=0.30)
    _feed(rm, 0.0, duration_s=0.30, start_angle=100.0)
    assert abs(rm.rate_deg_s) < 0.55


def test_sign_convention_constant_is_negative():
    assert TORQUE_TO_ANGLE_SIGN == -1.0


from bmw.rack_motion import (BreakawayEstimator, BREAKAWAY_SEED_FRAC,
                             BREAKAWAY_MIN_FRAC, BREAKAWAY_MAX_FRAC, SUSTAIN_RATIO)


def test_seed_is_the_measured_knee_not_the_old_friction_constant():
    # Measured knee 2.0-2.75 Nm at STEER_MAX=12 -> 0.167-0.229 frac.
    # The old FRICTION was 0.05. The seed must not be that.
    assert 0.15 <= BREAKAWAY_SEED_FRAC <= 0.25
    assert BreakawayEstimator().breakaway_frac == BREAKAWAY_SEED_FRAC


def test_no_observation_leaves_seed_untouched():
    est = BreakawayEstimator()
    for _ in range(50):
        est.update(0.30, moving_with_torque=False)
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC
    assert est.observations == 0


def test_records_torque_at_the_stationary_to_moving_transition():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    for _ in range(4):
        est.update(0.30, moving_with_torque=True)   # transition, confirmed at tick 4
    assert est.observations == 1
    assert est.breakaway_frac > BREAKAWAY_SEED_FRAC


def test_sustained_motion_records_only_once():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    for _ in range(20):
        est.update(0.30, moving_with_torque=True)
    assert est.observations == 1


def test_converges_toward_repeated_observations():
    est = BreakawayEstimator()
    for _ in range(60):
        est.update(0.30, moving_with_torque=False)
        for _ in range(4):
            est.update(0.30, moving_with_torque=True)   # confirm each cycle
    assert abs(est.breakaway_frac - 0.30) < 0.02


def test_observation_is_clamped_to_sane_range():
    est = BreakawayEstimator()
    for _ in range(200):
        est.update(0.95, moving_with_torque=False)
        for _ in range(4):
            est.update(0.95, moving_with_torque=True)   # confirm each cycle
    assert est.breakaway_frac <= BREAKAWAY_MAX_FRAC + 1e-9


def test_sign_is_ignored_only_magnitude_matters():
    a = BreakawayEstimator()
    b = BreakawayEstimator()
    a.update(+0.30, False)
    for _ in range(4):
        a.update(+0.30, True)
    b.update(-0.30, False)
    for _ in range(4):
        b.update(-0.30, True)
    assert abs(a.breakaway_frac - b.breakaway_frac) < 1e-9


def test_sustain_is_a_fraction_of_breakaway():
    est = BreakawayEstimator()
    assert abs(est.sustain_frac - SUSTAIN_RATIO * est.breakaway_frac) < 1e-9
    assert est.sustain_frac < est.breakaway_frac


def test_reset_restores_seed():
    est = BreakawayEstimator()
    est.update(0.30, False); est.update(0.30, True)
    est.reset()
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC
    assert est.observations == 0


def test_first_ever_sample_moving_is_not_counted_as_a_breakaway():
    """Engaging mid-turn with the wheel already moving must not record."""
    est = BreakawayEstimator()
    for _ in range(4):
        est.update(0.15, moving_with_torque=True)   # confirmed, but never armed
    assert est.observations == 0
    assert est.breakaway_frac == BREAKAWAY_SEED_FRAC


def test_arming_requires_seeing_the_rack_stationary_first():
    est = BreakawayEstimator()
    est.update(0.15, moving_with_torque=True)
    est.update(0.15, moving_with_torque=True)
    assert est.observations == 0
    est.update(0.15, moving_with_torque=False)   # arms here
    for _ in range(4):
        est.update(0.15, moving_with_torque=True)   # confirmed at tick 4
    assert est.observations == 1


def test_reset_disarms_the_edge_detector():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)
    for _ in range(4):
        est.update(0.30, moving_with_torque=True)
    est.reset()
    for _ in range(4):
        est.update(0.15, moving_with_torque=True)   # confirmed, but disarmed by reset
    assert est.observations == 0


def test_short_motion_blip_is_not_a_breakaway():
    """1-2 tick threshold-grazing artifacts must not be recorded."""
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)     # arm
    est.update(0.30, moving_with_torque=True)
    est.update(0.30, moving_with_torque=True)      # only 2 ticks, below confirm
    est.update(0.30, moving_with_torque=False)
    assert est.observations == 0


def test_sustained_motion_is_recorded_once_after_confirmation():
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)     # arm
    for _ in range(20):
        est.update(0.30, moving_with_torque=True)
    assert est.observations == 1


def test_records_the_torque_at_onset_not_after_confirmation():
    """Torque ramps while we confirm; the breakaway value is the onset one."""
    est = BreakawayEstimator()
    est.update(0.30, moving_with_torque=False)     # arm
    est.update(0.30, moving_with_torque=True)      # onset torque = 0.30
    for _ in range(10):
        est.update(0.90, moving_with_torque=True)  # torque climbs afterwards
    assert est.observations == 1
    # EMA of seed 0.20 toward the clamped onset 0.30, alpha 0.10 -> 0.21
    assert abs(est.breakaway_frac - 0.21) < 1e-9
