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
