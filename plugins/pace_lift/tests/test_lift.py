import importlib.util
import math
import os

# Load the pure law by explicit path under a unique name (no sys.path insert).
_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_spec = importlib.util.spec_from_file_location('pace_lift_law_test', os.path.join(_DIR, 'lift.py'))
lift = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(lift)

T = [i * 0.5 for i in range(13)]  # 0..6 s
T_LONG = [i * 0.5 for i in range(21)]  # 0..10 s


def call(a_e2e=0.1, v_ego=22.0, v_cruise=27.0, strength=0.4, t_follow=1.25,
         plan_t=None, plan_v=None, plan_yaw=None, should_stop=False,
         lead_present=False, lead_d=0.0, lead_vrel=0.0, **kw):
  plan_t = T if plan_t is None else plan_t
  plan_v = [v_ego] * len(plan_t) if plan_v is None else plan_v
  plan_yaw = [0.0] * len(plan_t) if plan_yaw is None else plan_yaw
  return lift.lift_target(a_e2e, v_ego, v_cruise, strength, t_follow, plan_t, plan_v, plan_yaw,
                          should_stop, lead_present, lead_d, lead_vrel, **kw)


def test_constants():
  assert lift.STRENGTH_DEFAULT == {'aggressive': 0.40, 'standard': 0.20, 'relaxed': 0.0}
  assert lift.T_FOLLOW == {'aggressive': 1.25, 'standard': 1.45, 'relaxed': 1.75}
  assert lift.ASK_DECEL == -0.10


def test_full_lift_aggressive():
  t, g = call()
  assert g == 'active'
  assert math.isclose(t, 0.40)


def test_gap_ramp_midpoint():
  t, g = call(v_cruise=24.5)
  assert g == 'active'
  assert math.isclose(t, 0.20)


def test_negative_gap_no_lift():
  assert call(v_cruise=20.0) == (0.0, 'none_needed')


def test_speed_fade_in():
  t, _ = call(v_ego=15.28, v_cruise=25.0)
  assert math.isclose(t, 0.20, abs_tol=1e-3)
  assert call(v_ego=13.0, v_cruise=25.0) == (0.0, 'none_needed')


def test_ask_cap():
  assert math.isclose(lift.ask_cap(-0.05, 0.4), 0.2)
  assert lift.ask_cap(-0.10, 0.4) == 0.0
  assert math.isclose(lift.ask_cap(0.0, 0.4), 0.4)
  assert math.isclose(lift.ask_cap(0.3, 0.4), 0.4)
  assert lift.ask_cap(-0.5, 0.4) == 0.0


def test_ask_decel_gate():
  assert call(a_e2e=-0.10) == (0.0, 'ask_decel')
  assert call(a_e2e=-0.5) == (0.0, 'ask_decel')


def test_relaxed_zero():
  t, g = call(strength=0.0, t_follow=1.75)
  assert t == 0.0 and g == 'none_needed'


def test_should_stop_gate():
  assert call(should_stop=True) == (0.0, 'stop')


def test_plan_decel_gate():
  assert lift.PLAN_DECEL_MARGIN == 2.0
  v = [22.0] * len(T)
  v[T.index(4.0)] = 22.0 - 1.5   # a 1.5 m/s dip no longer trips (V8 margin 2.0)
  assert call(plan_v=v)[1] == 'active'
  v[T.index(4.0)] = 22.0 - 2.5
  assert call(plan_v=v) == (0.0, 'plan_decel')
  v2 = [22.0] * len(T_LONG)
  v2[T_LONG.index(7.0)] = 22.0 - 2.5   # the same dip at t 7 s is outside the 6 s window
  assert call(plan_t=T_LONG, plan_v=v2)[1] == 'active'


def test_curve_gate():
  assert call(plan_yaw=[0.07] * len(T)) == (0.0, 'curve')
  assert call(plan_yaw=[0.05] * len(T))[1] == 'active'


def test_lead_gap_gate():
  assert call(lead_present=True, lead_d=44.0, lead_vrel=0.0) == (0.0, 'lead_gap')
  assert call(lead_present=True, lead_d=45.0, lead_vrel=0.0)[1] == 'active'
  # v 15: threshold 1.25*15 + 6 + 7.5 = 32.25
  assert call(v_ego=15.0, v_cruise=20.0, lead_present=True, lead_d=32.0)[1] == 'lead_gap'
  assert call(v_ego=15.0, v_cruise=20.0, lead_present=True, lead_d=32.5)[1] == 'active'
  # no lead present: distance ignored
  assert call(lead_present=False, lead_d=1.0)[1] == 'active'


def test_lead_closing_gate():
  assert call(lead_present=True, lead_d=80.0, lead_vrel=-0.6) == (0.0, 'lead_closing')
  assert call(lead_present=True, lead_d=80.0, lead_vrel=-0.4)[1] == 'active'


def test_gate_order():
  v = [22.0] * len(T)
  v[3] = 19.0
  assert call(plan_v=v, plan_yaw=[0.07] * len(T), lead_present=True, lead_d=10.0)[1] == 'plan_decel'
  assert call(plan_yaw=[0.07] * len(T), lead_present=True, lead_d=10.0)[1] == 'curve'
  assert call(should_stop=True, a_e2e=-0.5)[1] == 'stop'


def test_short_model_arrays():
  assert call(plan_t=[0.0], plan_v=[22.0], plan_yaw=[0.0]) == (0.0, 'no_model')
  assert call(plan_t=[], plan_v=[], plan_yaw=[]) == (0.0, 'no_model')
  assert call(plan_v=[22.0] * 5) == (0.0, 'no_model')  # length mismatch


def test_nonfinite_inputs():
  nan = float('nan')
  v = [22.0] * len(T)
  v[2] = nan
  assert call(plan_v=v) == (0.0, 'no_model')
  assert call(plan_yaw=[0.0, nan] + [0.0] * (len(T) - 2)) == (0.0, 'no_model')
  assert call(lead_present=True, lead_d=nan) == (0.0, 'no_model')
  assert call(lead_present=True, lead_d=40.0, lead_vrel=nan) == (0.0, 'no_model')
  assert call(v_cruise=nan) == (0.0, 'no_model')
  assert call(v_cruise=float('inf')) == (0.0, 'no_model')
  assert call(a_e2e=nan) == (0.0, 'no_model')
  assert call(v_ego=nan) == (0.0, 'no_model')
  assert call(strength=nan) == (0.0, 'no_model')


def test_no_model_precedes_stop():
  assert call(should_stop=True, v_cruise=float('nan')) == (0.0, 'no_model')


def test_lead_gap_hysteresis_kwarg():
  # aggressive, v 22: trip 27.5 + 6 + 0.5*22 = 44.5, clear margin 0.8 s -> 51.1
  assert lift.LEAD_GAP_CLEAR_MARGIN_S == 0.8
  assert call(lead_present=True, lead_d=50.0)[1] == 'active'
  assert call(lead_present=True, lead_d=50.0, lead_gap_margin_s=0.8) == (0.0, 'lead_gap')
  assert call(lead_present=True, lead_d=51.0, lead_gap_margin_s=0.8)[1] == 'lead_gap'
  assert call(lead_present=True, lead_d=51.2, lead_gap_margin_s=0.8)[1] == 'active'


def test_lead_closing_hysteresis_kwarg():
  assert lift.LEAD_CLOSING_CLEAR_VREL == -0.2
  kw = dict(lead_present=True, lead_d=90.0)
  assert call(lead_vrel=-0.3, **kw)[1] == 'active'
  assert call(lead_vrel=-0.3, lead_closing_vrel=-0.2, **kw) == (0.0, 'lead_closing')
  assert call(lead_vrel=-0.1, lead_closing_vrel=-0.2, **kw)[1] == 'active'
