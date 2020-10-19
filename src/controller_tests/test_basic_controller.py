from decimal import Decimal

from nose.tools import assert_equal

from controller.basic_controller import BasicController
from notes import VehicleState
from vehicle.state_estimation import Belief, Action
from vehicle.vehicle import VehicleStats

prob_treshold = Decimal('0.1')
a_min_m_s2 = Decimal('-5.0')
a_max_m_s2 = Decimal('5.0')
v_nominal_m_s = Decimal('10.0')
mass_g = Decimal('1000000.0')
ds_m = Decimal('0.1')
frequency_1_s = Decimal('1')
d_stop_m = Decimal('2.0')
vs = VehicleStats(a_min=a_min_m_s2, a_max=a_max_m_s2,
                  v_nominal=v_nominal_m_s, mass=mass_g)
controller = BasicController(prob_threshold=prob_treshold, vs=vs, ds=ds_m,
                             d_stop=d_stop_m, frequency=frequency_1_s)
x_test_m = Decimal('0.0')
d_critical_expect_m = Decimal('4.5')


def critical_distance_test1() -> None:
    v_test_m_s = Decimal('5.0')
    d_critical_m = controller.get_critical_distance(v=v_test_m_s)
    assert_equal(d_critical_m, d_critical_expect_m)


def get_action_test1() -> None:
    v_test_m_s = Decimal('5.0')
    v_state = VehicleState(x=x_test_m, v=v_test_m_s)
    po_test1 = [Decimal(i * 0.01) for i in range(10)]
    belief_test1 = Belief(po=po_test1)
    action_test1_expect = Action(a_min_m_s2)
    action = controller.get_action(vstate=v_state, belief=belief_test1)
    assert_equal(action.accel, action_test1_expect.accel)


def get_action_test2() -> None:
    v_test_m_s = Decimal('5.0')
    v_state = VehicleState(x=x_test_m, v=v_test_m_s)
    po_test2 = [Decimal(0.005) for _ in range(10)]
    belief_test2 = Belief(po=po_test2)
    action_test2_expect = Action(Decimal('2.5'))
    action = controller.get_action(vstate=v_state, belief=belief_test2)
    assert_equal(action.accel, action_test2_expect.accel)


def get_action_test3() -> None:
    v_test3_m_s = v_nominal_m_s
    v_state_3 = VehicleState(x=x_test_m, v=v_test3_m_s)
    po_test3 = [Decimal(0.005) for _ in range(10)]
    belief_test3 = Belief(po=po_test3)
    action = controller.get_action(vstate=v_state_3, belief=belief_test3)
    action_test3_expect = Action(Decimal('0'))
    assert_equal(action_test3_expect.accel, action.accel)
