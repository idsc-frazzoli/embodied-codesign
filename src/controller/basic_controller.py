from decimal import Decimal
import numpy as np

from controller.controller import Controller
from sensing.sensing_performance import SensingParameters
from vehicle.state_estimation import Belief, Action
from vehicle.vehicle import VehicleStats, VehicleState


class BasicController(Controller):

    def __init__(self, prob_threshold: Decimal, vs: VehicleStats, ds: Decimal,
                 d_stop: Decimal, frequency: Decimal):
        self.prob_threshold = prob_threshold
        self.vs = vs
        self.ds = ds
        self.d_stop = d_stop
        self.frequency = frequency
        assert frequency > 0, frequency

    def get_critical_distance(self, v: Decimal) -> Decimal:
        d_critical = v ** 2 / (2 * abs(self.vs.a_min)) + self.d_stop
        return d_critical

    def get_action(self, vstate: VehicleState, belief: Belief) -> Action:
        d_critical = self.get_critical_distance(vstate.v)
        i = int(np.ceil(d_critical / self.ds))
        p_obstacle_less_than_critical = sum(belief.po[:i])
        if float(p_obstacle_less_than_critical) > float(self.prob_threshold):
            a = self.vs.a_min
        elif vstate.v < self.vs.v_nominal:
            a = self.vs.a_max*Decimal('0.5')
        else:
            a = 0

        return Action(a)
