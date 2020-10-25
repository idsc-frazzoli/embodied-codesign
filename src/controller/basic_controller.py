from decimal import Decimal
from typing import List

import numpy as np

from controller.controller import Controller
from sensing.sensing_performance import SensingParameters
from vehicle.state_estimation import Belief, Action
from vehicle.vehicle import VehicleStats, VehicleState


class BasicController(Controller):

    def __init__(self, prob_threshold: Decimal, vs: VehicleStats, ds: Decimal,
                 d_stop: Decimal, cont_sampl_time_s: Decimal, p_a_max: Decimal):
        self.prob_threshold = prob_threshold
        self.vs = vs
        self.ds = ds
        self.d_stop = d_stop
        self.cont_sampl_time_s = cont_sampl_time_s
        self.p_a_max = p_a_max
        assert cont_sampl_time_s > 0, cont_sampl_time_s

    def get_critical_distance(self, v: Decimal) -> Decimal:
        d_critical = v ** 2 / (2 * abs(self.vs.a_min)) + self.d_stop

        return d_critical

    def get_action(self, vstate: VehicleState, alpha: List[Decimal]) -> Action:
        d_critical = self.get_critical_distance(vstate.v)
        i = round((d_critical / self.ds))
        # psi = [a*self.ds for a in alpha[:i]]
        psi = alpha[i]*d_critical
        p_obstacle_less_than_critical = psi
        print(f'p_obstacle_less_than_critical: {p_obstacle_less_than_critical}')
        print(f'v : {vstate.v}')
        print(f'x : {vstate.x}')
        if float(p_obstacle_less_than_critical) > float(self.prob_threshold):
            a = self.vs.a_min
        elif vstate.v < self.vs.v_nominal:
            # Choose scaling factors
            a = self.vs.a_max*self.p_a_max
        else:
            a = 0

        return Action(a)

