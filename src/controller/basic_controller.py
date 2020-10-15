from decimal import Decimal

import numpy as np

from controller.controller import Controller
from sensing.sensing_performance import SensingParameters
from vehicle.state_estimation import Action, Inference
from vehicle.vehicle import VehicleStats, VehicleState


class BasicController(Controller):

    def __init__(self, prob_threshold: Decimal, vs: VehicleStats,
                 d_stop: Decimal, t_react: Decimal, frequency: Decimal):
        self.prob_threshold = prob_threshold
        self.prob_threshold = Decimal('0.3')
        self.vs = vs
        self.d_stop = d_stop
        self.d_stop = Decimal('4.0')
        self.t_react = t_react
        self.frequency = frequency

    def get_action(self, vstate: VehicleState, inference: Inference, ds: Decimal) -> Action:
        d_critical = vstate.v ** 2 / (2*abs(self.vs.a_min)) + self.t_react*vstate.v + self.d_stop
        i = max(1, min(len(inference.alpha) - 1, int(d_critical / ds)))
        alpha_ds = np.array([float(a) for a in inference.alpha])
        sum_alpha = np.sum(alpha_ds)
        print('this is the sum', sum_alpha)
        alpha_ds = alpha_ds / sum_alpha
        # bl = int(Decimal(0.2)/ds)
        # bu = bl
        # if i - bl < 0:
        #     bl = 0
        # sum_inf = inference.alpha[(i-bl):(i+bu)]
        p_obstacle_less_than_critical = sum(alpha_ds[:i])
        #print("kritical", p_obstacle_less_than_critical)
        if float(p_obstacle_less_than_critical) > float(self.prob_threshold):
            a = self.vs.a_min
        elif vstate.v < self.vs.v_nominal:
            # Choose scaling factors
            a = self.vs.a_max*Decimal('1')
        else:
            a = 0

        return Action(a)
