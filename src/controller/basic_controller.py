from decimal import Decimal

from controller.controller import Controller
from sensing.sensing_performance import SensingParameters
from vehicle.state_estimation import Belief, Action
from vehicle.vehicle import VehicleStats, VehicleState


class BasicController(Controller):

    def __init__(self, prob_threshold: Decimal, vs: VehicleStats, sp: SensingParameters,
                 d_stop: Decimal, t_react: Decimal):
        self.prob_threshold = prob_threshold
        self.vs = vs
        self.sp = sp
        self.d_stop = d_stop
        self.t_react = t_react

    def get_action(self, vstate: VehicleState, belief: Belief) -> Action:
        d_critical = vstate.v ** 2 / (2*abs(self.vs.a_min)) + self.t_react*vstate.v + self.d_stop
        i = int(d_critical / self.sp.ds)
        p_obstacle_less_than_critical = sum(belief.po[:i])
        if float(p_obstacle_less_than_critical) > float(self.prob_threshold):
            a = self.vs.a_min
        elif vstate.v < self.vs.v_nominal:
            # Choose scaling factors
            a = self.vs.a_max*Decimal('0.5')
        else:
            a = 0

        return Action(a)
