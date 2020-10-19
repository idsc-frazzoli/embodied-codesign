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
<<<<<<< HEAD
        d_critical = vstate.v ** 2 / (2*abs(self.vs.a_min)) + self.t_react*vstate.v + self.d_stop
        i = int(np.ceil(d_critical / self.sp.ds))
=======
        d_critical = self.get_critical_distance(vstate.v)
        i = int(np.ceil(d_critical / self.ds))
>>>>>>> 70c8a8ff601a5d5f16175ef9b4bcd646e7da899d
        p_obstacle_less_than_critical = sum(belief.po[:i])
        if float(p_obstacle_less_than_critical) > float(self.prob_threshold):
            a = self.vs.a_min
        # GZ: Why do we need this specification? Can't we find a factor for all decelerations?
        elif vstate.v < self.vs.v_nominal:
            # Choose scaling factors
            a = self.vs.a_max*Decimal('0.5')
        else:
            a = 0

        return Action(a)

