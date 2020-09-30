# discretize space
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from decimal import Decimal
from typing import List, Optional
import numpy as np

ds = Decimal(0.1)  # meters
max_dist = Decimal(200)
n = int(max_dist / ds)


list_of_ds = [ ds*i for i in range(n)]

@dataclass
class SensingPerformance:
    # false positives as a function of distance
    # array of length n
    fp: List[Decimal]  # of length n
    # false negatives
    fn: List[Decimal]  # of length n
    # localization standard deviation
    lsd: List[Decimal]  # in meters

    def false_negative_at(self, d) -> Decimal:
        i = d/ds
        # todo: bound checking
        return self.fn[i]


    def false_positive_at(self, d) -> Decimal:
        pass

    def lsd_at(self, d: Decimal) -> Decimal:
        pass


@dataclass
class Object:
    # distance relative to car
    d: Decimal

@dataclass
class VehicleState:
    x: Decimal
    v: Decimal

@dataclass
class State:
    # vehicle state

    vstate: VehicleState
    # list of obstacles with distance
    objects: List[Object]


@dataclass
class Detection:
    d_mean: Decimal
    d_std: Decimal

    def at_distance(self, x) -> Decimal:
        """ Returns the likeilhood of something at x given this detection"""
        # return gaussian distribution density at  (x-d_mean)/d_std


@dataclass
class Observations:
    detections: List[Detection]

def toss_biased_coin(p_success: Decimal) -> bool:
    return  random.uniform(0, 1) < p_success


def compute_observations(sp: SensingPerformance, state: State) -> Observations:
    """ From the state, compute the detections """

    # for each object, see if we detect it
    detections = []
    for o in state.objects:
        # object at distance o.d
        # do we detect it?
        # depends on the false negatives
        false_negatives = sp.false_negative_at(o.d)
        # Probability of detecting an obstacle given that we have an obstacle (true positive)
        p_detect = 1 - false_negatives
        if toss_biased_coin(p_detect):
            # great we see it
            # with what variance?
            stdev = sp.lsd_at(o.d)

            d_detect = random.gauss(float(o.d), float(stdev))

            detection = Detection(Decimal(d_detect), stdev)
            detections.append(detection)

    # now sample false positives
    # according to a poisson with variable intensity given by sp.fn

    ... # scipy

    return Observations(detections)

@dataclass
class Belief:
    """ probability of obstacle at distance d """
    po: List[Decimal]  # of length n

@dataclass
class Action:
    accel: Decimal # velocity


@dataclass
class Prior:
    density: Decimal # Density of obstacles (1/m) - intensity of poisson process



def prediction_model(b0: Belief, u: Action, prior: Prior) -> Belief:
    # translate everything by v
    # (keep in mind conversion in cells)
    # for the new part, apply prior

    # easy: just transalte by integer
    delta = int(u.v / ds)

    po1 = b0.po[delta:] + [prior.density]* delta

    return Belief(po1)

def observation_model(b0: Belief, obs: Observations) -> Belief:

    def likelihood(d: Decimal) -> Decimal:
        # question: what is the likelihood that there is something at d

        # a: it's the sum of the likelihoods for each detection
        res = 0.0

        for detection in obs.detections:
            res += detection.at_distance(d)


        return Decimal(res)

    po1 = np.array(b0.po) * np.array([ likelihood(d) for d in list_of_ds])

    return Belief(list(po1))

@dataclass
class PerformanceMetrics:
    p_collision: Decimal
    discomfort: Decimal


class Controller(ABC):
    @abstractmethod
    def get_action(self, belief: Belief) -> Action:
        ...


def update_state(s: State, action: Action) -> State:
    pass

@dataclass
class SimParameters:
    nsims: int
    road_length: Decimal
    prior: Prior
    controller: Controller
    sens_perf: SensingPerformance
    dt: Decimal

def simulate(sp: SimParameters) -> PerformanceMetrics:
    """  nsims: number of simulations"""

    for i in range(sp.nsims):

        pm = simulate_one(sp)
        # todo: average

@dataclass
class CollisionStats:
    momentum: Decimal


@dataclass
class OneSimPerformanceMetrics:
    collided: Optional[CollisionStats]
    average_velocity: Decimal
    control_effort: Decimal

def collided(s: State) -> CollisionStats:
    # check if collided
    pass

def simulate_one(sp: SimParameters) -> OneSimPerformanceMetrics:

    # first, create the state
    # static obstacles

    density = sp.prior.density
    objects = ... # sample from poisson with intensity sp.prior.density

    vstate0=  VehicleState(Decimal(0), Decimal(0))

    state = State(vstate0, objects)
    po = [density/ds for _ in range(n)]
    belief = Belief(po)
    action = Action(Decimal(0))

    control_effort= 0
    t = 0
    while True:
        t += sp.dt
        observations = compute_observations(sp.sens_perf, state)
        belief1 = prediction_model(belief, action, sp.prior)
        belief = observation_model(belief1, observations)

        action = sp.controller.get_action(belief)
        state = update_state(state, action)

        control_effort += ...

        c = collided(state)

        if c is not None:
            avg_control_effort = control_effort / t
            average_velocity = ...
            return OneSimPerformanceMetrics(c, average_velocity, Decimal(avg_control_effort))

    avg_control_effort = control_effort / t
    average_velocity = ...
    return OneSimPerformanceMetrics(None, average_velocity, Decimal(avg_control_effort))

@dataclass
class VehicleStats:
    a_min: Decimal
    a_max: Decimal

@dataclass
class VehicleStats:
    a_min: Decimal
    a_max: Decimal
class BasicController(Controller):

    def __init__(self, d_critical: Decimal, prob_threshold: Decimal,
                 v_nominal: Decimal, vs):
        self.d_critical = d_critical
        self.prob_threshold = prob_threshold
        self.v_nominal = v_nominal
        self.vs=vs
        ...

    def get_action(self, vstate: VehicleState, belief: Belief) -> Action:
        p_obstacle_less_than_critical = ...

        if p_obstacle_less_than_critical > self.prob_threshold:
            a = self.vs.a_min
        elif vstate.v < self.v_nominal:
            a = self.vs.a_max
        else:
            a = 0

        return Action(a)

def main():

    vs = VehicleStats(a_min=..., a_max=...)

