import math
from decimal import Decimal
import random
from typing import List

import numpy as np
from dataclasses import dataclass

from sensing.sensing_performance import SensingPerformance, SensingParameters
from vehicle.vehicle import Detection, State, VehicleState

@dataclass
class Action:
    accel: Decimal  # velocity

@dataclass
class Prior:
    density: Decimal  # Density of obstacles (1/m) - intensity of poisson process


@dataclass
class Observations:
    detections: List[Detection]


def toss_biased_coin(p_success: Decimal) -> bool:
    return random.uniform(0, 1) < p_success


def compute_observations(sp: SensingPerformance, sparam: SensingParameters, prior: Prior, state: State) -> Observations:
    """ From the state, compute the detections """

    # for each object, see if we detect it
    detections = []
    for o in state.objects:
        # object at distance o.d
        # do we detect it?
        # depends on the false negatives and the range of the sensor
        if o.d > sparam.max_distance:
            continue

        false_negatives = sp.false_negative_at(o.d)
        p_detect = 1 - false_negatives
        if toss_biased_coin(p_detect):
            # great we see it
            # with what variance?
            stdev = sp.lsd_at(o.d)

            d_detect = random.gauss(float(o.d), float(stdev))

            if d_detect < 0:
                d_detect = -1*d_detect

            detection = Detection(Decimal(d_detect), stdev)
            detections.append(detection)

    density = prior.density * sparam.max_distance
    n_objects = np.random.poisson(lam=float(density))
    for o in range(0, n_objects):
        d = Decimal(round(random.uniform(0.0, float(sparam.max_distance)), 1))
        false_positive = sp.false_positive_at(d)
        if toss_biased_coin(false_positive):
            stdev = sp.lsd_at(d)

            d_detect = random.gauss(float(d), float(stdev))

            if d_detect < 0:
                d_detect = -1*d_detect

            detection = Detection(Decimal(d_detect), stdev)
            detections.append(detection)

    # now sample false positives
    # according to a poisson with variable intensity given by sp.fn

    ...  # scipy

    return Observations(detections)


@dataclass
class Belief:
    """ probability of obstacle at distance d """
    po: List[Decimal]  # of length n


def prediction_model(b0: Belief, delta_idx: int, delta: Decimal, prior: Prior) -> Belief:
    # translate everything by v
    # (keep in mind conversion in cells)
    # for the new part, apply prior

<<<<<<< HEAD
    # easy: just translate by integer
    vstate_prev = get_previos_state(s, u, dt)
    delta = s.vstate.x - vstate_prev.x
    delta_idx = int(delta / ds)
=======
    # easy: just transalte by integer
>>>>>>> c9d65a6b6ea3af006f1690f5af20f37b1631903e
    density = prior.density * delta
    pp_delta = density * Decimal(np.exp(-float(density)))
    if delta_idx != 0:
        po_delta = [Decimal(pp_delta / delta_idx) for _ in range(delta_idx)]
    else:
        po_delta = []

    po1 = b0.po[delta_idx:] + po_delta
    norm = Decimal(1/sum(po1))
    po1 = norm * np.array(po1)

    return Belief(list(po1))


def observation_model(b0: Belief, obs: Observations, list_of_ds: List[Decimal], sp: SensingPerformance) -> Belief:
    def likelihood(d: Decimal, fn: Decimal, fp: Decimal) -> Decimal:
        # question: what is the likelihood that there is something at d

        # a: it's the sum of the likelihoods for each detection
        res = 0.0

        for detection in obs.detections:
            res += float(detection.at_distance(d, fn, fp))

        if math.isnan(res):
            raise ValueError("The likelihood is not a number.")

        if res > 1.0:
            res = 1.0

        return Decimal(res)

    like = np.array([likelihood(d, sp.false_negative_at(d), sp.false_positive_at(d))
                                      for d in list_of_ds])
    po1 = np.array(b0.po) * like

    if np.sum(po1) == 0.0:
        po1 = np.array(b0.po)
    else:
        norm = 1 / np.sum(po1)
        po1 = norm * po1

    return Belief(list(po1))
