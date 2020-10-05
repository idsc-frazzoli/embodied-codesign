import math
from decimal import Decimal
import random
from typing import List

import numpy as np
import scipy.stats
from dataclasses import dataclass

from sensing.sensing_performance import SensingPerformance, SensingParameters
from vehicle.vehicle import Detection, State

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

    detections = []
    for o in state.objects:
        if o.d > sparam.max_distance:
            continue

        false_negatives = sp.false_negative_at(o.d)
        p_detect = 1 - false_negatives
        if toss_biased_coin(p_detect):
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


    return Observations(detections)


@dataclass
class Belief:
    """ probability of obstacle at distance d """
    po: List[Decimal]  # of length n


def prediction_model(b0: Belief, delta_idx: int, delta: Decimal, prior: Prior) -> Belief:
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


def observation_model(b0: Belief, obs: Observations, list_of_ds: List[Decimal]) -> Belief:
    like = [0]*len(list_of_ds)
    ds_list_f = [float(ds) for ds in list_of_ds]
    for detection in obs.detections:
        gauss_dist = scipy.stats.norm(float(detection.d_mean), float(detection.d_std))
        prob = gauss_dist.pdf(ds_list_f)
        like += prob
    like = np.array([Decimal(l) for l in like])
    po1 = np.array(b0.po) * like

    if np.sum(po1) == 0.0:
        po1 = np.array(b0.po)
    else:
        norm = 1 / np.sum(po1)
        po1 = norm * po1

    return Belief(list(po1))
