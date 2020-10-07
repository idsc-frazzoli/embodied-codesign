import random
from dataclasses import dataclass
from decimal import Decimal
from typing import List

import numpy as np
import scipy.stats

from sensing.sensing_performance import SensingParameters, SensingPerformance
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


def compute_observations(sp: SensingPerformance, sparam: SensingParameters, prior: Prior,
                         state: State) -> Observations:
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
                d_detect = -1 * d_detect

            detection = Detection(Decimal(d_detect), stdev)
            detections.append(detection)

    for i in range(sp.n):
        # distance
        d = sparam.list_of_ds[i]
        p_false_positives = sp.fp[i] * sp.ds
        if toss_biased_coin(p_false_positives):
            stdev = sp.lsd_at(d)

            d_detect = random.gauss(float(d), float(stdev))

            if d_detect < 0:
                d_detect = -1 * d_detect

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
    norm = Decimal(1 / sum(po1))
    po1 = norm * np.array(po1)

    return Belief(list(po1))


def observation_model(b0: Belief, obs: Observations, list_of_ds: List[Decimal], sp: SensingPerformance) -> Belief:
    like = np.zeros(len(list_of_ds))

    ds_list_f = np.array([float(ds) for ds in list_of_ds])
    for detection in obs.detections:
        gauss_dist = scipy.stats.norm(float(detection.d_mean), float(detection.d_std))
        prob = gauss_dist.pdf(ds_list_f)
        like += prob

    like += np.asarray(sp.fn, dtype=float)
    po1 = np.asarray(b0.po, dtype=float) * like

    norm = 1 / np.sum(po1)
    po1 = norm * po1

    po1 = [Decimal(p) for p in po1]

    return Belief(list(po1))
