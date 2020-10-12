import math
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

@dataclass
class ConfLevel:
    conf_level: List[Decimal]


@dataclass
class ConfLevelList:
    list: List[ConfLevel]
    treshold_idx: int


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

            d_detect = np.random.lognormal(float(o.d), float(stdev))

            if d_detect < 0:
                d_detect = -1 * d_detect

            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    for i in range(sp.n):
        d = sparam.list_of_ds[i]
        p_false_positives = sp.fp[i] * sp.ds
        if toss_biased_coin(p_false_positives):
            if d == 0:
                d_detect = 0
            else:
                d_detect = np.random.uniform(float(d - sp.ds), float(d))
            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    return Observations(detections)


@dataclass
class Inference:
    """ density of obstacles up to distance d 1/m """
    alpha: List[Decimal]  # of length n


def prediction_model(inf: Inference, delta_idx: int, delta: Decimal,
                     prior: Prior, list_ds: List[Decimal], ds: Decimal) -> Inference:
    if delta_idx != 0:
        alpha0 = inf.alpha[delta_idx:]
        alpha0 = [0.0 if i == 0 else inf.alpha[i+delta_idx] * (1 + delta / list_ds[i]) for i in range(len(alpha0))]
        alpha_new = [(alpha0[-1]*list_ds[len(alpha0)-1] + prior.density*ds)/list_ds[len(alpha0)+i]
                     for i in range(delta_idx)]
        alpha = alpha0 + alpha_new
    else:
        alpha = inf.alpha

    return Inference(alpha)


def observation_model(inf0: Inference, obs: Observations, cl_list: ConfLevelList, sens_param: SensingParameters, sp: SensingPerformance) -> Inference:
    def integrate(conf_level: List[Decimal], x, sens_param: SensingParameters):
        ucl_cell = int(min(sens_param.n, conf_level[1] / sens_param.ds))
        lcl_cell = int(max(0.0, conf_level[0] / sens_param.ds))
        x_ds = x*sens_param.ds
        alpha = 1 / ((ucl_cell + lcl_cell) * sens_param.ds) * np.sum(x_ds)

        return alpha
    ones = np.ones(sens_param.n)
    fp_ds = np.array(sp.fp)*sens_param.ds
    alpha_nodet = np.array(inf0.alpha)
    x_nodet_1 = np.array(sp.fn) * alpha_nodet * sens_param.ds
    x_nodet_2 = (ones - fp_ds) * (ones - alpha_nodet*sens_param.ds)
    x_nodet = x_nodet_1 + x_nodet_2

    if obs.detections:
        detections_cell = [min(sens_param.n, int(det.d_mean / sens_param.ds)) for det in obs.detections]
        alpha_det = np.array([inf0.alpha[cell] for cell in detections_cell])
        ones_det = np.ones(len(detections_cell))
        fp_det = [sp.fp[cell] for cell in detections_cell]
        fp_det_ds = np.array(fp_det) * sens_param.ds
        fn_det = np.array([sp.fn[cell] for cell in detections_cell])
        x_det_1 = (ones_det - fn_det) * alpha_det * sens_param.ds
        x_det_2 = fp_det_ds * (ones_det - alpha_det * sens_param.ds)
        x_det = x_det_1 + x_det_2

        i = 0
        for cell in detections_cell:
            x_nodet[cell] = x_det[i]
            i += 1

    x = x_nodet
    idx_tresh = cl_list.treshold_idx
    list_cl = cl_list.list

    for i in range(idx_tresh, len(x)):
        x_sum = np.array([x[int(k)]*sens_param.ds for k in list_cl[i].conf_level])
        sum = np.sum(x_sum)
        x[i] = 1 / (len(list_cl[i].conf_level) * sens_param.ds) * sum

    for i in range(idx_tresh):
        beta = list_cl[i].conf_level[1] - list_cl[i].conf_level[0]
        f_big = 0.05
        if sp.fp[i]*sens_param.ds > f_big or sp.fn[i] > f_big:
            gamma = 0
        else:
            gamma = random.uniform(0, 1)

        x[i] = (alpha_nodet[i] * beta + x[i]) / (beta + gamma)

    alpha = x

    return Inference(list(alpha))
