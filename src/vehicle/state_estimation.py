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


def compute_observations(sp: SensingPerformance, sparam: SensingParameters,
                         state: State) -> Observations:
    """ From the state, compute the detections """

    detections = []
    for o in state.objects:
        if o.d > sparam.max_distance:
            continue

        false_negatives = sp.false_negative_at(o.d)
        p_detect = 1 - false_negatives * sparam.ds
        if toss_biased_coin(p_detect):
            prob_acc = sp.prob_acc_at(o.d)

            d_detect = np.random.uniform(float(prob_acc.a), float(prob_acc.b))

            if d_detect < 0:
                d_detect = -1 * d_detect

            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    for i in range(sp.n):
        # distance
        d = sparam.list_of_ds[i]
        p_false_positives = sp.fp[i] * sp.ds
        if toss_biased_coin(p_false_positives):
            d_detect = np.random.uniform(float(d), float(d + sp.ds))

            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    return Observations(detections)


@dataclass
class Belief:
    """ probability of obstacle at distance d """
    po: List[Decimal]  # of length n


def prediction_model(b0: Belief, delta_idx: int, prior: Decimal) -> Belief:
    if delta_idx != 0:
        po0 = b0.po[delta_idx:]
        po1_new = [prior for _ in range(delta_idx)]
        po1 = po0 + po1_new
    else:
        po1 = b0.po

    return Belief(po1)


def observation_model(b0: Belief, obs: Observations,
                      sp: SensingPerformance, sens_param: SensingParameters, prior_poisson) -> Belief:
    n = sens_param.n
    fn = np.array([float(sp.fn[n - 1]) if i == n - 1 else 0.5 * float(sp.fn[i] + sp.fn[i + 1]) for i in range(n)])
    fp = np.array([float(sp.fp[n - 1]) if i == n - 1 else 0.5 * float(sp.fp[i] + sp.fp[i + 1]) for i in range(n)])
    prob_accuracy = sp.prob_accuracy
    ones = np.ones(sens_param.n)
    p_k = np.array([float(p) for p in b0.po])

    if not obs.detections:
        po1 = b0.po
    else:
        a_acc = np.array([float(a.a) for a in prob_accuracy])
        b_acc = np.array([float(b.b) for b in prob_accuracy])
        p_acc = np.array([float(p.p) for p in prob_accuracy])
        p_d_k = np.ones(n)
        p_d_k_not = np.ones(n)
        p_flake_star_d_matrix = np.zeros((n, n))
        p_flake_star_b = ones - fn
        p_flake_star_c_matirx = np.zeros((n, n))
        p_flake_star_e_matirx = np.zeros((n, n))
        p_flake_star_c_dot_matirx = np.zeros((n, n))
        p_flake_star_e_dot_matirx = np.zeros((n, n))
        p_flake_star = np.zeros((n, n))
        p_kross_dot = np.zeros((n, n))
        p_kross_dot_f_matrix = np.zeros((n, n))
        p_kross_dot_g = fp
        for i in range(n):
            p_flake_star_d_matrix[:, i] = fp
            p_flake_star_d_matrix[i, i] = 0.0
            p_flake_star_c_matirx[:, i] = p_k
            p_flake_star_c_matirx[i, i] = 1.0
            p_flake_star_e_matirx[:, i] = ones - p_k
            p_flake_star_e_matirx[i, i] = 0.0
            p_flake_star_c_dot_matirx[:, i] = p_k
            p_flake_star_c_dot_matirx[i, i] = 0.0
            p_flake_star_e_dot_matirx[:, i] = ones - p_k
            p_flake_star_e_dot_matirx[i, i] = 1.0
            p_flake_star[:, i] = p_flake_star_b * p_flake_star_c_matirx[:, i] + p_flake_star_d_matrix[:, i] * p_flake_star_e_matirx[:, i]
            p_kross_dot_f_matrix[:, i] = p_flake_star_b
            p_kross_dot_f_matrix[i, i] = 0.0
            p_kross_dot[:, i] = p_kross_dot_f_matrix[:, i] * p_flake_star_c_dot_matirx[:, i] + p_kross_dot_g * p_flake_star_e_dot_matirx[:, i]

        for det in obs.detections:
            d = float(det.d_mean)
            p_flake_a = np.array([0.0 if d < a_acc[i] or d > b_acc[i] else p_acc[i] for i in range(n)])
            # prod = p_flake_a * p_flake_star[:, 0]
            # print(prod)
            # p_flake_nosum = np.array([p_flake_a * p_flake_star[:, k] for k in range(n)])
            # p_flake = np.array([np.sum(p_flake_nosum[:, k]) for k in range(n)])
            p_flake = np.array([np.sum(p_flake_a * p_flake_star[:, k]) for k in range(n)])
            p_d_k *= p_flake
            p_kross = np.array([np.sum(p_flake_a * p_kross_dot[:, k]) for k in range(n)])
            p_d_k_not *= p_kross

        p_d = p_d_k * p_k + p_d_k_not * (ones - p_k)
        p_p_k_d = p_d_k * p_k / p_d
        po1 = [Decimal(p) for p in p_p_k_d]


    return Belief(po1)

# def observation_model(b0: Belief, obs: Observations,
#                       sp: SensingPerformance, sens_param: SensingParameters) -> Belief:
#     ds = float(sens_param.ds)
#     list_d = np.array([float(d) + ds for d in sens_param.list_of_ds])
#     n = sens_param.n
#     fn = np.array([float(p) for p in sp.fn])
#     fp_ds = np.array([float(p)*ds for p in sp.fp])
#     prob_accuracy = sp.prob_accuracy
#     ones = np.ones(sens_param.n)
#     p_k = np.array([float(p) for p in b0.po])
#
#     if not obs.detections:
#         p_nu_k = fn
#         p_nu_k_not = ones - fp_ds
#         p_nu = p_nu_k * p_k + p_nu_k_not * (ones - p_k)
#
#         p_p_k_nu = p_nu_k * p_k / p_nu
#         po1 = [Decimal(p) for p in p_p_k_nu]
#     else:
#         a_acc = np.array([float(a.a) for a in prob_accuracy])
#         b_acc = np.array([float(b.b) for b in prob_accuracy])
#         p_acc = np.array([float(p.p) for p in prob_accuracy])
#         p_d_k = np.zeros(n)
#         p_d_k_not = np.zeros(n)
#         for det in obs.detections:
#             d = float(det.d_mean)
#             p_acc_dd_dd = np.array([0.0 if d < a_acc[i] or d > b_acc[i] else p_acc[i] for i in range(n)])
#             p_d_k_0 = (ones - fn) * p_acc_dd_dd + fp_ds
#             p_d_k_1 = np.array([ p_d_k_0[i] + np.sum((ones - fn) * p_acc_dd_dd * p_k) -
#                                  ((1-fn[i]) * p_acc_dd_dd[i] * p_k[i]) for i in range(n)])
#             p_d_k += p_d_k_1
#             p_d_k_not += fp_ds + np.sum((ones - fn) * p_acc_dd_dd * p_k)
#
#         p_d = p_d_k * p_k + p_d_k_not * (ones - p_k)
#
#         p_p_k_d = p_d_k * p_k / p_d
#         po1 = [Decimal(p) for p in p_p_k_d]
#
#     return Belief(po1)

# def observation_model(b0: Belief, obs: Observations,
#                       sp: SensingPerformance, sens_param: SensingParameters) -> Belief:
#     ds = float(sens_param.ds)
#     list_d = np.array([float(d) + ds for d in sens_param.list_of_ds])
#     n = sens_param.n
#     fn_ds = np.array([float(p)*ds for p in sp.fn])
#     fp_ds = np.array([float(p)*ds for p in sp.fp])
#     prob_accuracy = sp.prob_accuracy
#     ones = np.ones(sens_param.n)
#     p_k = np.array([float(p) for p in b0.po])
#
#     if not obs.detections:
#         p_nu_k = fn_ds
#         p_nu_k_not = ones - fp_ds
#         p_nu = p_nu_k * p_k + p_nu_k_not * (ones - p_k)
#
#         p_p_k_nu = p_nu_k * p_k / p_nu
#         po1 = [Decimal(p) for p in p_p_k_nu]
#     else:
#         a_acc = np.array([float(a.a) for a in prob_accuracy])
#         b_acc = np.array([float(b.b) for b in prob_accuracy])
#         p_acc = np.array([float(p.p) for p in prob_accuracy])
#         p_d_k = np.zeros(n)
#         p_d_k_not = np.zeros(n)
#         for det in obs.detections:
#             d = float(det.d_mean)
#             p_acc_dd_dd = np.array([0.0 if d < a_acc[i] or d > b_acc[i] else p_acc[i]*abs(list_d[i] + 0.5*ds - d) for i in range(n)])
#             p_d_k += (ones - fn_ds) * p_acc_dd_dd + fp_ds + np.sum((ones - fn_ds) * p_acc_dd_dd * p_k)
#             p_d_k_not += fp_ds + np.sum((ones - fn_ds) * p_acc_dd_dd * p_k)
#
#         p_d = p_d_k * p_k + p_d_k_not * (ones - p_k)
#
#         p_p_k_d = p_d_k * p_k / p_d
#         po1 = [Decimal(p) for p in p_p_k_d]
#
#     return Belief(po1)
