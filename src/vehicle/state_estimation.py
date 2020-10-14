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
            prob_acc = sp.prob_acc_at(o.d)

            d_detect = np.random.uniform(float(prob_acc.a), float(prob_acc.b))

            print("Detection:", d_detect)

            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    for i in range(sp.n):
        d = sparam.list_of_ds[i]
        p_false_positives = sp.fp[i] * sp.ds
        if toss_biased_coin(p_false_positives):
            if d == 0:
                d_detect = 0
            else:
                d_detect = np.random.uniform(float(d), float(d + sp.ds))
            detection = Detection(Decimal(d_detect))
            detections.append(detection)

    return Observations(detections)


@dataclass
class Inference:
    """ density of obstacles up to distance d 1/m """
    alpha: List[Decimal]  # of length n


def prediction_model(inf: Inference, delta_idx: int, delta: Decimal,
                     prior, list_ds: List[Decimal], ds: Decimal) -> Inference:
    if delta_idx != 0:
        alpha0 = inf.alpha[delta_idx:]
        # alpha0 = [Decimal('0.0') if i == 0 else inf.alpha[i + delta_idx] * (1 + delta / list_ds[i]) for i in
        #           range(len(alpha0))]
        # alpha_new = [(alpha0[-1] * list_ds[len(alpha0) - 1] + prior.density * ds) / list_ds[len(alpha0) + i]
        #              for i in range(delta_idx)]
        alpha_new = [prior for _ in range(delta_idx)]
        alpha = alpha0 + alpha_new
    else:
        alpha = inf.alpha

    return Inference(alpha)


def observation_model(inf0: Inference, obs: Observations, sens_param: SensingParameters,
                      sp: SensingPerformance, density: Decimal) -> Inference:
    ds = float(sens_param.ds)
    list_d = np.array([float(d)+ds for d in sens_param.list_of_ds])
    n = sens_param.n
    fn = np.array([float(p) for p in sp.fn])
    fp = np.array([float(p) for p in sp.fp])
    fp_ds = np.array([float(p) * ds for p in sp.fp])
    fp_ds = fp
    prob_accuracy = sp.prob_accuracy
    ones = np.ones(sens_param.n)
    p_p = np.array([float(a) for a in inf0.alpha])
    if np.isnan(p_p).any():
        print(np.argwhere(np.isnan(p_p)))
        print("NAN")
        raise ValueError("NaN Value, invalid input")

    if not obs.detections:
        p_d_nan_giv_p = fn + float(density)
        p_d_nan_giv_n = (ones - fp_ds)
        p_z_nan = (p_d_nan_giv_p*p_p + p_d_nan_giv_n*(ones - p_p))
        if np.any(p_z_nan == 0.0):
            print(np.argwhere(p_z_nan == 0.0))
            print('zero')
        p_p_z = (p_d_nan_giv_p*p_p) / p_z_nan

        if np.isnan(p_p).any():
            print(np.argwhere(np.isnan(p_p)))
            print("NAN")
            raise ValueError("NaN Value, invalid input")


    else:
        a_acc = np.array([float(a.a) for a in prob_accuracy])
        b_acc = np.array([float(b.b) for b in prob_accuracy])
        p_acc = np.array([float(p.p) for p in prob_accuracy])
        p_z_tot = np.zeros(n)
        p_z_p_tot = np.zeros(n)
        for det in obs.detections:
            det_idx = int(min(n-1, det.d_mean / sens_param.ds))
            std = sp.lsd[det_idx]
            gauss_dist = scipy.stats.norm(float(det.d_mean), float(std))
            x = gauss_dist.pdf(list_d)
            # x = np.array([0.0 if float(det.d_mean) > b_acc[i] or float(det.d_mean) < a_acc[i] or i == det_idx else p_acc[i] for i in range(n)])
            p_z_p = x + (ones-fn) + fp
            y = np.array([(1-fn[i])*x[i]*p_p[i] for i in range(n)])
            y = np.sum(y)
            p_z_n = fp + y
            p_z = p_z_p*p_p + p_z_n*(ones - p_p)
            p_z_tot += p_z
            p_z_p_tot += p_z_p
            # fp_ds_det = fp_ds[det_idx]*(1 - p_p[det_idx])
            # p_acc_dn_dn = 0.0 if det.d_mean > b_acc[det_idx] or det.d_mean < a_acc[det_idx] else p_acc[det_idx]
            # tp_det = (1-fn[det_idx])*p_acc_dn_dn*p_p[det_idx]
            # y = np.array([0.0 if float(det.d_mean) > b_acc[i] or float(det.d_mean) < a_acc[i] or det_idx == i else (1-fn[i])*p_acc[i]*p_p[i] for i in range(n)])
            # y = np.sum(y)
            # p_d_n_giv_p = fn + (ones - fn)*x + fp_ds_det + tp_det + y
            # p_d_n_giv_p[det_idx] = 0.0
            # p_d_n_giv_n = (ones - fp_ds) + fp_ds_det + y
            # p_d_n_giv_n[det_idx] = 0.0
            # p_d_d_giv_p = (1 - fn[det_idx])*p_acc[det_idx] + fp_ds[det_idx] + y
            # # p_d_d_giv_p = (1 - fn[det_idx]) + fp_ds[det_idx]
            # p_d_d_giv_n = fp_ds[det_idx] + y
            # p_z_dd = np.zeros(n)
            # p_z_dd[det_idx] = p_d_d_giv_p*p_p[det_idx] + p_d_d_giv_n*(1 - p_p[det_idx])
            # p_z_p = p_d_n_giv_p
            # p_z_p[det_idx] = p_d_d_giv_p
            # p_z_p_tot = p_z_p + p_z_p_tot
            # p_z = p_z_dd + p_d_n_giv_p * p_p + p_d_n_giv_n * (ones - p_p)
            # p_z_tot = p_z + p_z_tot

        # if np.any(p_z_tot == 0.0):
        #     print(np.argwhere(p_z_tot == 0.0))
        #     print('zero')
        #     raise ValueError("Zero Value, invalid input")
        # if np.isnan(p_z_tot).any():
        #     print(np.argwhere(np.isnan(p_z_tot)))
        #     print("NAN")
        #     raise ValueError("NaN Value, invalid input")
        #
        # if np.isnan(p_z_p_tot).any():
        #     print(np.argwhere(np.isnan(p_z_p_tot)))
        #     print("NAN")
        #     raise ValueError("NaN Value, invalid input")
        p_p_z = p_z_p_tot*p_p / p_z_tot
        if np.isnan(p_p_z).any():
            print(len(np.argwhere(np.isnan(p_p_z))))
            print("NAN")

            if np.any(p_z_tot == 0.0):
                print(len(np.argwhere(p_z_tot == 0.0)))
                print('zero')

            raise ValueError("NaN Value, invalid input")

    # cl_list = sp.cl_list
    # idx_tresh = cl_list.treshold_idx
    # list_cl = cl_list.list
    #
    # for i in range(idx_tresh, len(p_p_z)):
    #     x_sum = np.array([p_p_z[int(k)] for k in list_cl[i].conf_level])
    #     sum_x = np.sum(x_sum)
    #     p_p_z[i] = 1 / (len(list_cl[i].conf_level)) * sum_x
    alpha = [Decimal(str(a)) for a in p_p_z]

    return Inference(alpha)




    # lamb = float(density)
    # fn = np.array([float(p) for p in sp.fn])
    # ones = np.ones(sens_param.n)
    # fp_ds = np.array([float(p) * ds for p in sp.fp])
    # alpha0 = np.array([float(a) for a in inf0.alpha])
    # x_nodet_1 = fn * alpha0 * ds
    # x_nodet_2 = (ones - fp_ds) * (ones - alpha0 * ds)*0
    # x_nodet = x_nodet_1 + x_nodet_2
    #
    # if obs is not None:
    #     detections_cell = [min(sens_param.n-1, int(det.d_mean / sens_param.ds)) for det in obs.detections]
    #     alpha_det = np.array([float(inf0.alpha[cell]) for cell in detections_cell])
    #     ones_det = np.ones(len(detections_cell))
    #     fp_det_ds = np.array([fp_ds[cell] for cell in detections_cell])
    #     fn_det = np.array([fn[cell] for cell in detections_cell])
    #     x_det_1 = (ones_det - fn_det) * alpha_det * ds
    #     x_det_2 = fp_det_ds * (ones_det - alpha_det * ds)
    #     x_det = x_det_1 + x_det_2
    #
    #     i = 0
    #     for cell in detections_cell:
    #         x_nodet[cell] = x_det[i]
    #         i += 1

    # x = [0.0 if idx == 0 else np.sum(x_nodet[:idx]) / float(sens_param.list_of_ds[idx]) for idx in range(sens_param.n)]
    # x = x_nodet

    # cl_list = sp.cl_list
    # idx_tresh = cl_list.treshold_idx
    # list_cl = cl_list.list
    #
    # for i in range(idx_tresh, len(x)):
    #     x_sum = np.array([x[int(k)] for k in list_cl[i].conf_level])
    #     sum_x = np.sum(x_sum)
    #     x[i] = 1 / (len(list_cl[i].conf_level)) * sum_x

    # for i in range(idx_tresh):
    #     beta = float(list_cl[i].conf_level[1] - list_cl[i].conf_level[0])
    #     f_big = 0.05
    #     if fp_ds[i] > f_big or fn[i] > f_big:
    #         gamma = 0
    #     else:
    #         gamma = random.uniform(0, 0.01)
    #
    #     x[i] = (alpha_nodet[i] * beta + x[i]) / (beta + gamma)

    # norm = 1/np.sum(x)
    # x = norm * x

