import math
import os
from decimal import Decimal
from multiprocessing import Pool
import random
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import yaml

from scripts.create_catalogue import simulate_and_write
from sensing.sensing_performance import SensingParameters, SensingPerformance, calc_unit_dist_a_b_prob, ConfLevel, \
    ConfLevelList
from simulator.simulator import SimParameters
from utils.yaml_file_generation import read_results, write_bc_dpc
from vehicle.state_estimation import Inference, compute_observations, Observations, prediction_model, observation_model, \
    Prior
from vehicle.vehicle import Detection, Object, State, VehicleState

if __name__ == '__main__':
    with open('data/input/test_sens_perf.yaml') as file:
        camera_curves = yaml.load(file, Loader=yaml.FullLoader)
    with open('data/input/test_sensr.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    sens = sensors
    sens_curves = camera_curves


    dt = Decimal('0.01')
    ds = Decimal('0.01')
    max_distance = Decimal(sens_curves["max_distance"])
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]

    freq = Decimal(str(sens["frequency"]))
    ts_sens = 1 / freq
    n_ts_sens = round(ts_sens / dt)
    latency_sens = Decimal(str(sens["latency"]))
    n_ts_lat_sens = round(latency_sens / dt)
    n_ts_lat_sens = 1
    n_ts_sens = 1
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, frequency=n_ts_sens * Decimal(0.01), latency=n_ts_lat_sens * Decimal(0.01))



    sens_perf = SensingPerformance(sp=sens_param)
    fn = sens_curves["fn"]
    sens_perf.fn = [Decimal(p) for p in fn]
    fp = sens_curves["fp"]
    sens_perf.fp = [Decimal(p) for p in fp]
    lsd = sens_curves["accuracy"]
    sens_perf.lsd = [Decimal(p) for p in lsd]
    list_prob_acc = [calc_unit_dist_a_b_prob(d=list_of_ds[i], ds=ds, std=sens_perf.lsd[i]) for i in range(n)]
    sens_perf.prob_accuracy = list_prob_acc

    tresh_idx = 0
    for i in range(n):
        idx = int(sens_perf.lsd[i] / ds)
        if idx >= 1:
            tresh_idx = i
            break

    list_cl = []
    for i in range(n):
        cl = 0.95
        sigma = float(sens_perf.lsd[i])
        df = n - 1
        mean = float(list_of_ds[i])
        standard_error = sigma / math.sqrt(n)
        if mean < 0.1:
            cL_level = [0.0, 0.0]
        else:
            cL_level = scipy.stats.t.interval(cl, df, mean, standard_error)
        if i < tresh_idx:
            if math.isnan(cL_level[0]) or math.isnan(cL_level[1]):
                clist_cell = [Decimal(str(0.0)), Decimal(str(0.0))]
            else:
                clist_cell = [Decimal(str(cL_level[0])), Decimal(str(cL_level[1]))]

            confidence_level = ConfLevel(clist_cell)
        else:
            ucl_cell = int(min(n, (cL_level[1]) / float(ds)))
            lcl_cell = int(min(n, max(0.0, (cL_level[0]) / float(ds))))
            if ucl_cell == lcl_cell:
                clist_cell = [int(mean / float(ds))]
            else:
                clist_cell = [Decimal(str(i)) for i in range(lcl_cell, ucl_cell)]
            confidence_level = ConfLevel(clist_cell)

        list_cl.append(confidence_level)

    confidence_level_list = ConfLevelList(list_cl, tresh_idx)
    sens_perf.cl_list = confidence_level_list

    # sens_perf = SensingPerformance(sp=sens_param)
    # fn = sens_curves["fn"]
    # sens_perf.fn = [Decimal(fn[i+10]) for i in range(n)]
    # fp = sens_curves["fp"]
    # sens_perf.fp = [Decimal(fp[i+10]) for i in range(n)]
    # lsd = sens_curves["accuracy"]
    # sens_perf.lsd = [Decimal(lsd[i+10]) for i in range(n)]
    # list_prob_acc = [calc_unit_dist_a_b_prob(d=list_of_ds[i], ds=ds, std=sens_perf.lsd[i]) for i in range(n)]
    # sens_perf.prob_accuracy = list_prob_acc

    density = Decimal(7/(1000))
    prior = Prior(density=density)
    density_belief = density * max_distance
    pp = density_belief * Decimal(np.exp(-float(density_belief)))
    po = [Decimal(pp) for _ in range(n)]
    alpha0 = po
    inference = Inference(alpha=alpha0)

    x = Decimal('0.0')
    x_prev = x
    # detec = []#[Detection(Decimal('20.0'))]
    # observations = Observations(detec)
    objects = [Object(Decimal('20')), Object(Decimal("40.0"))]
    vehstate = VehicleState(x, x, x, x)
    state = State(vehstate, objects)
    # observations = None
    dx = Decimal('0.1')
    while x <= Decimal('500'):

        observations = compute_observations(sens_perf, sens_param, prior, state)

        delta = x - x_prev
        delta_idx = int(delta / ds)
        inference1 = prediction_model(inf=inference, delta=delta, delta_idx=delta_idx, prior=alpha0[0],
                                      list_ds=sens_param.list_of_ds, ds=ds)

        inference = observation_model(inf0=inference1, obs=observations, sens_param=sens_param, sp=sens_perf,
                                      density=alpha0[0])

        print(x)

        if x % Decimal('1.0') == 0:
            plt.plot(list_of_ds, inference.alpha)
            plt.show()

        x_prev = x

        x += dx
        objects[0].d = objects[0].d - dx
        objects[1].d = objects[1].d - dx
        state.objects = objects

        # observations.detections[0].d_mean = observations.detections[0].d_mean - dx + Decimal(random.uniform(-0.1, 0.1))
