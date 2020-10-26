import math
import os
import random
from dataclasses import dataclass
from decimal import Decimal
from typing import Dict

import numpy as np
import yaml
import scipy.stats

from controller.basic_controller import BasicController
from controller.controller import Action, Controller
from sensing.sensing_performance import SensingParameters, SensingPerformance, ConfLevel, ConfLevelList, \
    calc_unit_dist_a_b_prob
from simulator.create_animation import create_animation
from simulator.performance import CollisionStats, OneSimPerformanceMetrics, PerformanceMetrics, Statistics
from vehicle.state_estimation import compute_observations, observation_model, prediction_model, Prior, Inference
from vehicle.vehicle import DelayedStates, Object, State, VehicleState, VehicleStats

from . import logger


def update_state(s: State, action: Action, dt: Decimal) -> State:
    x = Decimal('0.5') * action.accel * dt ** 2 + s.vstate.v * dt + s.vstate.x
    if (x - s.vstate.x) < 0:
        x = s.vstate.x  # no backward driving

    v = action.accel * dt + s.vstate.v
    if v < 0:
        v = Decimal('0.0')  # no backwards driving

    for i in range(len(s.objects)):
        s.objects[i].d = s.objects[i].d - x + s.vstate.x

    vstate = VehicleState(x=x, v=v, x_prev=s.vstate.x, v_prev=s.vstate.v)

    return State(vstate, s.objects)


@dataclass
class SimParameters:
    nsims: int
    road_length: Decimal
    prior: Prior
    controller: Controller
    sens_perf: SensingPerformance
    dt: Decimal
    sens_param: SensingParameters
    vs: VehicleStats
    seed: int
    do_animation: bool

    def __init__(self, nsims: int, road_length: Decimal, dt: Decimal, seed: int, do_animation: bool) -> None:
        self.nsims = nsims
        self.road_length = road_length
        self.dt = dt
        self.seed = seed
        self.do_animation = do_animation


def simulate(sp: SimParameters, dyn_perf: Dict, sens: Dict, sens_curves: Dict, s: int,
             env: Dict, cont: Dict, experiment_key: str) -> PerformanceMetrics:
    discomfort = np.zeros(sp.nsims)
    average_velocity = np.zeros(sp.nsims)
    average_collision_momentum = np.zeros(sp.nsims)
    collision = np.zeros(sp.nsims)

    ds = Decimal(sens_curves["ds"])
    max_distance = Decimal(sens_curves["max_distance"])
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    freq = Decimal(str(sens["frequency"]))
    ts_sens = 1 / freq
    n_ts_sens = round(ts_sens / sp.dt)
    latency_sens = Decimal(str(sens["latency"]))
    n_ts_lat_sens = round(latency_sens / sp.dt)
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, frequency=n_ts_sens * sp.dt, latency=n_ts_lat_sens * sp.dt)

    vs = VehicleStats(a_min=Decimal(str(dyn_perf["a_min"])), a_max=Decimal(str(dyn_perf["a_max"])),
                      v_nominal=Decimal(str(Decimal(str(s)) * Decimal('0.44704'))),
                      mass=Decimal(str(dyn_perf["mass"])))
    density = Decimal(str(env["density"])) / Decimal(str(1000))
    prior = Prior(density=density)
    freq_con = Decimal(str(cont["frequency"]))
    ts_con = 1 / freq_con
    n_ts_con = round(ts_con / sp.dt)
    controller = BasicController(prob_threshold=Decimal(str(cont["prob_threshold"])), vs=vs,
                                 d_stop=Decimal(str(cont["d_stop"])), t_react=Decimal(str(cont["t_react"])),
                                 frequency=n_ts_con * sp.dt)
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
        idx = int(sens_perf.lsd[i]/ds)
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

    logger.info('Fineshed conficence level initialization for accuracy.')
    confidence_level_list = ConfLevelList(list_cl, tresh_idx)
    sens_perf.cl_list = confidence_level_list

    sp.sens_param = sens_param
    sp.vs = vs
    sp.prior = prior
    sp.controller = controller
    sp.sens_perf = sens_perf
    logger.info('Start level 0 simulations.')
    for i in range(sp.nsims):
        fn = f'output/{experiment_key}-{i}.yaml'
        if not os.path.exists(fn):
            sp.seed = i
            pm = simulate_one(sp)
            if pm.collided is None:
                momentum = pm.collided
            else:
                momentum = pm.collided.momentum

            data = {'collided': str(momentum),
                    'average_velocity': str(pm.average_velocity),
                    'control_effort': str(pm.control_effort)}
            with open(fn, 'w') as f:
                yaml.dump(data, f)

        with open(fn, 'r') as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        collided_mom = data['collided']
        if collided_mom == str(None):
            collided_mom = None
        else:
            collided_mom = Decimal(collided_mom)
        av_vel = Decimal(data['average_velocity'])
        cont_eff = Decimal(data['control_effort'])

        if collided_mom is not None:
            collision[i] = 1
            average_collision_momentum[i] = collided_mom
        discomfort[i] = cont_eff
        average_velocity[i] = av_vel

    confidence_level = 0.95
    degrees_freedom = sp.nsims - 1
    discomfort_mean = np.mean(discomfort)
    discomfort_var = np.var(discomfort)
    discomfort_standard_error = scipy.stats.sem(discomfort)
    discomfort_confidence_interval = scipy.stats.t.interval(confidence_level, degrees_freedom,
                                                            discomfort_mean, discomfort_standard_error)
    p_collision = np.mean(collision)
    danger = average_collision_momentum * p_collision
    danger_mean = np.mean(danger)
    danger_var = np.var(danger)
    danger_standard_error = scipy.stats.sem(danger)
    danger_confidence_interval = scipy.stats.t.interval(confidence_level, degrees_freedom,
                                                        danger_mean, danger_standard_error)
    average_velocity_mean = np.mean(average_velocity)
    average_velocity_var = np.var(average_velocity)
    average_velocity_standard_error = scipy.stats.sem(average_velocity)
    average_velocity_confidence_interval = scipy.stats.t.interval(confidence_level, degrees_freedom,
                                                                  average_velocity_mean,
                                                                  average_velocity_standard_error)

    discomfort_stat = Statistics(mean=Decimal(np.asscalar(discomfort_mean)), var=Decimal(np.asscalar(discomfort_var)),
                                 u95=Decimal(np.asscalar(discomfort_confidence_interval[1])),
                                 l95=Decimal(np.asscalar(discomfort_confidence_interval[0])))
    danger_stat = Statistics(mean=Decimal(np.asscalar(danger_mean)), var=Decimal(np.asscalar(danger_var)),
                             u95=Decimal(np.asscalar(danger_confidence_interval[1])),
                             l95=Decimal(np.asscalar(danger_confidence_interval[0])))
    average_velocity_stat = Statistics(mean=Decimal(np.asscalar(average_velocity_mean)),
                                       var=Decimal(np.asscalar(average_velocity_var)),
                                       u95=Decimal(np.asscalar(average_velocity_confidence_interval[1])),
                                       l95=Decimal(np.asscalar(average_velocity_confidence_interval[0])))

    return PerformanceMetrics(danger=danger_stat, discomfort=discomfort_stat, average_velocity=average_velocity_stat)


def collided(s: State, vs: VehicleStats) -> CollisionStats:
    if s.objects:
        if s.objects[0].d <= 0:
            momentum = s.vstate.v * vs.mass
            cs = CollisionStats(momentum=momentum)
            return cs


def stopped(s: State) -> bool:
    if s.objects:
        if round(s.vstate.v, 1) == 0.0 and s.objects[0].d <= Decimal('10'):
            print("stopped True")
            return True

    return False


def simulate_one(sp: SimParameters) -> OneSimPerformanceMetrics:
    ds = sp.sens_param.ds
    n = sp.sens_param.n
    np.random.seed(sp.seed)
    random.seed(sp.seed)

    density = sp.prior.density * sp.road_length
    n_objects = np.random.poisson(lam=float(density))
    print("Number of objects at track: ", n_objects)
    objects = []  # sample from poisson with intensity sp.prior.density
    for o in range(0, n_objects):
        x = round(random.uniform(0.0, float(sp.road_length)), 1)
        obj = Object(Decimal(str(x)))
        objects.append(obj)
    objects.append(Object(Decimal('20')))
    objects.sort(key=lambda o: o.d, reverse=False)  # sorting objects

    vstate0 = VehicleState(Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0'))

    state = State(vstate0, objects)
    logger.info('State initialization.')
    density_belief = sp.prior.density * sp.sens_param.ds
    pp = density_belief * Decimal(np.exp(-float(density_belief)))
    po = [pp for _ in range(n)]
    alpha0 = po
    inference = Inference(alpha=alpha0)
    action = Action(accel=Decimal('0'))

    control_interval = int(sp.controller.frequency / sp.dt)
    logger.info(f'control_interval {control_interval}')
    control_effort = 0
    t = Decimal(0.0)
    l = int(sp.sens_param.latency / sp.dt)
    delays = [state] * l
    delayed_st = DelayedStates(states=delays, latency=sp.sens_param.latency, l=l)
    vstates_list = []
    inference_list = []
    object_list = []
    print("Simulation running...")
    i = 0
    sensing_interval = int(sp.sens_param.frequency / sp.dt)
    logger.info(f'sensing_interval {sensing_interval}')
    annimation_interval = int(Decimal('0.05') / sp.dt)
    while state.vstate.x <= sp.road_length:
        i += 1
        t = i * sp.dt
        print('time', t)
        if i % sensing_interval == 0:
            observations = compute_observations(sp.sens_perf, sp.sens_param, sp.prior, delayed_st.states[0])
        else:
            observations = None
        delta = state.vstate.x - state.vstate.x_prev
        delta_idx = int(delta / ds)
        inference1 = prediction_model(inf=inference, delta_idx=delta_idx, prior=alpha0[0])

        if observations is None:
            inference = inference1
        else:
            inference = observation_model(inf0=inference1, obs=observations, sens_param=sp.sens_param, sp=sp.sens_perf, density=po)

        if i % control_interval == 0:
            action = sp.controller.get_action(state.vstate, inference, ds)
        else:
            action = action
        state = update_state(state, action, sp.dt)
        delayed_st.update(state)
        print('x', state.vstate.x)
        print('v', state.vstate.v)
        print("object", state.objects[0].d)
        control_effort += abs(action.accel) * sp.dt

        c = collided(state, sp.vs)
        is_stopped = stopped(state)

        if sp.do_animation:
            if i % annimation_interval == 0:
                vstates_list.append(state.vstate)
                inference_list.append(inference)
                obj_a = [ob.d for ob in state.objects]
                object_list.append(obj_a)

        if c is not None:
            print("Vehicle crashed in object!!")
            avg_control_effort = control_effort / t
            average_velocity = state.vstate.x / t
            if sp.do_animation:
                create_animation(vstates_list, inference_list, object_list, sp.sens_param.list_of_ds)
            return OneSimPerformanceMetrics(c, Decimal(average_velocity), Decimal(avg_control_effort))

        if is_stopped:
            print("Vehicle stopped safely in front of obstacle.")
            # Adjust distance between objects
            i = 0
            for obj in state.objects:
                if obj.d < 10:
                    i += 1

            state.objects = state.objects[i:]

            # try this instead
            # state.objects = [_ for _ in state.objects if _.d > 10]

    avg_control_effort = control_effort / t
    average_velocity = state.vstate.x / t
    if sp.do_animation:
        create_animation(vstates_list, inference_list, object_list, sp.sens_param.list_of_ds)
    return OneSimPerformanceMetrics(None, Decimal(average_velocity), Decimal(avg_control_effort))
