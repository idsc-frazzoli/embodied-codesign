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
from sensing.sensing_performance import SensingParameters, SensingPerformance
from simulator.create_animation import create_animation
from simulator.performance import CollisionStats, OneSimPerformanceMetrics, PerformanceMetrics, Statistics
from vehicle.state_estimation import Belief, compute_observations, observation_model, prediction_model, Prior
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
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, frequency=n_ts_sens * sp.dt, latency=1 * sp.dt)

    vs = VehicleStats(a_min=Decimal(str(dyn_perf["a_min"])), a_max=Decimal(str(dyn_perf["a_max"])),
                      v_nominal=Decimal(str(s / 3.6)),
                      mass=Decimal(str(dyn_perf["mass"])))
    density = Decimal(str(env["density"])) / Decimal(str(1000))
    prior = Prior(density=density)
    controller = BasicController(prob_threshold=Decimal(str(cont["prob_threshold"])), vs=vs, sp=sens_param,
                                 d_stop=Decimal(str(cont["d_stop"])), t_react=Decimal(str(cont["t_react"])))
    sens_perf = SensingPerformance(sp=sens_param)
    fn = sens_curves["fn"]
    sens_perf.fn = [Decimal(p) for p in fn]
    fp = sens_curves["fp"]
    sens_perf.fp = [Decimal(p) for p in fp]
    sens_perf.lsd = [Decimal(str(ds * i * Decimal(str(0.05)) + Decimal(0.5))) for i in range(n)]

    sp.sens_param = sens_param
    sp.vs = vs
    sp.prior = prior
    sp.controller = controller
    sp.sens_perf = sens_perf

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
                                 u95=Decimal(np.asscalar(discomfort_confidence_interval[0])),
                                 l95=Decimal(np.asscalar(discomfort_confidence_interval[1])))
    danger_stat = Statistics(mean=Decimal(np.asscalar(danger_mean)), var=Decimal(np.asscalar(danger_var)),
                             u95=Decimal(np.asscalar(danger_confidence_interval[0])),
                             l95=Decimal(np.asscalar(danger_confidence_interval[1])))
    average_velocity_stat = Statistics(mean=Decimal(np.asscalar(average_velocity_mean)),
                                       var=Decimal(np.asscalar(average_velocity_var)),
                                       u95=Decimal(np.asscalar(average_velocity_confidence_interval[0])),
                                       l95=Decimal(np.asscalar(average_velocity_confidence_interval[1])))

    return PerformanceMetrics(danger=danger_stat, discomfort=discomfort_stat, average_velocity=average_velocity_stat)


def collided(s: State, vs: VehicleStats) -> CollisionStats:
    if s.objects:
        if s.objects[0].d <= 0:
            momentum = s.vstate.v * vs.mass
            cs = CollisionStats(momentum=momentum)
            return cs


def stopped(s: State) -> bool:
    if s.objects:
        if round(s.vstate.v, 2) == 0.0 and s.objects[0].d <= 5:
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
    objects.sort(key=lambda o: o.d, reverse=False)  # sorting objects

    vstate0 = VehicleState(Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0'))

    state = State(vstate0, objects)
    density_belief = sp.prior.density * sp.sens_param.max_distance
    pp = density_belief * Decimal(np.exp(-float(density_belief)))
    po = [Decimal(pp / n) for _ in range(n)]
    belief = Belief(po)

    control_effort = 0
    t = Decimal(0.0)
    l = int(sp.sens_param.latency / sp.dt)
    delays = [state] * l
    delayed_st = DelayedStates(states=delays, latency=sp.sens_param.latency, l=l)
    vstates_list = []
    belief_list = []
    object_list = []
    print("Simulation running...")
    i = 0
    sensing_interval = int(sp.sens_param.frequency / sp.dt)
    logger.info(f'sensing_interval {sensing_interval}')
    while state.vstate.x <= sp.road_length:
        i += 1
        t = i * sp.dt

        if i % sensing_interval == 0:
            observations = compute_observations(sp.sens_perf, sp.sens_param, sp.prior, delayed_st.states[0])
        else:
            observations = None

        delta = state.vstate.x - state.vstate.x_prev
        delta_idx = int(delta / ds)
        belief1 = prediction_model(b0=belief, delta_idx=delta_idx, delta=delta, prior=sp.prior)

        if observations is None:
            belief = belief1
        else:
            belief = observation_model(belief1, observations, sp.sens_param.list_of_ds, sp.sens_perf)

        action = sp.controller.get_action(state.vstate, belief)
        state = update_state(state, action, sp.dt)
        delayed_st.update(state)

        control_effort += abs(action.accel) * sp.dt

        c = collided(state, sp.vs)
        is_stopped = stopped(state)

        if sp.do_animation:
            if float(t % Decimal(str(0.05))) == 0.0:
                vstates_list.append(state.vstate)
                belief_list.append(belief)
                obj_a = [ob.d for ob in state.objects]
                object_list.append(obj_a)

        if c is not None:
            print("Vehicle crashed in object!!")
            avg_control_effort = control_effort / t
            average_velocity = state.vstate.x / t
            if sp.do_animation:
                create_animation(vstates_list, belief_list, object_list, sp.sens_param.list_of_ds)
            return OneSimPerformanceMetrics(c, Decimal(average_velocity), Decimal(avg_control_effort))

        if is_stopped:
            print("Vehicle stopped safely in front of obstacle.")
            # Adjust distance between objects
            i = 0
            for obj in state.objects:
                if obj.d < 10:
                    i += 1

            state.objects = state.objects[i:]

    avg_control_effort = control_effort / t
    average_velocity = state.vstate.x / t
    if sp.do_animation:
        create_animation(vstates_list, belief_list, object_list, sp.sens_param.list_of_ds)
    return OneSimPerformanceMetrics(None, Decimal(average_velocity), Decimal(avg_control_effort))
