import os
import random
import time
from dataclasses import dataclass
from decimal import Decimal
from typing import Dict

import numpy as np
import yaml
#import scipy.stats
import scipy

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

    vehicle_state = VehicleState(x=x, v=v, x_prev=s.vstate.x, v_prev=s.vstate.v)

    return State(vehicle_state, s.objects)


@dataclass
class SimParameters:
    nsims: int
    road_length: Decimal # IN METERS
    prior: Prior
    controller: Controller
    sens_perf: SensingPerformance
    dt: Decimal
    sens_param: SensingParameters
    vs: VehicleStats
    seed: int
    do_animation: bool
    add_object_at: str

    def __init__(self, nsims: int, road_length: Decimal, dt: Decimal, seed: int, do_animation: bool, add_object_at: str) -> None:
        self.nsims = nsims
        self.road_length = road_length
        self.dt = dt
        self.seed = seed
        self.do_animation = do_animation
        self.add_object_at = add_object_at


def initialize_metrics(sp: SimParameters):
    discomfort = np.zeros(sp.nsims)
    average_velocity = np.zeros(sp.nsims)
    average_collision_momentum = np.zeros(sp.nsims)
    collision = np.zeros(sp.nsims)
    return discomfort, average_velocity, average_collision_momentum, collision


def initialize_veh_stats(s: Decimal, dyn_perf: Dict):
    vs = VehicleStats(a_min=Decimal(str(dyn_perf["a_min_m_s2"])), a_max=Decimal(str(dyn_perf["a_max_m_s2"])),
                      v_nominal=s,
                      mass=Decimal(str(dyn_perf["mass_g"])))
    return vs


def read_params_from_curves(sens_curves: Dict, sens: Dict):
    ds = Decimal(sens_curves["ds"])
    max_distance = Decimal(sens_curves["max_distance"])
    freq_sens_hz = Decimal(str(sens["frequency_hz"]))
    lat_sens_s = Decimal(str(sens["latency_s"]))
    return ds, max_distance, freq_sens_hz, lat_sens_s


def initialize_sensing_parameters(sens_curves: Dict, sens: Dict, sp: SimParameters):
    # Reading them out
    ds, max_distance, freq_sens_hz, lat_sens_s = read_params_from_curves(sens_curves=sens_curves, sens=sens)
    # First, we compute the number of steps in the space discretization up to max_distance
    n = int(round(max_distance / ds))
    # and we list the different steps
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    # Computing the sampling period
    ts_sens_s = 1 / freq_sens_hz
    # Computing the number of steps in the time discretization
    n_ts_sens = round(ts_sens_s / sp.dt)
    # Same with latency
    n_ts_lat_sens = round(lat_sens_s / sp.dt)
    # Initializing sensing parameters
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, sens_sampl_time_s=n_ts_sens * sp.dt, latency_s=n_ts_lat_sens * sp.dt)
    return sens_param


def initialize_controller(cont: Dict, s: Decimal, dyn_perf: Dict, sens_param: SensingParameters, sp: SimParameters):
    # Reading out control frequency in Hz
    freq_con_hz = Decimal(str(cont["frequency_hz"]))
    # Reading out probability threshold
    prob_threshold = Decimal(str(cont["prob_threshold"]))
    # Computing control period
    ts_con = 1 / freq_con_hz
    # Computing number of time discretization steps for control
    n_ts_con = round(ts_con / sp.dt)
    # Initialize vehicle statistics
    vs = initialize_veh_stats(s=s, dyn_perf=dyn_perf)
    # Initialize the procentage of a_max usage
    p_a_max = Decimal(str(cont["percentage_amax"]))
    # Initialize controller
    controller = BasicController(prob_threshold=prob_threshold, vs=vs, ds=sens_param.ds,
                                 d_stop=Decimal(str(cont["d_stop_m"])),
                                 cont_sampl_time_s=n_ts_con * sp.dt, p_a_max=p_a_max)
    return controller


def load_sensing_performance(sens_curves: Dict, sens_param: SensingParameters):
    sens_perf = SensingPerformance(sp=sens_param)
    fn = sens_curves["fn"]
    sens_perf.fn = [Decimal(p) for p in fn]
    fp = sens_curves["fp"]
    sens_perf.fp = [Decimal(p) for p in fp]
    lsd = sens_curves["accuracy"]
    sens_perf.lsd = [Decimal(p) for p in lsd]
    return sens_perf


def get_stats(performance_list, cl:float, df:int) -> Statistics:
    confidence_level = cl
    degrees_freedom = max(1, df - 1)
    if len(performance_list) == 1:
        performance_stat = Statistics(mean=Decimal(performance_list[0]), var=Decimal('0.0'),
                                      u95=Decimal('0.0'), l95=Decimal('0.0'))
    else:
        performance_mean = np.mean(performance_list)
        performance_var = np.var(performance_list)
        performance_standard_error = scipy.stats.sem(performance_list)
        if performance_standard_error == float('nan') or performance_standard_error == 0.0:
            performance_standard_error = 0.00001
        performance_confidence_interval = scipy.stats.t.interval(confidence_level, degrees_freedom,
                                                                performance_mean, performance_standard_error)
        performance_stat = Statistics(mean=Decimal(np.asscalar(performance_mean)), var=Decimal(np.asscalar(performance_var)),
                                     u95=Decimal(np.asscalar(performance_confidence_interval[1])),
                                     l95=Decimal(np.asscalar(performance_confidence_interval[0])))

    return performance_stat


def simulate(sp: SimParameters, dyn_perf: Dict, sens: Dict, sens_curves: Dict, s: Decimal,
             env: Dict, cont: Dict, experiment_key: str, file_directory: str) -> PerformanceMetrics:
    # Initializing metrics
    discomfort, average_velocity, average_collision_momentum, collision = initialize_metrics(sp=sp)
    # Initializing sensing parameters
    sens_param = initialize_sensing_parameters(sens_curves=sens_curves, sens=sens, sp=sp)
    # Initializing controller
    controller = initialize_controller(cont=cont, s=s, dyn_perf=dyn_perf, sens_param=sens_param, sp=sp)
    # In ppl/m
    density = Decimal(str(env["density_ped_km"])) / Decimal(str(1000))
    prior = Prior(density=density)

    sens_perf = load_sensing_performance(sens_curves=sens_curves, sens_param=sens_param)

    sp.sens_param = sens_param
    sp.vs = controller.vs
    sp.prior = prior
    sp.controller = controller
    sp.sens_perf = sens_perf
    for i in range(sp.nsims):
        fn = os.path.join(file_directory,'single_experiments', f'{experiment_key}.experiment.{i}.yaml')
        if not os.path.exists(fn):
            dn = os.path.dirname(fn)
            if not os.path.exists(dn):
                os.makedirs(dn)
            sp.seed = i
            t0 = time.process_time()
            pm = simulate_one(sp)
            t1 = time.process_time()
            dt = t1-t0
            logger.info(f'{fn}  {dt:.2f} seconds')
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

    discomfort_stat = get_stats(discomfort, cl=0.95, df=sp.nsims)
    p_collision = np.mean(collision)
    danger = average_collision_momentum * p_collision
    danger_stat = get_stats(danger, cl=0.95, df=sp.nsims)
    average_velocity_stat = get_stats(average_velocity, cl=0.95, df=sp.nsims)

    return PerformanceMetrics(danger=danger_stat, discomfort=discomfort_stat, average_velocity=average_velocity_stat)


def collided(s: State, vs: VehicleStats) -> CollisionStats:
    if s.objects:
        if s.objects[0].d <= 0:
            # transform mass to kg since units of momentum are N s = kg m/s
            momentum = s.vstate.v * vs.mass/Decimal("1000.0")
            cs = CollisionStats(momentum=momentum)
            return cs


def stopped(s: State) -> bool:
    if s.objects:
        if round(s.vstate.v, 2) == 0.0 and s.objects[0].d <= 10:
            return True

    return False


def generate_objects(sp: SimParameters):
    poisson_density = sp.prior.density*sp.road_length
    number_objects = np.random.poisson(lam=float(poisson_density))
    objects = []
    for o in range(number_objects):
        dist = round(random.uniform(0.0, float(sp.road_length)),1)
        obj = Object(Decimal(str(dist)))
        objects.append(obj)
    if sp.add_object_at != "none":
        objects.append(Object(Decimal(sp.add_object_at)))
    objects.sort(key=lambda ob: ob.d, reverse=False)
    logger.info(f'Number of objects at track: {len(objects)}')
    return objects


def initialize_state(objects):
    initial_state = VehicleState(Decimal('0.0'), Decimal('0.0'), Decimal('0.0'), Decimal('0.0'))
    state = State(initial_state, objects)
    return state


def initialize_belief(sp: SimParameters):
    # for Dejan: note that density is in 1/m and distance in m
    belief_density = sp.prior.density * sp.sens_param.max_distance
    n = sp.sens_param.n
    temp_prob = np.exp(-float(belief_density))
    pp = belief_density * Decimal(temp_prob)
    po = [Decimal(pp / n) for _ in range(n)]
    initialized_belief = Belief(po)
    return initialized_belief


def simulate_one(sp: SimParameters) -> OneSimPerformanceMetrics:
    ds = sp.sens_param.ds
    np.random.seed(sp.seed)
    random.seed(sp.seed)

    objects = generate_objects(sp)
    state = initialize_state(objects)

    belief = initialize_belief(sp)
    action = Action(accel=Decimal('0'))

    logger.info(f'Sampling time controller (inverse frequency) {sp.controller.cont_sampl_time_s} dt {sp.dt}')
    control_interval = int(np.ceil(sp.controller.cont_sampl_time_s/sp.dt))

    control_effort = 0
    t = Decimal(0.0)
    l = int(np.ceil(sp.sens_param.latency_s / sp.dt))
    logger.info(f'latency interval {sp.sens_param.latency_s} dt {sp.dt} l {l}')

    l = max(1, l)
    delays = [state] * l
    delayed_st = DelayedStates(states=delays, latency=sp.sens_param.latency_s, l=l)
    vstates_list = []
    belief_list = []
    object_list = []
    i = 0
    sensing_interval = int(np.ceil(sp.sens_param.sens_sampl_time_s/sp.dt))
    logger.info(f'Sampling time sensor (inverse frequency) {sp.sens_param.sens_sampl_time_s} dt {sp.dt} sensing_interval {sensing_interval}')
    logger.info("Simulation running...")

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

        if i % control_interval == 0:
            action = sp.controller.get_action(state.vstate, belief)
        else:
            action = action

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

            # try this instead
            # state.objects = [_ for _ in state.objects if _.d > 10]

    avg_control_effort = control_effort / t
    average_velocity = state.vstate.x / t
    if sp.do_animation:
        create_animation(vstates_list, belief_list, object_list, sp.sens_param.list_of_ds)
    return OneSimPerformanceMetrics(None, Decimal(average_velocity), Decimal(avg_control_effort))
