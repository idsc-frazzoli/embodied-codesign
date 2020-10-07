from decimal import Decimal
import random
from typing import Dict

import numpy as np
from dataclasses import dataclass

from controller.basic_controller import BasicController
from controller.controller import Action, Controller
from sensing.sensing_performance import SensingPerformance, SensingParameters
from simulator.create_animation import create_animation
from simulator.performance import PerformanceMetrics, CollisionStats, OneSimPerformanceMetrics
from vehicle.state_estimation import Prior, Belief, compute_observations, prediction_model, observation_model
from vehicle.vehicle import State, VehicleState, VehicleStats, Object, DelayedStates


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
    wt: Decimal  # waiting time in front of obstacle until obstacle disappears
    do_animation: bool

    def __init__(self, nsims: int, road_length: Decimal, dt: Decimal, seed: int, wt: Decimal,
                 do_animation: bool) -> None:
        self.nsims = nsims
        self.road_length = road_length
        self.dt = dt
        self.seed = seed
        self.wt = wt
        self.do_animation = do_animation


def simulate(sp: SimParameters, dyn_perf: Dict, sens: Dict, sens_curves: Dict, s: int,
             env: Dict, cont: Dict) -> PerformanceMetrics:
    """  nsims: number of simulations"""
    n_collisions = 0
    discomfort = Decimal('0')
    average_velocity = Decimal('0')
    average_collision_momentum = Decimal('0')

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
        sp.seed = i
        pm = simulate_one(sp)
        if pm.collided is not None:
            n_collisions += 1
            average_collision_momentum += pm.collided.momentum
        discomfort += pm.control_effort
        average_velocity += pm.average_velocity

    discomfort = discomfort / sp.nsims
    p_collision = Decimal(str(n_collisions / sp.nsims))
    average_velocity = average_velocity / sp.nsims
    average_collision_momentum = average_collision_momentum / sp.nsims
    danger = average_collision_momentum * p_collision

    return PerformanceMetrics(danger=danger, discomfort=discomfort, average_velocity=average_velocity)


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
    delays = [state for i in range(int(sp.sens_param.latency / sp.dt))]
    l = len(delays)
    delayed_st = DelayedStates(states=delays, latency=sp.sens_param.latency, l=l)
    delta_sum = Decimal(str(0))
    vstates_list = []
    belief_list = []
    object_list = []
    print("Simulation running...")
    while state.vstate.x <= sp.road_length:
        t += sp.dt
        if float(t % sp.sens_param.frequency) == 0.0:
            observations = compute_observations(sp.sens_perf, sp.sens_param, sp.prior, delayed_st.states[0])
        else:
            observations = None

        delta = state.vstate.x - state.vstate.x_prev
        delta_sum = delta_sum + delta

        if delta_sum >= ds:
            delta_idx = int(delta_sum / ds)
            belief1 = prediction_model(b0=belief, delta_idx=delta_idx, delta=delta, prior=sp.prior)
            delta_sum = Decimal(str(0))
        else:
            belief1 = belief

        if observations is None:
            belief = belief1
        else:
            belief = observation_model(belief1, observations, sp.sens_param.list_of_ds)

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
            state.objects = state.objects[1:]
            belief = Belief(po)

    avg_control_effort = control_effort / t
    average_velocity = state.vstate.x / t
    if sp.do_animation:
        create_animation(vstates_list, belief_list, object_list, sp.sens_param.list_of_ds)
    return OneSimPerformanceMetrics(None, Decimal(average_velocity), Decimal(avg_control_effort))
