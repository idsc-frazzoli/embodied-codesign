from decimal import Decimal
import random

import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

from controller.basic_controller import BasicController
from controller.controller import Action, Controller
from sensing.sensing_performance import SensingPerformance, SensingParameters
from simulator.performance import PerformanceMetrics, CollisionStats, OneSimPerformanceMetrics, StoppedStats
from vehicle.state_estimation import Prior, Belief, compute_observations, prediction_model, observation_model
from vehicle.vehicle import State, VehicleState, VehicleStats, Object, DelayedStates


def update_state(s: State, action: Action, dt: Decimal) -> State:
    x = Decimal('0.5') * action.accel * dt**2 + s.vstate.v * dt + s.vstate.x
    if (x-s.vstate.x) < 0:
        x = s.vstate.x # no backward driving

    v = action.accel * dt + s.vstate.v
    if v < 0:
        v = Decimal('0.0') # no backwards driving

    for i in range(len(s.objects)):
        s.objects[i].d = s.objects[i].d - x + s.vstate.x

    vstate = VehicleState(x=x, v=v)

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
    wt: Decimal # waiting time in front of obstacle until obstacle disappears


def simulate(sp: SimParameters, dyn_perf, sens, s, env, cont) -> PerformanceMetrics:
    """  nsims: number of simulations"""
    n_collisions = 0
    discomfort = Decimal('0')
    average_velocity = Decimal('0')
    average_collision_momentum = Decimal('0')

    ds = Decimal('0.5')
    max_distance = Decimal('50.0')
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    freq = Decimal(str(sens["frequency"]))
    ts_sens = 1/freq
    n_ts_sens = round(ts_sens/sp.dt)
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, frequency=n_ts_sens * sp.dt, latency=1 * sp.dt)

    vs = VehicleStats(a_min=Decimal(str(dyn_perf["a_min"])), a_max=Decimal(str(dyn_perf["a_max"])), v_nominal=Decimal(str(s / 3.6)),
                      mass=Decimal(str(dyn_perf["mass"])))
    prior = Prior(density=Decimal(str(env["density"])))
    controller = BasicController(prob_threshold=Decimal(str(cont["prob_threshold"])), vs=vs, sp=sens_param,
                                 d_stop=Decimal(str(cont["d_stop"])), t_react=Decimal(str(cont["t_react"])))
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

    return PerformanceMetrics(p_collision=p_collision, discomfort=discomfort, average_velocity=average_velocity,
                              average_collision_momentum=average_collision_momentum)


def collided(s: State, vs: VehicleStats) -> CollisionStats:
    # check if collided
    # for obj in s.objects:
    #     if obj.d <= 0:
    #         momentum = s.vstate.v * vs.mass
    #         cs = CollisionStats(momentum=momentum)
    #         return cs
    if s.objects[0].d <= 0:
        momentum = s.vstate.v * vs.mass
        cs = CollisionStats(momentum=momentum)
        return cs


def simulate_one(sp: SimParameters) -> OneSimPerformanceMetrics:
    ds = sp.sens_param.ds
    n = sp.sens_param.n
    np.random.seed(sp.seed)
    random.seed(sp.seed)

    # first, create the state
    # static obstacles

    density = sp.prior.density * sp.road_length
    n_objects = np.random.poisson(lam=float(density))
    print("Number of objects at track: ", n_objects)
    objects = []  # sample from poisson with intensity sp.prior.density
    print("Objects at following positions: ")
    for o in range(0, n_objects):
        x = round(random.uniform(0.0, float(sp.road_length)), 1)
        print(x)
        obj = Object(Decimal(str(x)))
        objects.append(obj)

    objects.sort(key=lambda o: o.d, reverse=False) # sorting objects

    vstate0 = VehicleState(Decimal('0.0'), Decimal('0.0'))

    state = State(vstate0, objects)
    action = Action(Decimal('0.0'))
    density_belief = sp.prior.density * sp.sens_param.max_distance
    pp = density_belief * Decimal(np.exp(-float(density_belief)))
    po = [Decimal(pp / n) for _ in range(n)]
    belief = Belief(po)

    plot_belief = False
    control_effort = 0
    t = Decimal(0.0)
    stop_stats = StoppedStats(wt=Decimal(0.0), stop=False, d_stop=sp.controller.d_stop)
    delays = [state for i in range(int(sp.sens_param.latency/sp.dt))]
    l = len(delays)
    delayed_st = DelayedStates(states=delays, latency=sp.sens_param.latency, l=l)

    while state.vstate.x <= sp.road_length:
        t += sp.dt
        if float(t % sp.sens_param.frequency) == 0.0:
            observations = compute_observations(sp.sens_perf, sp.sens_param, sp.prior, delayed_st.states[0])
        else:
            observations = None

        belief1 = prediction_model(b0=belief, u=action, s=state, dt=sp.dt, ds=ds, prior=sp.prior)

        if observations is None:
            belief = belief1
        else:
            belief = observation_model(belief1, observations, sp.sens_param.list_of_ds, sp.sens_perf)

        if plot_belief:
            plt.plot(belief.po)
            plt.ylabel('Belief')
            plt.xlabel('d [m]')
            plt.show()

        action = sp.controller.get_action(state.vstate, belief)
        state = update_state(state, action, sp.dt)
        delayed_st.update(state)
        print('Time: ', t)
        print("Vehicle's current position [m]: ", round(state.vstate.x, 2))
        print("Vehicle's current velocity [m/s]: ", round(state.vstate.v, 2))

        control_effort += abs(action.accel) * sp.dt

        c = collided(state, sp.vs)
        stop_stats.stopped(state, sp.dt)

        if c is not None:
            print("Vehicle crashed in object!!")
            avg_control_effort = control_effort / t
            average_velocity = state.vstate.x / t
            return OneSimPerformanceMetrics(c, Decimal(average_velocity), Decimal(avg_control_effort))

        if stop_stats.stop:
            if stop_stats.wt >= sp.wt:
                print("Vehicle stopped safely in front of obstacle.")
                state.objects = state.objects[1:]
                stop_stats.wt = Decimal(0.0)

    avg_control_effort = control_effort / t
    average_velocity = state.vstate.x / t
    return OneSimPerformanceMetrics(None, Decimal(average_velocity), Decimal(avg_control_effort))
