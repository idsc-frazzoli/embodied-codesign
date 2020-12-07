import argparse
import json
import os
import random
import time
from decimal import Decimal
from multiprocessing import Pool

import yaml

from simulator.simulator import SimParameters, simulate
from utils.yaml_file_generation import read_results, write_bc_dpc
from simulator import logger

def write_store_param_file(args, param_file_name, params):
    sp = params[0]
    sens_curves = params[3]
    s = params[4]
    env = params[5]
    cont = params[6]
    sens_key = params[9]
    s_perf = params[10]
    veh_key = params[11]
    env_key = params[12]
    cont_key = params[13]
    params_f = vars(args)
    params_f["add_object_at"] = sp.add_object_at
    params_f["algorithm"] = s_perf
    params_f["controller"] = cont_key
    params_f["control_freq"] = str(cont["frequency_hz"])
    params_f["control_treshold"] = str(cont["prob_threshold"])
    params_f["environment"] = env_key
    params_f["env_day_night"] = env["scenario_day_night"]
    params_f["env_density"] = str(env["density_ped_km"])
    params_f["ds"] = sens_curves["ds"]
    params_f["max_distance"] = sens_curves["max_distance"]
    params_f["vehicle"] = veh_key
    params_f["sensor"] = sens_key
    params_f["speed"] = str(s)
    with open(param_file_name, 'w') as f:
        yaml.dump(params_f, f, default_flow_style=False)

def generate_all(args):
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)
    print('sensors')
    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)
    print('dyn_perf')
    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)
    print('environment')
    with open('data/input/speeds.yaml') as file:
        cruise_speeds = yaml.load(file, Loader=yaml.FullLoader)
    print('speeds')
    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)
    print('control_param')
    with open('data/input/sensing_performance_curves.json') as file:
        sens_curves = json.load(file)
    print('sens_curves')

    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=args.nsims, road_length=Decimal(args.road_length), dt=Decimal(args.dt),
                       seed=args.seed, do_animation=False, add_object_at="none", stop_time=Decimal(args.stop_time),
                       discomfort_penalty=Decimal(args.discomfort_penalty))

    logger.info(f'nsims = {args.nsims}')

    # first see what needs to be run
    to_run = []  # list of parameters
    for veh_key, dyn_perf in vehicles.items():
        for env_key, env in environment.items():
            print(f'{veh_key}, {env_key}')
            for sens_key, sens in sensors.items():
                sens_algs = sens["algorithm"]
                sens_algs_env_dn_list = sens_algs[env["scenario_day_night"]]
                for s_perf in sens_algs_env_dn_list:
                    s_perf_curve_key = f'{sens_key}_{s_perf}'
                    s_perf_curve = sens_curves[s_perf_curve_key]
                    for s in speed:
                        for cont_key, cont in control_param.items():
                            experiment_key = f'{veh_key}-{env_key}-{sens_key}-{s_perf_curve_key}-{str(s)}-{cont_key}'
                            fn = os.path.join(args.basedir, args.simversion, str(veh_key), str(env_key),
                                              str(sens_key), str(s_perf),
                                              str(s), str(cont_key), f'{experiment_key}.experiment.yaml')
                            dn = os.path.dirname(fn)
                            param_file_name = os.path.join(dn, f'{experiment_key}.parameters.yaml')
                            if not os.path.exists(dn):
                                os.makedirs(dn)
                            if not os.path.exists(fn):
                                params = sp, dyn_perf, sens, s_perf_curve, s, env, cont, experiment_key, \
                                         fn, sens_key, s_perf_curve_key, veh_key, env_key, cont_key,

                                write_store_param_file(args, param_file_name, params)
                                to_run.append(params)


    nprocesses = args.nprocesses
    n = len(to_run)
    print(f'Created {n} experiments to run')

    params_f = vars(args)
    params_f['number_experiments_to_run'] = n
    param_file_name = os.path.join(args.basedir, args.simversion, f'{args.simversion}.parameters.yaml')
    with open(param_file_name, 'w') as f:
        yaml.dump(params_f, f, default_flow_style=False)

    random.shuffle(to_run)

    # to_run = to_run[:10]
    multiprocess = True
    if multiprocess:
        with Pool(processes=nprocesses) as pool:
            pool.map(simulate_and_write, to_run)
    else:
        for x in to_run:
            simulate_and_write(x)

def simulate_and_write(params):
    sp = params[0]
    dyn_perf = params[1]
    sens = params[2]
    sens_curves = params[3]
    s = params[4]
    env = params[5]
    cont = params[6]
    experiment_key = params[7]
    fn = params[8]
    sens_key = params[9]
    s_perf_key = params[10]
    veh_key = params[11]
    env_key = params[12]
    cont_key = params[13]
    dn = os.path.dirname(fn)
    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, dn)
    danger = {
        "mean": str(round(performance.danger.mean, 2)), "var": str(round(performance.danger.var, 2)),
        "u95": str(round(performance.danger.u95, 2)),
        "l95": str(round(performance.danger.l95, 2))
    }
    discomfort = {
        "mean": str(round(performance.discomfort.mean, 2)), "var": str(round(performance.discomfort.var, 2)),
        "u95": str(round(performance.discomfort.u95, 2)), "l95": str(round(performance.discomfort.l95, 2))
    }
    control_effort = {
        "mean": str(round(performance.control_effort.mean, 2)), "var": str(round(performance.control_effort.var, 2)),
        "u95": str(round(performance.control_effort.u95, 2)), "l95": str(round(performance.control_effort.l95, 2))
    }
    ad_perf = {
        "danger": danger,
        "discomfort": discomfort,
        "control_effort": control_effort,
        "speed": s, "sensor": sens_key, "sens_perf": s_perf_key, "dyn_perf": veh_key,
        "environment": env_key,
        "controller": cont_key,
        "stopped_too_slow": performance.stopped_too_slow,
    }
    with open(fn, 'w') as f:
        yaml.dump(ad_perf, f, default_flow_style=False)
    print("Finished Experiment: ", fn)


def generate_specifc(args):
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)
    print('sensors')
    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)
    print('dyn_perf')
    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)
    print('environment')
    with open('data/input/speeds.yaml') as file:
        cruise_speeds = yaml.load(file, Loader=yaml.FullLoader)
    print('speeds')
    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)
    print('control_param')
    with open('data/input/sensing_performance_curves.json') as file:
        sens_curves = json.load(file)
    print('sens_curves')
    with open('data/input/object_detection.yaml') as file:
        algorithms = yaml.load(file, Loader=yaml.FullLoader)
    print('algorithms')

    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=args.nsims, road_length=Decimal(args.road_length), dt=Decimal(args.dt),
                       seed=args.seed, do_animation=False, add_object_at="none", stop_time=Decimal(args.stop_time),
                       discomfort_penalty=Decimal(args.discomfort_penalty))

    logger.info(f'nsims = {args.nsims}')

    if "none" in args.vehicle_key:
        vehicle_key = []
        for v_key in vehicles:
            vehicle_key.append(v_key)
    else:
        vehicle_key = args.vehicle_key

    if "none" in args.environment_key:
        environment_key = []
        for e_key in environment:
            environment_key.append(e_key)
    else:
        environment_key = args.environment_key

    if "none" in args.sensor_key:
        sensor_key = []
        for s_key in sensors:
            sensor_key.append(s_key)
    else:
        sensor_key = args.sensor_key

    if "none" in args.control_key:
        control_key = []
        for c_key in control_param:
            control_key.append(c_key)
    else:
        control_key = args.control_key

    if "none" in args.speed_list:
        speed_list = []
        for s in speed:
            speed_list.append(s)
    else:
        speed_list = [float(s) for s in args.speed_list]

    if "none" in args.alg_key:
        alg_key = []
        for a in algorithms:
            alg_key.append(a)
    else:
        alg_key = args.alg_key


    # first see what needs to be run
    to_run = []  # list of parameters
    sens_perf_key_list = []
    for veh_key in vehicle_key:
        dyn_perf = vehicles[veh_key]
        for env_key in environment_key:
            env = environment[env_key]
            print(f'{veh_key}, {env_key}')
            for sens_key in sensor_key:
                print(sens_key)
                sens = sensors[sens_key]
                sens_algs = sens["algorithm"]
                sens_algs_env_dn_list = sens_algs[env["scenario_day_night"]]
                for s_perf in sens_algs_env_dn_list:
                    if s_perf not in alg_key:
                        continue
                    s_perf_curve_key = f'{sens_key}_{s_perf}'
                    sens_perf_key_list.append(s_perf_curve_key)
                    s_perf_curve = sens_curves[s_perf_curve_key]
                    for s in speed_list:
                        for cont_key in control_key:
                            cont = control_param[cont_key]
                            experiment_key = f'{veh_key}-{env_key}-{sens_key}-{s_perf_curve_key}-{str(s)}-{cont_key}'
                            fn = os.path.join(args.basedir, args.simversion, args.simversion, str(veh_key), str(env_key),
                                              str(sens_key), str(s_perf),
                                              str(s), str(cont_key), f'{experiment_key}.experiment.yaml')
                            dn = os.path.dirname(fn)
                            param_file_name = os.path.join(dn, f'{experiment_key}.parameters.yaml')
                            if not os.path.exists(dn):
                                os.makedirs(dn)
                            if not os.path.exists(fn):
                                params = sp, dyn_perf, sens, s_perf_curve, s, env, cont, experiment_key, \
                                         fn, sens_key, s_perf_curve_key, veh_key, env_key, cont_key,

                                write_store_param_file(args, param_file_name, params)
                                to_run.append(params)

    nprocesses = args.nprocesses
    n = len(to_run)
    print(f'Created {n} experiments to run')

    params_f = vars(args)
    params_f['vehicle_key'] = vehicle_key
    params_f['environment_keys'] = environment_key
    params_f['sensor_key'] = sensor_key
    params_f['sens_perf_key'] = sens_perf_key_list
    params_f['speed_list'] = speed_list
    params_f['control_key'] = control_key
    params_f['number_experiments_to_run'] = n
    param_file_name = os.path.join(args.basedir, args.simversion, f'{args.simversion}.parameters.yaml')
    with open(param_file_name, 'w') as f:
        yaml.dump(params_f, f, default_flow_style=False)

    random.shuffle(to_run)

    # to_run = to_run[:10]
    multiprocess = True
    if multiprocess:
        with Pool(processes=nprocesses) as pool:
            pool.map(simulate_and_write, to_run)
    else:
        for x in to_run:
            simulate_and_write(x)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--nsims', type=int, default=100, help='Number of simulations per parameter set.')
    parser.add_argument('--dt', type=str, default='0.01', help='Simulation sampling time in seconds.')
    parser.add_argument('--road_length', type=str, default='500.0', help='Road length in meters.')
    parser.add_argument('--do_animation', default=False, action="store_true", help='Flag for creating an animation.')
    parser.add_argument('--do_not_animation', default=True, action="store_true", help='Flag for creating an animation.')
    parser.add_argument('--seed', type=int, default=0, help='Seed for simulation.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--simversion', type=str, default='simulation_v.1.0', help='Controller name.')
    parser.add_argument('--nprocesses', type=int, default=4, help='Number of paralles processes.')
    parser.add_argument("--all", default=False, action="store_true", help="Flag to simulate all parameters.")
    parser.add_argument("--not_all", default=False, action="store_true", help="Flag to simulate all parameters.")
    parser.add_argument('--sensor_key', nargs='+', type=str, default=["none"], help='Sensor keys.')
    parser.add_argument('--vehicle_key', nargs='+', type=str, default=["none"], help='Vehicle keys.')
    parser.add_argument('--environment_key', nargs='+', type=str, default=["none"], help='Environment keys.')
    parser.add_argument('--control_key', nargs='+', type=str, default=["none"], help='Controller keys.')
    parser.add_argument('--alg_key', nargs='+', type=str, default=["none"], help='Algorithm keys.')
    parser.add_argument('--speed_list', nargs='+', type=str, default=["none"], help='Speed list.')
    parser.add_argument('--stop_time', type=str, default='400.0', help='Stop time of simulation if running too long.')
    parser.add_argument('--discomfort_penalty', type=str, default='5.0', help='The discomfort penalty when collided.')
    args = parser.parse_args()

    if args.all:
        generate_all(args)
    else:
        generate_specifc(args)


    fn_results = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.results.yaml')
    dn = os.path.dirname(fn_results)
    if not os.path.exists(dn):
        os.makedirs(dn)

    if not os.path.exists(fn_results):
        read_results(os.path.join(args.basedir, args.simversion), fn_results)
        print(f'Finished results file of {args.simversion}.')

    fn_dpc_models = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.brake_control_models.yaml')

    if not os.path.exists(fn_dpc_models):
        write_bc_dpc(dn, fn_dpc_models)
        print(f'Finished catalogue DPC file of {args.simversion}.')


