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

def generate(basedir: str):
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
    # with open('data/input/camera_curves.yaml') as file:
    #     camera_curves = yaml.load(file, Loader=yaml.Loader)
    with open('data/input/camera_curves.json') as file:
            camera_curves = json.load(file)
    print('camera_curves')
    # with open('data/input/lidar_curves.yaml') as file:
    #     lidar_curves = yaml.load(file, Loader=yaml.Loader)
    with open('data/input/lidar_curves.json') as file:
        lidar_curves = json.load(file)

    print('lidar_curves')
    speed = cruise_speeds["speeds"]

    e = 0.1 / 100
    p = 0.01 / 100
    z = 1.96

    nsims = int(z ** 2 * (1 - p) * p / e ** 2)
    nsims = 50 # XXX
    dt = Decimal('0.1')
    logger.info(f'nsims = {nsims}')
    sp = SimParameters(nsims=nsims, road_length=Decimal('500.0'), dt=dt, seed=0,
                       do_animation=False)

    # first see what needs to be run
    to_run = []  # list of parameters
    for veh_key, dyn_perf in vehicles.items():
        for env_key, env in environment.items():
            print(f'{veh_key}, {env_key}')
            for sens_type_key, sens_type in sensors.items():
                for sens_key, sens in sens_type.items():
                    sens_perf = sens["sens_perf"]
                    for s_perf in sens_perf:
                        if sens_type_key == "lidar":
                            if "night" in env["scenario"]:
                                continue
                            sens_curves = lidar_curves[s_perf]
                        else:
                            if env["scenario"] not in s_perf:
                                continue
                            sens_curves = camera_curves[s_perf]
                        for s in speed:
                            for cont_key, cont in control_param.items():
                                experiment_key = f'{veh_key}-{env_key}-{sens_type_key}-{sens_key}-' \
                                                 f'{s_perf}-{s}-{cont_key}'
                                fn = os.path.join(basedir, str(veh_key), str(env_key), str(sens_type_key),
                                                  str(sens_key), str(s_perf),
                                                  str(s), str(cont_key), f'{experiment_key}.experiment.yaml')
                                dn = os.path.dirname(fn)
                                if not os.path.exists(dn):
                                    os.makedirs(dn)
                                if not os.path.exists(fn):
                                    params = sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, \
                                             fn, sens_key, s_perf, veh_key, env_key, cont_key, sens_type_key,
                                    to_run.append(params)
    nprocesses = 4
    n = len(to_run)
    print(f'Created {n} experiments to run')
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
    s_perf = params[10]
    veh_key = params[11]
    env_key = params[12]
    cont_key = params[13]
    sens_type_key = params[14]
    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key)
    danger = {
        "mean": str(performance.danger.mean), "var": str(performance.danger.var),
        "u95": str(performance.danger.u95),
        "l95": str(performance.danger.l95)
    }
    discomfort = {
        "mean": str(performance.discomfort.mean), "var": str(performance.discomfort.var),
        "u95": str(performance.discomfort.u95), "l95": str(performance.discomfort.l95)
    }
    ad_perf = {
        "danger": danger,
        "discomfort": discomfort,
        "speed": s,
        "sensor": sens_key,
        "sens_perf": s_perf,
        "dyn_perf": veh_key,
        "environment": env_key,
        "controller": cont_key,
        "sens_type": sens_type_key,
    }
    with open(fn, 'w') as f:
        yaml.dump(ad_perf, f, default_flow_style=False)
    print("Finished Experiment: ", fn)


if __name__ == '__main__':
    basedir = 'DB/output'
    generate(basedir)
    # TODO: make prper filenames for results and dpc models
    timestr = time.strftime("%Y%m%d-%H%M%S")
    fn_results = os.path.join(basedir, f'{timestr}.results.yaml')
    read_results(basedir, fn_results)
    fn_dpc_models = os.path.join(basedir, f'{timestr}_brake_control_models.yaml')
    write_bc_dpc(basedir, fn_dpc_models)
