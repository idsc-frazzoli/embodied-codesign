import os
from decimal import Decimal
import time
from glob import glob
from multiprocessing import Pool

import yaml

from simulator.simulator import SimParameters, simulate
from zuper_commons.fs import locate_files

def generate(basedir: str):
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/speeds.yaml') as file:
        cruise_speeds = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/curves.yaml') as file:
        curves = yaml.load(file, Loader=yaml.FullLoader)

    camera = sensors["camera"]
    speed = cruise_speeds["speeds"]

    e = 0.1 / 100
    p = 0.01 / 100
    z = 1.96

    nsims = int(z ** 2 * (1 - p) * p / e ** 2)



    sp = SimParameters(nsims=nsims, road_length=Decimal('500.0'), dt=Decimal(str(0.01)), seed=0,
                       wt=Decimal('1.0'), do_animation=False)



    # results = {}
    # timestr = time.strftime("%Y%m%d-%H%M%S")
    # with open('data/output/ad-performance-' + timestr + '.yaml', 'w') as file:

    # first see what needs to be run
    to_run = [] # list of parameters
    for veh_key, dyn_perf in vehicles.items():
        for cam_key, sens in camera.items():
            sens_perf_key = sens["sens_perf"]
            sens_curves = curves[sens_perf_key]
            for s in speed:
                for env_key, env in environment.items():
                    for cont_key, cont in control_param.items():
                        experiment_key = f'{veh_key}-{cam_key}-{s}-{env_key}-{cont_key}'
                        fn = os.path.join(basedir, f'{experiment_key}.experiment.yaml')
                        if not os.path.exists(fn):
                            params = sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, fn, cam_key, veh_key, env_key, cont_key
                            to_run.append(params)

    nprocesses = 10
    with Pool(processes=nprocesses) as pool:
        pool.map(simulate_and_write, to_run)

def simulate_and_write(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, fn, cam_key, veh_key, env_key, cont_key):
    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key)
    ad_perf = {
        "danger": str(performance.danger),
        "discomfort": str(performance.discomfort),
        "speed": s, "sensor": cam_key, "dyn_perf": veh_key,
        "environment": env_key,
        "controller": cont_key,
    }
    yaml.dump(ad_perf, fn, default_flow_style=False)

def read_results(basedir: str, result: str):
    filenames = list(glob(os.path.join(basedir, '*.experiment.yaml'), recursive=True))
    results = {}
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read())
        experiment_key = os.path.basename(fn).replace('.experiment.yaml', '')
        results[experiment_key] = data
    with open(result, 'w') as f:
        yaml.dump(results, f)

