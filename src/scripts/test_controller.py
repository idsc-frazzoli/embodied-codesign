import os
from decimal import Decimal
from multiprocessing import Pool

import yaml

from scripts.create_catalogue import simulate_and_write
from simulator.simulator import SimParameters
from utils.yaml_file_generation import read_results, write_bc_dpc

if __name__ == '__main__':

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

    with open('data/input/camera_curves.yaml') as file:
        camera_curves = yaml.load(file, Loader=yaml.FullLoader)

    sens_type_key = "camera"
    camera = sensors[sens_type_key]
    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=10, road_length=Decimal('500.0'), dt=Decimal(str(0.01)),
                       seed=0, do_animation=False)

    veh_key = "suv_m"
    dyn_perf = vehicles[veh_key]
    env_key = "07day_env"
    sens_key = "Ace13gm"
    sens = camera[sens_key]
    s_perf = sens["sens_perf"][0]
    sens_curves = camera_curves[s_perf]
    s = speed[4]
    env = environment[env_key]
    basedir = 'output'
    to_run = []  # list of parameters

    cont_key = "cont-th-0.1-ds-2.0-tr-0.1-f-100.0"
    cont = control_param[cont_key]
    experiment_key = f'{veh_key}-{env_key}-{sens_type_key}-{sens_key}-{s_perf}-{s}-{cont_key}'
    fn = os.path.join(basedir, f'{experiment_key}.experiment.yaml')
    params = sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, fn, sens_key, s_perf, veh_key, env_key, cont_key, sens_type_key,
    to_run.append(params)

    # for cont_key, cont in control_param.items():
    #     experiment_key = f'{veh_key}-{env_key}-{sens_type_key}-{sens_key}-{s_perf}-{s}-{cont_key}'
    #     fn = os.path.join(basedir, f'output/{experiment_key}.experiment.yaml')
    #     params = sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key, fn, sens_key, s_perf, veh_key, env_key, cont_key
    #     to_run.append(params)

    nprocesses = 4
    with Pool(processes=nprocesses) as pool:
        pool.map(simulate_and_write, to_run)

    basedir = "output"
    read_results(basedir, "output/test_controller.results.yaml")
    write_bc_dpc(basedir, "output/brake_control_models.yaml")
