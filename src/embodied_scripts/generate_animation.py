import argparse
import json
import os
from decimal import Decimal

import yaml

from simulator.simulator import SimParameters, simulate


def write_store_param_file(args, param_file_name, alg_sens_curv):
    params = vars(args)
    params["ds"] = alg_sens_curv["ds"]
    params["max_distance"] = alg_sens_curv["max_distance"]
    with open(param_file_name, 'w') as f:
        yaml.dump(params, f, default_flow_style=False)


def generate_animation(args):
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file, Loader=yaml.FullLoader)

    with open('data/input/sensing_performance_curves.json') as file:
        sens_curves = json.load(file)

    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file, Loader=yaml.FullLoader)

    sens = sensors[args.sensor]
    dyn_perf = vehicles[args.vehicle]

    if args.environment == "none":
        env = {"density_ped_km": float(args.env_density), "scenario_day_night": args.env_day_night}
        env_key = f'{args.env_density}{env["scenario_day_night"]}_env'
    else:
        env = environment[args.environment]
        env_key = args.environment

    sens_perf_curv_key = f'{args.sensor}_{env["scenario_day_night"]}_{args.algorithm}'
    alg_sens_curv = sens_curves[sens_perf_curv_key]
    speed = Decimal(args.speed)

    if args.controller == "none":
        d_stop = Decimal(str(args.control_d_stop))
        p_a_max = Decimal(str(args.control_percentage_amax))
        p_treshold = Decimal(args.control_treshold)
        cont_freq = Decimal(args.control_freq)
        controller = {"d_stop_m": float(d_stop), "frequency_hz": float(cont_freq), "prob_threshold": float(p_treshold),
                      "percentage_amax": p_a_max}
        cont_key = f'cont_th_{p_treshold}_ds_{d_stop}_f_{cont_freq}_pamx_{p_a_max}'
    else:
        controller = control_param[args.controller]
        cont_key = args.controller

    if args.do_animation:
        do_animation = True
    else:
        do_animation = False

    sp = SimParameters(nsims=args.nsims, road_length=Decimal(args.road_length), dt=Decimal(args.dt),
                       seed=args.seed, do_animation=do_animation, add_object_at=args.add_object_at,
                       stop_time=Decimal(args.stop_time), discomfort_penalty=Decimal(args.discomfort_penalty))

    experiment_key = f'{args.vehicle}-{env_key}-{args.sensor}-{sens_perf_curv_key}-{str(speed)}-{cont_key}'
    fn = os.path.join(args.basedir, str(args.vehicle), str(env_key), str(args.sensor),
                      str(sens_perf_curv_key), str(speed), str(cont_key), f'{experiment_key}.experiment.yaml')
    dn = os.path.dirname(fn)
    if not os.path.exists(dn):
        os.makedirs(dn)
    param_file_name = os.path.join(dn, f'{experiment_key}.parameters.yaml')
    write_store_param_file(args, param_file_name, alg_sens_curv)

    performance = simulate(sp=sp, dyn_perf=dyn_perf, sens=sens, sens_curves=alg_sens_curv, s=speed,
             env=env, cont=controller, experiment_key=experiment_key, file_directory=dn)

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
        "speed": speed, "sensor": args.sensor, "sens_perf": sens_perf_curv_key, "dyn_perf": args.vehicle,
        "environment": env_key,
        "controller": cont_key,
        "stopped_too_slow": performance.stopped_too_slow,
    }

    with open(fn, 'w') as f:
        yaml.dump(ad_perf, f, default_flow_style=False)
    print(f'Finished experiment: {experiment_key}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating sensing curves.')
    parser.add_argument('--nsims', type=int, default=10, help='Number of simulations per parameter set.')
    parser.add_argument('--dt', type=str, default='0.01', help='Simulation sampling time in seconds.')
    parser.add_argument('--road_length', type=str, default='500.0', help='Road length in meters.')
    parser.add_argument('--do_animation', default=False, action="store_true", help='Flag for creating an animation.')
    parser.add_argument('--do_not_animation', default=True, action="store_true", help='Flag for creating an animation.')
    parser.add_argument('--seed', type=int, default=0, help='Seed for simulation.')
    parser.add_argument('--sensor', type=str, default='Ace13gm', help='Sensor name from sensor.yaml file.')
    parser.add_argument('--vehicle', type=str, default='sedan_s', help='Dynamic performance of the vehicle from dyn_perf.yaml file.')
    parser.add_argument('--environment', type=str, default='env_d_5.0_day', help='Environment from environment.yaml file.')
    parser.add_argument('--env_density', type=str, default='7', help='Environment density in pedestrian per kilometer.')
    parser.add_argument('--env_day_night', type=str, default='day', help='Environment day or night.')
    parser.add_argument('--algorithm', type=str, default='faster_rcnn1', help='Algorithm for object detection '
                                                                              'from sensors.yaml file. Check if lidar or cam!!.')
    parser.add_argument('--speed', type=str, default='10.0', help='Cruise speed in meter per seconds.')
    parser.add_argument('--control_freq', type=str, default='20.0', help='Controller frequency in Hz.')
    parser.add_argument('--control_treshold', type=str, default='0.1', help='Treshohld for controller.')
    parser.add_argument('--controller', type=str, default='cont_th_0.4_ds_3.0_f_20.0_pamx_0.5', help='Controller name.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--add_object_at', type=str, default='20.0', help='Add additional object at a specif '
                                                                          'distance for testing. If no additional write none.')
    parser.add_argument('--control_d_stop', type=float, default=3.0, help='Additional distance to stop in front of object.')
    parser.add_argument('--control_percentage_amax', type=float, default=0.5,
                        help='The procentage of a_max usage.')
    parser.add_argument('--stop_time', type=str, default='400.0', help='Stop time of simulation if running too long.')
    parser.add_argument('--discomfort_penalty', type=str, default='5.0', help='The discomfort penalty when collided.')
    args = parser.parse_args()

    generate_animation(args)



