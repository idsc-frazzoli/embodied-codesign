from decimal import Decimal

import yaml

from simulator.simulator import SimParameters, simulate

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


    camera = sensors["camera"]
    speed = cruise_speeds["speeds"]

    sp = SimParameters(nsims=1, road_length=Decimal('500.0'), dt=Decimal(str(0.01)),
                       seed=0, do_animation=True)

    dyn_perf = vehicles["suv_m"]
    sens = camera["Ace13gm"]
    sens_perf = sens["sens_perf"][0]
    sens_curves = camera_curves[sens_perf]
    s = speed[6]
    s = 80
    env = environment["07day_env"]
    cont = control_param["cont-treshold-0.1-d_stop-1.5-t_react-0.1-freq-0.01"]
    performance = simulate(sp, dyn_perf, sens, sens_curves, s, env, cont, experiment_key="test2")


