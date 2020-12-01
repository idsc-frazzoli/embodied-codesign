import os
from decimal import Decimal
from glob import glob
from pathlib import Path
import re

import numpy as np
import yaml
import ruamel.yaml
from ruamel.yaml.scalarstring import DoubleQuotedScalarString as dq

from simulator.performance import PerformanceMetrics
from simulator.simulator import initialize_metrics, SimParameters, get_stats


def write_bc_dpc(basedir: str, models: str):
    yaml = ruamel.yaml.YAML()
    yaml.preserve_quotes = True
    with open('data/input/sensors.yaml') as file:
        sensors = yaml.load(file)

    with open('data/input/dyn_perf.yaml') as file:
        vehicles = yaml.load(file)

    with open('data/input/environment.yaml') as file:
        environment = yaml.load(file)

    with open('data/input/control_param.yaml') as file:
        control_param = yaml.load(file)

    filenames = list(glob(os.path.join(basedir, '*.results.yaml'), recursive=True))
    implementation = {}
    results = {}
    ms = dq("m/s")
    timeday = dq("`timeday")
    pax_km = dq("pax/km")
    s_str = dq("s")
    ms2 = dq("m/s^2")
    senprod = dq("`sen_prod")
    hz = dq("Hz")
    kgms = dq("kg*m/s")
    dimensionless = dq("dimensionless")
    f = [ms, timeday, pax_km, s_str]
    r = [ms, ms2, ms2, senprod, hz, hz, kgms, dimensionless]
    results["F"] = f
    results["R"] = r
    i = 1
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read())
        for data_key, d in data.items():
            speed = round(Decimal(d["speed"]), 2)
            # if d["dyn_perf"] !='sedan_s':
            #     continue
            speed_string = dq(f"{speed} m/s")
            env = environment[d["environment"]]["scenario_day_night"]
            env = dq(f"`timeday: {env}")
            density_p = environment[d["environment"]]["density_ped_km"]
            density_p_string = dq(f"{density_p} pax/km")
            dyn_perf = d["dyn_perf"]
            veh = vehicles[dyn_perf]
            a_max = veh["a_max_m_s2"]
            a_max_string = dq(f"{a_max} m/s^2")
            a_min = veh["a_min_m_s2"]*-1
            a_min_string = dq(f"{a_min} m/s^2")
            v_max = round(Decimal(veh["v_max_m_s"]), 2)
            v_max_string = dq(f"{v_max} m/s")
            sens_perf = d["sens_perf"]
            sens_per_string = dq(f"`sen_prod: {sens_perf}")
            sens = sensors[d["sensor"]]
            # if d["sensor"] == "puck":
            #     continue
            freq_sensor = sens["frequency_hz"]
            freq_sensor_string = dq(f"{freq_sensor} Hz")
            lat_sens = sens["latency_s"]
            lat_sens_string = dq(f"{lat_sens} s")
            cont_f = control_param[d["controller"]]["frequency_hz"]
            cont_th = control_param[d["controller"]]["prob_threshold"]
            # if cont_th != 7.0:
            #     continue
            # if cont_f < 3.0:
            #     continue
            cont_f_string = dq(f"{cont_f} Hz")
            danger_mean = round(Decimal(d["danger"]["mean"]), 5)
            danger_mean_string = dq(f"{danger_mean} kg*m/s")
            discomfort_mean = round(Decimal(d["discomfort"]["mean"]), 5)
            discomfort_mean_string = dq(f"{discomfort_mean} dimensionless")
            f_max = [speed_string, env, density_p_string, lat_sens_string]
            r_min = [v_max_string, a_max_string, a_min_string, sens_per_string, freq_sensor_string, cont_f_string,
                     danger_mean_string, discomfort_mean_string]
            implementation["model" + str(i)] = {"f_max": f_max, "r_min": r_min}
            i += 1
    results["implementations"] = implementation
    with open(models, 'w') as f:
        yaml.dump(results, f)


def read_results(basedir: str, result: str):
    filenames = [path for path in Path(basedir).rglob('*.experiment.yaml')]
    results = {}
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)
        stopped_key = "stopped_too_slow"
        if stopped_key in data:
            if data[stopped_key]:
                continue
        experiment_key = os.path.basename(fn).replace('.experiment.yaml', '')
        results[experiment_key] = data
    with open(result, 'w') as f:
        yaml.dump(results, f)


def read_results_from_single_exp(basedir: str, result: str, sp: SimParameters):
    filenames = [path for path in Path(basedir).rglob('*.experiment.yaml')]
    results = {}
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)
        stopped_key = "stopped_too_slow"
        if stopped_key in data:
            if data[stopped_key]:
                continue
        dn = os.path.dirname(fn)
        dn_s = os.path.join(dn, 'single_experiments')
        filenames_single = [path for path in Path(dn_s).rglob('*.yaml')]
        if len(filenames_single) != sp.nsims:
            raise ValueError
        # Initializing metrics
        discomfort, average_velocity, average_collision_momentum, collision = initialize_metrics(sp=sp)
        i = 0
        stopped_too_slow = False
        for fns in filenames_single:
            with open(fns, 'r') as f:
                data_s = yaml.load(f, Loader=yaml.FullLoader)

            collided_mom = data_s['collided']
            if collided_mom == str(None):
                collided_mom = None
            else:
                collided_mom = Decimal(collided_mom)
            av_vel = Decimal(data_s['average_velocity'])
            cont_eff = Decimal(data_s['control_effort'])

            if collided_mom is not None:
                collision[i] = 1
                average_collision_momentum[i] = collided_mom

            if collided_mom is not None:
                discomfort[i] = cont_eff + Decimal('5.0')
            else:
                discomfort[i] = cont_eff

            average_velocity[i] = av_vel

            if data['stopped_too_slow']:
                stopped_too_slow = True
                break
            i += 1
        discomfort_stat = get_stats(discomfort, cl=0.95, df=sp.nsims)
        p_collision = np.mean(collision)
        danger = average_collision_momentum * p_collision
        danger_stat = get_stats(danger, cl=0.95, df=sp.nsims)
        average_velocity_stat = get_stats(average_velocity, cl=0.95, df=sp.nsims)
        pm = PerformanceMetrics(danger=danger_stat, discomfort=discomfort_stat, average_velocity=average_velocity_stat,
                              stopped_too_slow=stopped_too_slow)

        danger = {
            "mean": str(round(pm.danger.mean, 5)), "var": str(round(pm.danger.var, 5)),
            "u95": str(round(pm.danger.u95, 5)),
            "l95": str(round(pm.danger.l95, 5))
        }
        discomfort = {
            "mean": str(round(pm.discomfort.mean, 5)), "var": str(round(pm.discomfort.var, 5)),
            "u95": str(round(pm.discomfort.u95, 5)), "l95": str(round(pm.discomfort.l95, 5))
        }


        data["danger"] = danger
        data["discomfort"] = discomfort
        with open(fn, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)

        experiment_key = os.path.basename(fn).replace('.experiment.yaml', '')
        results[experiment_key] = data
    with open(result, 'w') as f:
        yaml.dump(results, f)

    
def combine_mcdp(basedir: str, models: str):
    yaml = ruamel.yaml.YAML()
    yaml.preserve_quotes = True
    filenames = list(glob(os.path.join(basedir, '*.brake_control_models.yaml'), recursive=True))
    implementations_new = {}
    results = {}
    i = 1
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f)
        implementations = data["implementations"]
        for imp_key, imp in implementations.items():
            speed = imp["f_max"][0]
            speed_float = re.findall("\d+\.\d+", speed)
            speed_float_corr = round(Decimal('3.6')*Decimal(speed_float[0]),2)
            speed_string = dq(f"{speed_float_corr} m/s")
            f_max = [speed_string, imp["f_max"][1], imp["f_max"][2], imp["f_max"][3]]
            imp["f_max"] = f_max
            implementations_new["model" + str(i)] = imp
            i += 1

        results = {"implementations": implementations_new}
    with open(models, 'w') as f:
        yaml.dump(results, f)
