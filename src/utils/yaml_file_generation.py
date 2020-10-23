import os
from decimal import Decimal
from glob import glob
from pathlib import Path

import yaml
import ruamel.yaml
from ruamel.yaml.scalarstring import DoubleQuotedScalarString as dq


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
    i = 1
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read())
        for data_key, d in data.items():
            speed = round(Decimal(d["speed"]) / Decimal(3.6), 2)
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
            freq_sensor = sens["frequency_hz"]
            freq_sensor_string = dq(f"{freq_sensor} Hz")
            lat_sens = sens["latency_s"]
            lat_sens_string = dq(f"{lat_sens} s")
            cont_f = control_param[d["controller"]]["frequency_hz"]
            cont_f_string = dq(f"{cont_f} Hz")
            danger_mean = round(Decimal(d["danger"]["mean"]), 2)
            danger_mean_string = dq(f"{danger_mean} kg*m/s")
            discomfort_mean = round(Decimal(d["discomfort"]["mean"]), 2)
            discomfort_mean_string = dq(f"{discomfort_mean} dimensionless")
            f_max = [speed_string, env, density_p_string, lat_sens_string]
            r_min = [v_max_string, a_max_string, a_min_string, sens_per_string, freq_sensor_string, cont_f_string,
                     danger_mean_string, discomfort_mean_string]
            implementation["model" + str(i)] = {"f_max": f_max, "r_min": r_min}
            i += 1
    results = {"implementations": implementation}
    with open(models, 'w') as f:
        yaml.dump(results, f)


def read_results(basedir: str, result: str):
    filenames = [path for path in Path(basedir).rglob('*.experiment.yaml')]
    results = {}
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read(), Loader=yaml.FullLoader)
        experiment_key = os.path.basename(fn).replace('.experiment.yaml', '')
        results[experiment_key] = data
    with open(result, 'w') as f:
        yaml.dump(results, f)
