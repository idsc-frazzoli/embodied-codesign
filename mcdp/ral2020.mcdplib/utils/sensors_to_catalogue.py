import ruamel.yaml
from ruamel.yaml.scalarstring import DoubleQuotedScalarString as dq


def sensors_to_cat(fileName):
    yaml = ruamel.yaml.YAML()
    yaml.preserve_quotes = True
    yaml.representer.ignore_aliases = lambda *data: True

    implementation_c1 = {}
    implementation_c2 = {}
    implementation_c3 = {}
    implementation_c4 = {}
    implementation_l1 = {}
    implementation_l2 = {}
    implementation_l3 = {}
    implementation_l4 = {}
    implementation_l5 = {}
    implementation_l6 = {}

    with open(fileName) as f:
        data = yaml.load(f.read())
    for data_key, d in data.items():
        env_1 = dq(f"`timeday: day")
        env_2 = dq(f"`timeday: night")
        name = data_key
        sensor = data[data_key]
        type = sensor["type"]
        cost = sensor["cost_chf"]
        cost_string = dq(f"{cost} CHF")
        mass = sensor["mass_g"]
        mass_string = dq(f"{mass} g")
        power = sensor["power_w"]
        power_string = dq(f"{power} W")
        freq_sensor = sensor["frequency_hz"]
        freq_sensor_string = dq(f"{freq_sensor} Hz")
        lat_sens = sensor["latency_s"]
        lat_sens_string = dq(f"{lat_sens} s")
        algorithm = sensor["algorithm"]
        algos_day = algorithm["day"]
        algos_night = algorithm["night"]
        if type == "camera":
            sens_per_string_1 = dq(f"`sen_prod: {name}_{algos_day[0]}")
            sens_per_string_2 = dq(f"`sen_prod: {name}_{algos_day[1]}")
            sens_per_string_3 = dq(f"`sen_prod: {name}_{algos_night[0]}")
            sens_per_string_4 = dq(f"`sen_prod: {name}_{algos_night[1]}")
            f_max_c1 = [sens_per_string_1, freq_sensor_string, env_1]
            f_max_c2 = [sens_per_string_2, freq_sensor_string, env_1]
            f_max_c3 = [sens_per_string_3, freq_sensor_string, env_2]
            f_max_c4 = [sens_per_string_4, freq_sensor_string, env_2]
            r_min_c = [cost_string, mass_string, power_string, lat_sens_string]
            implementation_c1[name + algos_day[0]] = {"f_max": f_max_c1, "r_min": r_min_c}
            implementation_c2[name + algos_day[1]] = {"f_max": f_max_c2, "r_min": r_min_c}
            implementation_c3[name + algos_night[0]] = {"f_max": f_max_c3, "r_min": r_min_c}
            implementation_c4[name + algos_night[1]] = {"f_max": f_max_c4, "r_min": r_min_c}
        elif type == "lidar":
            sens_per_string_1 = dq(f"`sen_prod: {name}_{algos_day[0]}")
            sens_per_string_2 = dq(f"`sen_prod: {name}_{algos_day[1]}")
            sens_per_string_3 = dq(f"`sen_prod: {name}_{algos_day[2]}")
            sens_per_string_4 = dq(f"`sen_prod: {name}_{algos_night[0]}")
            sens_per_string_5 = dq(f"`sen_prod: {name}_{algos_night[1]}")
            sens_per_string_6 = dq(f"`sen_prod: {name}_{algos_night[2]}")
            f_max_l1 = [sens_per_string_1, freq_sensor_string, env_1]
            f_max_l2 = [sens_per_string_2, freq_sensor_string, env_1]
            f_max_l3 = [sens_per_string_3, freq_sensor_string, env_1]
            f_max_l4 = [sens_per_string_4, freq_sensor_string, env_2]
            f_max_l5 = [sens_per_string_5, freq_sensor_string, env_2]
            f_max_l6 = [sens_per_string_6, freq_sensor_string, env_2]
            r_min_l = [cost_string, mass_string, power_string, lat_sens_string]
            implementation_l1[name + algos_day[0]] = {"f_max": f_max_l1, "r_min": r_min_l}
            implementation_l2[name + algos_day[1]] = {"f_max": f_max_l2, "r_min": r_min_l}
            implementation_l3[name + algos_day[2]] = {"f_max": f_max_l3, "r_min": r_min_l}
            implementation_l4[name + algos_night[0]] = {"f_max": f_max_l4, "r_min": r_min_l}
            implementation_l5[name + algos_night[1]] = {"f_max": f_max_l5, "r_min": r_min_l}
            implementation_l6[name + algos_night[2]] = {"f_max": f_max_l6, "r_min": r_min_l}

    impl = [implementation_c1, implementation_c2, implementation_c3, implementation_c4,
            implementation_l1, implementation_l2, implementation_l3, implementation_l4,
            implementation_l5, implementation_l6]
    results = {"F": [dq("`sen_prod"), dq("Hz"), dq("`timeday")],
               "R": [dq("CHF"), dq("g"), dq("W"), dq("us")],
               "implementations": impl
               }

    intro_F = {"F": [dq("`sen_prod"), dq("Hz"), dq("`timeday")]}
    intro_R = {"R": [dq("CHF"), dq("g"), dq("W"),dq("us")]}

    with open('long_sensing_prod.dpc.yaml', 'w') as f:
        #yaml.dump(intro_F, f)
        #yaml.dump(intro_R, f)
        yaml.dump(results, f)


sensorsToCat('sensors.yaml')