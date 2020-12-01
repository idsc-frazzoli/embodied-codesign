import argparse
import os
from decimal import Decimal
from glob import glob

import numpy as np
import ruamel.yaml
import matplotlib.pyplot as plt

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generating MCDP files.')
    parser.add_argument('--basedir', type=str, default='DB', help='Controller name.')
    parser.add_argument('--simversion', type=str, default='simulation_v.1.2_sensors_nsims_50', help='Controller name.')

    args = parser.parse_args()

    fn_results = os.path.join(args.basedir, args.simversion, 'catalogue', f'{args.simversion}.results.yaml')
    dn = os.path.dirname(fn_results)

    filenames = list(glob(os.path.join(dn, '*.results.yaml'), recursive=True))
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


    freq_list = []
    freq_list_5 = []
    freq_list_20 = []
    dange_list = []
    dange_list_5 = []
    dange_list_20 = []
    discomfort_list = []
    discomfort_list_5 = []
    discomfort_list_20 = []
    speed_list = []
    speed_list_5 = []
    speed_list_20 = []
    sensor_list = []
    sens_name = 'Pointgrey'
    for fn in filenames:
        with open(fn) as f:
            data = yaml.load(f.read())
        for data_key, d in data.items():
            sens_perf = d["sens_perf"]
            sens = d["sensor"]
            # if sens != sens_name:
            #     continue
            if d['dyn_perf'] != 'sedan_s':
                continue
            cont_f = control_param[d["controller"]]["frequency_hz"]
            cont_th = control_param[d["controller"]]["prob_threshold"]
            if cont_th != 7.0:
                continue

            danger_mean = round(Decimal(d["danger"]["mean"]), 5)

            # if danger_mean != 0.0:
            #     continue

            # if d['speed'] != 13.41:
            #     continue

            discomfort_mean = round(Decimal(d["discomfort"]["mean"]), 5)
            freq_list.append(float(cont_f))
            dange_list.append(float(danger_mean))
            discomfort_list.append(float(discomfort_mean))
            speed_list.append(d['speed'])
            sensor_list.append(d["sensor"])

    freq_list = np.array(freq_list)
    dange_list = np.array(dange_list)
    discomfort_list = np.array(discomfort_list)
    discomfort_list_min = np.min(discomfort_list)
    argmin_list = np.argmin(discomfort_list)
    print(discomfort_list_min)
    mask = np.array(discomfort_list) == discomfort_list_min
    print(np.array(sensor_list)[mask])
    print(freq_list[mask])
    color = np.where(mask, 'red', 'blue')
    mask_v_8 = np.array(speed_list) == 8.93
    mask_v_13 = np.array(speed_list) == 13.41
    mask_v_17 = np.array(speed_list) == 17.88
    mask_v_22 = np.array(speed_list) == 22.35
    freq_v_8 = freq_list[mask_v_8]
    freq_v_13 = freq_list[mask_v_13]
    freq_v_17 = freq_list[mask_v_17]
    freq_v_22 = freq_list[mask_v_22]
    danger_v_8 = dange_list[mask_v_8]
    danger_v_13 = dange_list[mask_v_13]
    danger_v_17 = dange_list[mask_v_17]
    danger_v_22 = dange_list[mask_v_22]
    disc_v_8 = discomfort_list[mask_v_8]
    disc_v_13 = discomfort_list[mask_v_13]
    disc_v_17 = discomfort_list[mask_v_17]
    disc_v_22 = discomfort_list[mask_v_22]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(freq_v_8, danger_v_8, disc_v_8, c=disc_v_8, marker='o', cmap='viridis', linewidth=0.5, label='speed 8.93 m/s')
    ax.scatter(freq_v_13, danger_v_13, disc_v_13, c=disc_v_13, marker='x', cmap='viridis', linewidth=0.5, label='speed 13.41 m/s')
    ax.scatter(freq_v_17, danger_v_17, disc_v_17, c=disc_v_17, marker='s', cmap='viridis', linewidth=0.5, label='speed 17.88 m/s')
    ax.scatter(freq_v_22, danger_v_22, disc_v_22, c=disc_v_22, marker='v', cmap='viridis', linewidth=0.5, label='speed 22.35 m/s')
    ax.set_xlabel(r"Frequency in Hz")
    ax.set_ylabel('Danger in kg*m/s')
    ax.set_zlabel('Discomfort')
    plt.legend(loc='upper left', numpoints=1, ncol=1, fontsize=8, bbox_to_anchor=(0, 0))
    plt.title(f'Controller freqency vs. Danger vs. Discomfort for Sedan with {sens_name}')
    plt.savefig(f'data/output/fdd-{sens_name}.png')
    # plt.show()
    plt.close()
    # plt.scatter(np.array(freq_list), np.array(dange_list))
    # plt.ylabel('Danger')
    # plt.xlabel('Frequency')
    # plt.show()
    # plt.close()
    # plt.scatter(np.array(freq_list), np.array(discomfort_list))
    # plt.ylabel('Discomfort')
    # plt.xlabel('Frequency')
    # plt.show()
    # plt.close()
    # plt.scatter(np.array(dange_list), np.array(discomfort_list))
    # plt.ylabel('Discomfort')
    # plt.xlabel('Danger')
    # plt.show()
    # plt.close()
    # plt.scatter(np.array(speed_list), np.array(discomfort_list))
    # plt.ylabel('Discomfort')
    # plt.xlabel('speed')
    # plt.show()
    # plt.close()
    # plt.scatter(np.array(speed_list), np.array(dange_list))
    # plt.ylabel('Danger')
    # plt.xlabel('speed')
    # plt.show()

    # plt.scatter(np.array(speed_list_5), np.array(discomfort_list_5), color="blue", label="5")
    # plt.scatter(np.array(speed_list_20), np.array(discomfort_list_20), color="red", label="50")
    # plt.ylabel('Discomfort')
    # plt.xlabel('speed')
    # plt.legend(loc='upper left')
    # plt.show()
    # plt.close()
    # plt.scatter(np.array(speed_list_5), np.array(dange_list_5), color="blue", label="5")
    # plt.scatter(np.array(speed_list_20), np.array(dange_list_20), color="red", label="50")
    # plt.ylabel('Danger')
    # plt.xlabel('speed')
    # plt.legend(loc='upper left')
    # plt.show()
