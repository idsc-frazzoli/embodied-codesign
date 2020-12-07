import argparse
import json
from decimal import Decimal
from typing import List

import matplotlib.pyplot as plt


def plot_all():
    with open('data/input/sensing_performance_curves.json') as file:
        curves = json.load(file)

    ds = Decimal(curves["Ace13gm_day_faster_rcnn1"]["ds"])
    max_distance = Decimal(curves["Ace13gm_day_faster_rcnn1"]["max_distance"])
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]

    for c_key, c in curves.items():
        data = c["fn"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=c_key)
        plt.ylabel('FNR')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig('data/output/fn-all.png')
    plt.show()
    plt.close()

    for c_key, c in curves.items():
        data = c["fp"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=c_key)
        plt.ylabel('FPR')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig('data/output/fp-all.png')
    plt.show()
    plt.close()

    for c_key, c in curves.items():
        data = c["accuracy"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=c_key)
        plt.ylabel('accuracy')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig('data/output/accuracy-all.png')
    plt.show()
    plt.close()


    plt.show()
    plt.savefig('data/output/fp.png')
    plt.close()

def plot_one(list_names: List[str]):
    with open('data/input/sensing_performance_curves.json') as file:
        curves = json.load(file)

    ds = Decimal(curves["Ace13gm_day_faster_rcnn1"]["ds"])
    max_distance = Decimal(curves["Ace13gm_day_faster_rcnn1"]["max_distance"])
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]

    for name in list_names:
        curve = curves[name]
        data = curve["fn"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=name)
        plt.ylabel('FNR')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig(f'data/output/fn.png')
    plt.show()
    plt.close()

    for name in list_names:
        curve = curves[name]
        data = curve["fp"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=name)
        plt.ylabel('FPR')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig(f'data/output/fp.png')
    plt.show()
    plt.close()

    for name in list_names:
        curve = curves[name]
        data = curve["accuracy"]
        data = [Decimal(s) for s in data]
        plt.plot(list_of_ds, data, label=name)
        plt.ylabel('accuracy')
        plt.xlabel('d in [m]')
        plt.legend(loc="upper left")
    plt.savefig(f'data/output/accuracy.png')
    plt.show()
    plt.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plotting sensing curves.')
    parser.add_argument('--sens_alg', nargs='+', type=str, default=['Ace13gm_day_faster_rcnn1'], help='The discretization of space in meters.')
    args = parser.parse_args()
    if args.sens_alg == "all":
        plot_all()
    else:
        plot_one(args.sens_alg)
