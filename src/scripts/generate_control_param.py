import os

import yaml

if __name__ == '__main__':
    p_treshold_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    d_stop_list = [1, 1.5, 2, 2.5, 3]
    t_react_list = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    control_param = {}
    id = 1
    for p_treshold in p_treshold_list:
        for d_stop in d_stop_list:
            for t_react in t_react_list:
                param = {"prob_threshold": p_treshold, "d_stop": d_stop, "t_react": t_react}
                control_param["cont" + str(id)] = param
                id += 1

    with open('data/input/control_param.yaml', 'w') as file:
        documents = yaml.dump(control_param, file, default_flow_style=False)
