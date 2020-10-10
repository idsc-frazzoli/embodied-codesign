import yaml
import numpy as np

class curve:
    def __init__(self):
        self.sen = []
        self.fn = []
        self.fp = []
        self.acc = []

def get_curves(fileName):
    sensor_names = []
    all_curves = []
    with open(fileName) as f:
        sensors_file = yaml.load(f, Loader=yaml.FullLoader)
        for item, doc in sensors_file.items():
            sensor_names.append(item)

        for sensor in sensor_names:
            curves = sensors_file[sensor]
            acc = curves["accuracy"]
            fn = curves["fn"]
            fp = curves["fp"]
            s = curve()
            s.sen = sensor
            s.fn = fn
            s.fp = fp
            s.acc = acc
            all_curves.append(s)
    return all_curves

def compare_arrays(a,b):
    " assuming same size"
    assert len(a)==len(b)
    pairwise_comp = []
    for i in range(len(a)):
        aa = float(a[i])
        bb = float(b[i])
        if aa <= 1e-9:
            aa = 0
        if bb <= 1e-9:
            bb = 0
        is_smaller = (aa >= bb)
        pairwise_comp.append(is_smaller)
    if np.all(pairwise_comp):
        ret = 1
    else:
        ret = 0
    return ret

def compare_curves(array_curves):
    fn_order = np.zeros((len(array_curves), len(array_curves)))
    fp_order = np.zeros((len(array_curves), len(array_curves)))
    acc_order = np.zeros((len(array_curves), len(array_curves)))

    for id_s, s in enumerate(array_curves):
        for id_t, t in enumerate(array_curves):
            if (id_s != id_t):
                if compare_arrays(s.fn, t.fn):
                    fn_order[id_s][id_t] = 1
                if compare_arrays(s.fp, t.fp):
                    fp_order[id_s][id_t] = 1
                if compare_arrays(s.acc, t.acc):
                    acc_order[id_s][id_t] = 1
    return fn_order, fp_order, acc_order

def produce_poset_fn(fn_order, array_curves):
    poset_fn = open("poset_fn.mcdp_poset", 'w')
    poset_fn.write("add_bottom poset {\n")
    for a in array_curves:
        poset_fn.write(a.sen + ' ')
    poset_fn.write("\n")
    for i in range(len(fn_order)):
        for j in range(len(fn_order)):
            if (i!=j):
                if (fn_order[i][j]):
                    poset_fn.write(array_curves[i].sen + " <= " + array_curves[j].sen + "\n")
    poset_fn.write("}")

def produce_poset_fp(fp_order, array_curves):
    poset_fp = open("poset_fp.mcdp_poset", 'w')
    poset_fp.write("add_bottom poset {\n")
    for a in array_curves:
        poset_fp.write(a.sen + ' ')
    poset_fp.write("\n")
    for i in range(len(fp_order)):
        for j in range(len(fp_order)):
            if (i != j):
                if (fp_order[i][j]):
                    poset_fp.write(array_curves[i].sen + " <= " + array_curves[j].sen + "\n")
    poset_fp.write("}")

def produce_poset_acc(acc_order, array_curves):
    poset_acc = open("poset_acc.mcdp_poset", 'w')
    poset_acc.write("add_bottom poset {\n")
    for a in array_curves:
        poset_acc.write(a.sen + ' ')
    poset_acc.write("\n")
    for i in range(len(acc_order)):
        for j in range(len(acc_order)):
            if (i != j):
                if (acc_order[i][j]):
                    poset_acc.write(array_curves[i].sen + " <= " + array_curves[j].sen + "\n")
    poset_acc.write("}")

def produce_prod_poset(fp_order, fn_order, acc_order,array_curves):
    poset_prod = open("sen_prod.mcdp_poset", 'w')
    poset_prod.write("add_bottom poset {\n")
    for a in array_curves:
        poset_prod.write(a.sen + ' ')
    poset_prod.write("\n")
    for i in range(len(fp_order)):
        for j in range(len(fp_order)):
            if(i!=j):
                if (fp_order[i][j] and fn_order[i][j] and acc_order[i][j]):
                    poset_prod.write(array_curves[i].sen + " <= " + array_curves[j].sen + "\n")
    poset_prod.write("}")

curves_here = get_curves('all_curves.yaml')
fn, fp, acc = compare_curves(curves_here)
produce_prod_poset(fp,fn,acc,curves_here)
#produce_poset_fn(fn, curves_here)
#produce_poset_fp(fp, curves_here)
#produce_poset_acc(acc, curves_here)

#produce_poset_acc(acc, curves_here)

