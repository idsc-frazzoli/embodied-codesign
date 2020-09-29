from decimal import Decimal
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

from controller.basic_controller import BasicController
from sensing.sensing_performance import SensingParameters, SensingPerformance, get_recall_precision, CameraSpecification
from simulator.simulator import simulate, SimParameters
from vehicle.state_estimation import Prior
from vehicle.vehicle import VehicleStats


if __name__ == '__main__':
    vs = VehicleStats(a_min=Decimal('-4.0'), a_max=Decimal('3.0'), v_nominal=Decimal(str(100/3.6)),
                      mass=Decimal('1000.0'), length=Decimal('4.0'), width=Decimal('2.0'))
    ds = Decimal('0.1')
    dt = Decimal('0.1')
    max_distance = Decimal('200.0')
    n = int(round(max_distance / ds))
    list_of_ds = [ds * Decimal(i) for i in range(n)]
    sens_param = SensingParameters(ds=ds, max_distance=max_distance, n=n,
                                   list_of_ds=list_of_ds, frequency=1*dt, latency= 1*dt)
    prior = Prior(density=Decimal('0.01'))
    recall, precision = get_recall_precision(str("faster_rcnn96"), n, ds)
    f_recall = interp1d(range(n), recall)
    f_precision = interp1d(range(n), precision)
    cam_basler = CameraSpecification(f=Decimal(str(4.0)), height=Decimal(str(4.55)), width=Decimal(str(6.17)),
                                 n_pixel_height=1920, n_pixel_width=1200)
    cam_flea = CameraSpecification(f=Decimal(str(4.0)), height=Decimal(str(6.2)), width=Decimal(str(7.6)),
                                     n_pixel_height=1384, n_pixel_width=1032)

    rfov_basler = cam_basler.get_vertical_fov(list_of_ds)
    resolution_basler = cam_basler.get_vertical_resolution(rfov_basler)
    recall_basler = [f_recall(float(resolution_basler[i])) for i in range(len(resolution_basler))]
    precision_basler = [f_precision(float(resolution_basler[i])) for i in range(len(resolution_basler))]
    fn_basler = 1 - np.array(recall_basler)
    fp_basler = 1 - np.array(precision_basler)

    rfov_flea = cam_flea.get_vertical_fov(list_of_ds)
    resolution_flea = cam_flea.get_vertical_resolution(rfov_flea)
    recall_flea = [f_recall(float(resolution_flea[i])) for i in range(len(resolution_flea))]
    precision_flea = [f_precision(float(resolution_flea[i])) for i in range(len(resolution_flea))]
    fn_flea = 1 - np.array(recall_flea)
    fp_flea = 1 - np.array(precision_flea)

    fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
    fig.suptitle('Recall, Precision, FPR, FNR dependent on distance/object height')
    ax1.plot(range(len(recall)), recall)
    ax1.set(xlabel='Object height in pixel', ylabel='Recall')
    ax2.plot(range(len(precision)), precision)
    ax2.set(xlabel='Object height in pixel', ylabel='Precision')
    ax3.plot(list_of_ds, recall_basler, color="red", label="Basler")
    ax3.plot(list_of_ds, recall_flea, color="blue", label="FLIR")
    ax3.legend(loc="upper right")
    ax3.set(xlabel='Distance in [m]', ylabel='Recall')
    ax4.plot(list_of_ds, precision_basler, color="red", label="Basler")
    ax4.plot(list_of_ds, precision_flea, color="blue", label="FLIR")
    ax4.legend(loc="upper right")
    ax4.set(xlabel='Distance in [m]', ylabel='Precision')
    ax5.plot(list_of_ds, fn_basler, color="red", label="Basler")
    ax5.plot(list_of_ds, fn_flea, color="blue", label="FLIR")
    ax5.legend(loc="lower right")
    ax5.set(xlabel='Distance in [m]', ylabel='FNR')
    ax6.plot(list_of_ds, fp_basler, color="red", label="Basler")
    ax6.plot(list_of_ds, fp_flea, color="blue", label="FLIR")
    ax6.legend(loc="lower right")
    ax6.set(xlabel='Distance in [m]', ylabel='FPR')
    plt.tight_layout()
    plt.savefig('Camera-Performance.png')
    plt.show()


    fn = [Decimal(str((ds*i/(max_distance*2))**4)) for i in range(n)]
    fp = [Decimal(str((ds * i / (max_distance * 3/2)) ** 4)) for i in range(n)]
    lsd = [Decimal(str(ds * i/(max_distance) + Decimal(0.5))) for i in range(n)]
    sens_perf = SensingPerformance(sp=sens_param)
    sens_perf.fn = fn
    sens_perf.fp = fp
    sens_perf.lsd = lsd
    controller = BasicController(prob_threshold=Decimal('0.2'), vs=vs, sp=sens_param,
                                 d_stop=Decimal('2.0'), t_react=Decimal('0.5'))

    sp = SimParameters(nsims=1, road_length=Decimal('500.0'), prior=prior, controller=controller,
                       sens_perf=sens_perf, dt=dt, sens_param=sens_param, vs=vs, seed=0,
                       wt=Decimal('1.0'))
    simulate(sp)

