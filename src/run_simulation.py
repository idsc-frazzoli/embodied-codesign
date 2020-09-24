from decimal import Decimal

from controller.basic_controller import BasicController
from sensing.sensing_performance import SensingParameters, SensingPerformance
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

