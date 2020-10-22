# embodied-codesign

## RUN
Run tests:

`$ make test`

Run sensor curve generation where you can specify ds and maximum sensing range.
The default values are already shown in the command:

`$ make generate_sens_curves DS=0.05 SENSE_RANGE=500.0`

To plot the curves you can use the following command which plots all curves 
(if adding flag `PLOTSENSCURVE=Ace13gm` it plots only the specified sensor):

`$ make plot_sens_curves`

To generate the controller parameters you can use following command (`CONTROL_PARAM_PAMAX` is the percentage of your maximum acceleration which you want to use):

`$ make generate_control_param CONTROL_PARAM_FREQ='20 10' CONTROL_PARAM_TH='0.1 0.2' CONTROL_PARAM_DSTOP=3.0 
CONTROL_PARAM_PAMAX=0.5`

To generate the enviroment use the following command:

`$ make generate_sim_environment ENV_SIM_DENSITY='1 10 50' ENV_SIM_DAYNIGHT='day night'`

Then to run a a fixed paramter set you can run the following command, wehre many parameters can be set (instructions will come):

`$ make generate_animation`

To create a whole catalogue use the following command. Important to mention here is that with the flag 
`ALL=--all` it will run all the parameters in the yaml  files. If you want to fix some paramters then use e.g.
`VEHICLE_KEY='sedan_s sedan_m'`. 

`$ create_catalogue ALL=--all`

## Paremeters to set:

## Simulation
- `nsims`: number of simulations
- `road_length`: the total length of the road.
- `prior`: the prior wich is a poisson process and needs the appearance rate of obstacles
- `controller`: which controller to use for braking
- `seed`: seed for random number generation
- `ws`: the waiting time for the stopped vehicle in front of obstacle until obstacle disappears and the vehicle can move on.

### Vehicle
- `a_min`: minimum acceleration or maximum breaking
- `a_max`: maximum acceleration
- `v_nominal`: maximum nominal vehicle velocity
- `mass`: vehicle's mass
- `length`: vehicle's length (not used so far)
- `width`: vehicle's width (not used so far)

### Sensor
- `ds`: sensor measurement resolution
- `max_distance`: sensor range
- `fn`: the false negative rate curves need to be set or given
- `fp`: the false positive rate curves need to be set or given
- `lsd`: the distance standard deviation for the object detection
- `frequency`: the frequency of the sensing
- `latency`: the latency of the sensing

### Controller
- `prob_treshold`: the probability treshold when to brake
- `d_stop`: the distance to the obstacle we want the vehicle to stop or the additional distance to our braking distance
- `t_react`: the reaction time of the actuators



## Description of curves.yaml

Curves are just the 3 curves

```yaml

sens_perf0:
  FP:  [[1, 0.1], [2, 0.3], [100, 0.5]]
  FN:  [[1, 0.1], [2, 0.3], [100, 0.5]]
  acc: [[1, 0.1], [2, 0.3]]
  latency: 0.1
  frequency: 12

sens_perf1:
```

Assume < 200 m 

## Description of sensors.yaml

Sensor has a curve, but also cost, power, mass 

```yaml 
sensor0:
  perf: sens_perf0
  cost: 1000
  power:  12
  mass:  12
```


## Sensor model generation

This will produce `sensors.yaml` and `curves.yaml`.

How? ...




## Description of speeds.yaml

```yaml
- 10
- 15
- 20
- 25
- 30
- Let's do from 10 to 50 mph (in km/h !)

```

## Description of environment.yaml

```yaml

IDenv: 
  density: 9 # ped/km
  scenario: day # day, night
```



## Generation of "co-design" catalogue

``` 

    create_catalogue()

```

Given 

- `curves.yaml`, `dyn_perf.yaml`, `speeds.yaml`, `environment.yaml`

Creates:

- `brake_control.yaml`


1. Open `curves.yaml` and read the N sensor_performance structures
2. Open `dyn_perf.yaml` and read the M dynamic performance structuers.
3. Read `speeds.yaml`
3. Read `environment.yaml`

1. For each sens_perf
2. For each dyn_perfr
3. For each cruise speed
4. For each pedestrian_density 

Run  `control_enum(cruise_speed, dyn_perf, sens_perf, environment)`
and obtain a list of `{(control_param, danger, discomfort)}`

Output the following catalogue entry in `brake_control.mcdp`  

```yaml

IDiteration:
    control_params: 
    sens_perf: ...
    dyn_perf:
    cruise_speed: 
    danger: ...     
    discomfort: ...


```



##  Controller enumeration step

```

def control_enum(cruise_speed, dyn_perf, sens_perf, environment) -> {[control_params, danger, discomfort]} 

```

Given: 

- `cruise_speed`
- `environment prior` (density of pedestrians)
- `dynamic_performance`
- `sensor_performance` (FP, FN, accuracy, frequency, latency)

Iterate over:

- `control_param`: `prob_treshold`, `d_stop` 

call `sim_level1(cruise_speed, dyn_perf, sens_perf, control_param, pedestrian_density)`
and compute:

- average danger
- average discomfort 

[Also report:
- average speed
- time you take]


Now report the `minima` of the relation.
In general something like:

   control param1 ,  danger1  , discomfort1
   control param2 ,  danger2  , discomfort2
   control param3 ,  danger3  , discomfort3
   
For example it could be that  danger1<danger2 but discomfort2<discomfort1, so here we have
multiple solutions. 


##  "Simulator" level 1


```

def sim_level1(cruise_speed, dyn_perf, sens_perf, control_param, pedestrian_density) -> [danger, discomfort] 

```


Runs `sim_level0` for N simulations (or adaptive threshold).

Computes danger by averaging over each sample.



##  "Simulator" level 0

```

def sim_level0(cruise_speed, dyn_perf, sens_perf, control_param, pedestrian_density) -> [hazard, discomfort] 

```

Given: 

- `cruise_speed`
- `dynamic_performance`
- `sensor_performance`
- `controller parameters`
- `environment prior` (density of pedestrians)

Simulate everything for N kms. 

Compute:

- Hazard: Whether you collided, velocity collided. (Hazard)
- Discomfort.

[Also report:
- average speed
- time you take]


#  For Gioele

## Generation of curves poset

Given `curves.yaml`, create `curves.mcdp_poset`:


```

sens_perf0

sens_perf1

sens_perf2 <= sens_perf3

...

```

## Generation of sensor catalogue

Given `sensors.yaml`, put it in co-design catalogue format:

```
catalogue {


      sensor_perf <- |sensor0| -> cost, power, mass

}
```
 
## Generation of brake_control catalogue


Generate a catalogue of this:

```
cruise_speed, pedestrian_density  <-| IDiteration |-> sens_perf, dyn_perf, danger, discomfort, frequency computation
```
