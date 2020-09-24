# embodied-codesign

## TODO
- stopping criterium for simulation. So far only when it hits an object. Does the objects disappear when safely stopped infront of them? To the objects have a time schedule?
- So far only simple vehicle kinematics used. Maybe we could add dynamics with additional vehicle parameters such as tires or engine.
- Testing
- additional controllers
- false positive and false negative rates curve generation
- distance standard deviation curve for object detction (so far linear increasing with distance)
- statistics (number of false positives, false negatives, ....)

## Uncertainty
- not sure about the observation update for the bayes filter.
- not sure about the implementation of the false positive detections
- does prior of the bayes filter need to be normalized?

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
