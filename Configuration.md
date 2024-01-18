# Configuration of CARLA OSI Service

## Interface

### Filled Fields in OSI3 SensorView

* sensor_view.sensor_id
<!-- * sensor_view.mounting_position-->
* sensor_view.global_ground_truth.timestamp
* sensor_view.global_ground_truth.host_vehicle_id
* sensor_view.global_ground_truth.stationary_object.id
* sensor_view.global_ground_truth.stationary_object.base.dimension
* sensor_view.global_ground_truth.stationary_object.base.position
* sensor_view.global_ground_truth.stationary_object.base.orientation
* sensor_view.global_ground_truth.stationary_object.classification.type
* sensor_view.global_ground_truth.stationary_object.model_reference
* sensor_view.global_ground_truth.moving_object.id
* sensor_view.global_ground_truth.moving_object.base.position
* sensor_view.global_ground_truth.moving_object.base.orientation
* sensor_view.global_ground_truth.moving_object.base.orientation_rate
* sensor_view.global_ground_truth.moving_object.base.velocity
* sensor_view.global_ground_truth.moving_object.base.acceleration
* sensor_view.global_ground_truth.moving_object.base.dimension
* sensor_view.global_ground_truth.moving_object.type
* sensor_view.global_ground_truth.moving_object.assigned_lane_id
* sensor_view.global_ground_truth.moving_object.vehicle_attributes.radius_wheel
* sensor_view.global_ground_truth.moving_object.vehicle_attributes.number_wheels
* sensor_view.global_ground_truth.moving_object.vehicle_attributes.bbcenter_to_rear
* sensor_view.global_ground_truth.moving_object.vehicle_attributes.bbcenter_to_front
* sensor_view.global_ground_truth.moving_object.vehicle_attributes.ground_clearance
* sensor_view.global_ground_truth.moving_object.vehicle_classification.type
* sensor_view.global_ground_truth.moving_object.vehicle_classification.light_state
* sensor_view.global_ground_truth.moving_object.vehicle_classification.has_trailer
* sensor_view.global_ground_truth.moving_object.model_reference
* sensor_view.global_ground_truth.traffic_sign.id
* sensor_view.global_ground_truth.traffic_sign.main_sign.base.dimension
* sensor_view.global_ground_truth.traffic_sign.main_sign.base.position
* sensor_view.global_ground_truth.traffic_sign.main_sign.base.orientation
* sensor_view.global_ground_truth.traffic_sign.main_sign.classification.direction_scope
* sensor_view.global_ground_truth.traffic_sign.main_sign.classification.variability
* sensor_view.global_ground_truth.traffic_sign.main_sign.classification.value
Additional fields with option --mapnetwork:
* sensor_view.global_ground_truth.lane.id
* sensor_view.global_ground_truth.lane.classification.type
* sensor_view.global_ground_truth.lane.classification.lane_pairing
* sensor_view.global_ground_truth.lane.classification.centerline
* sensor_view.global_ground_truth.lane.classification.left_adjacent_lane_id
* sensor_view.global_ground_truth.lane.classification.right_adjacent_lane_id
* sensor_view.global_ground_truth.lane_boundary.left_lane_boundary_id
* sensor_view.global_ground_truth.lane_boundary.right_lane_boundary_id

### Interpreted Fields in OSI3 TrafficUpdate

* traffic_update.update.id
* traffic_update.update.model_reference
* traffic_update.update.base.dimension
* traffic_update.update.base.position
* traffic_update.update.base.orientation
* traffic_update.update.base.velocity
* traffic_update.update.base.orientation_rate
* traffic_update.update.vehicle_classification.light_state

## Configuration options

In the configuration yaml file of CoSiMa additional_parameters can be set to run CARLA-OSI-Service with several options.
The main options have specialised entries in the yaml file, like IP and port can be found [here](https://github.com/DLR-TS/CoSiMa/blob/master/Configuration.md).
The additional parameters can be chained, like "-v -ego herovehicle -sr"

```
additional_parameters: "-v"
additional_parameters: "--verbose"
```

Print verbose logs.

```
additional_parameter: "-sr"
additional_parameter: "--scenariorunner"
```

Waiting for connection of the Scenariorunner.

```
additional_parameter: "-a"
additional_parameter: "--async"
```

Running in asnychronous mode.

```
additional_parameter: "-l log.txt"
additional_parameter: "--logfile log.txt"
```

Write a csv log with trajectories (ID, X, Y and Yaw) to log.txt.

```
additional_parameter: "-ego hero"
```

Identify the ego vehicle in CARLA by name, like hero.

```
additional_parameter: "-replay"
```

In replay mode TrafficUpdate messages will spawn vehicles in Carla.
Default values are used.
If other replay options (weights, offsets or spawn height) are defined the replay mode will be set implicit.
```
additional_parameter: "-replaySpawnCarByName X"
```

In replay mode TrafficUpdate messages will spawn vehicles in Carla.
The given name will be used to spawn vehicles in Carla. Examples: _vehicle.tesla.model3_ or _vehicle.audi.etron_

```
additional_parameter: "-replayWeights X Y Z"
```

In replay mode TrafficUpdate messages will spawn vehicles in Carla.
The weight values are used to determine the best matching vehicle in CARLA to represent the bounding box given in the TrafficUpdate message.
X = height, Y = width, Z = height

```
additional_parameter: "-replayMapOffsets X Y"
```

In replay mode TrafficUpdate messages will spawn vehicles in Carla.
The map offsets are substracted from the location values given in the TrafficUpdate message.
X = UTM East, Y = UTM North

```
additional_parameter: "-replayOutputUTM"
```

In replay mode the GroundTruth message will take the original UTM offset into account.
The map offsets are given by "-replayMapOffsets X Y".
If you use this option the values can be quite large.
Make sure your further toolchain can handle this.

```
additional_parameter: "-replaySpawnHeight Z"
```

In replay mode TrafficUpdate messages will spawn cars in Carla.
New cars are spawned at a specific height if values are not provided by TrafficUpdate message.

```
additional_parameter: "-filterbyname name"
```

Only parse objects with "name" as a substring in their names for GroundTruth.

```
additional_parameter: "-resumeafter seconds"
additional_parameter: "-maxresponseinterval seconds"
```

Configure watchdog to set the simulation back to ansychronous mode after simulation.
Determine end of simulation by "seconds" with no interaction between CoSiMa and CARLA-OSI-Service.
Timer starts after first stepping (doStep) of simulation.

```
additional_parameter: "--camera"
additional_parameter: "--lidar"
additional_parameter: "--radar"
additional_parameter: "--ultrasonic"
```

Enable input of specific sensors in Carla.
Use "OSMPSensorView0" for first sensor to get the input.
The number increase for every added sensor in the simulation.
Use this only, if the sensor can not be spawned via OSTAR and is spawned by Carla itself or an other tool.

```
additional_parameter: "--cityobjectlabel label"
```

Only parse Objects defined by this filter option.
Possible values are: None, Buildings, Fences, Other, Poles, RoadLines, Roads, Sidewalks, TrafficSigns, Vegetation, Walls, Ground, Bridge, RailTrack, GuardRail, TrafficLight, Static, Water, Terrain, Any

```
additional_parameter: "--mapnetwork"
```

Parse OpenDrive map network to OSI GroundTruth message.
This will enlarge every GroundTruth message by a significant amount of data.
