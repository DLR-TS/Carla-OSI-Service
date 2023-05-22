# Configuration of CARLA OSI Service

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

Write a csv log with trajectories to given location, like log.txt.

```
additional_parameter: "-ego hero"
```

Identify the ego vehicle in CARLA by name, like hero.

```
additional_parameter: "-replay"
```

In replay mode TrafficUpdate messages will spawn cars in Carla.
Default values are used.
If other replay options (weights, offsets or spawn height) are defined the replay mode will be set implicit.

```
additional_parameter: "-replayWeights X Y Z"
```

In replay mode TrafficUpdate messages will spawn cars in Carla.
The weight values are used to determine the best matching vehicle in CARLA to represent the bounding box given in the TrafficUpdate message.
X = height, Y = width, Z = height

```
additional_parameter: "-replayMapOffsets X Y"
```

In replay mode TrafficUpdate messages will spawn cars in Carla.
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

Spawn specific sensor.

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
