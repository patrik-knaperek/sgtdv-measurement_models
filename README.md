# **Measurement Models package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Processing and logging data during acquisition for camera / LIDAR measurement model estimation. 
___

The `measurement_models` node provides an automated tool which task is to collect cone detections from sensors (stereocamera, LIDAR or both; real, logged or simulated), process them and log output into CSV file. The logged data is to be used for estimation of sensors' measurement model (systematic and random error characteristics) which appears in the cone detection fusion.

### Experiment description

During the experiment, we let a sensor to observe a set of cones placed on known positions with respect to the sensor, such that each cone is in the direct visibility of the sensor. We consequently repeat the experiment with new cone placements, until the whole Field Of View (or its desired part) is covered.

To do so, the cones are placed on the same `x`-coordinates, with a 1 m sparse in the `y`-coordinates. Experiment is repeated with a 1 m sparse in the `x`-coordinates. (Such layout is not a necessary condition, it is recommended though, as it provides a good balance between density of measured points and ability to associate a measurement to the real position of a cone, even in case of higher measurement error.)

### Related packages
* [`camera_driver`](../camera_driver/README.md)
* [`lidar_cone_detection`](../lidar_cone_detection/README.md)
* [`cone_detection_si`](../simulation_interface/cone_detection_si/README.md)

## Compilation
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin build measurement_models
```

## Launch
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ source ./devel/setup.bash
```
* on real or rosbag data
```sh
$ roslaunch measurement_models measurement_models_rc.launch
```
* alongside FSSIM (the `slam_si` package has to be built first)
```sh
$ roslaunch measurement_models measurement_models_sim.launch
```
* with RC car config
```sh
$ roslaunch path_planning path_planning_rc.launch
```

### Configuration
* `fixed_frame` : the coordinate frame of measurements
* Real cones coordinates specification
  - specify exact positions of the observed cones in the `cone_coords` parameter OR
  - specify x-coordinate(s) `cone_coords_x` and y-coordinates `cone_coords_y` of the cones (the exact coordinates are computed automatically in this case)
* `number_of_cones` : number of observed real cones
* `number_of_sensors` : number of sensors which the data is being collected from (during simulated experiment, it is possible to collect data from both sensors simultaneously)
* `number_of_measurements` : terminating condition for data acquisition
* `distance_threshold_x` : threshold for association of the measurement to a real cone
* `distance_threshold_y` : threshold for association of the measurement to a real cone
* `output_filename` : log file identification (the whole filename is composed as `<path_to_package>/data/<output_filename>_<sensor_name>.csv`)

### Flowchart

<p align="left">
    <img src="./doc/SW flowcharts-Measuremet Models.svg" width="500">
</p>
