# area_provider

This package provides services to access the area of the mission and regions of interest (ROIs) in local coordinates. It is part of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation).

## Dependencies
This package depends on the following message definitions:
* [std_msgs](https://wiki.ros.org/std_msgs)
* [std_srvs](https://wiki.ros.org/std_srvs)
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* mavros_gps

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

The ROI services depend on following external libraries:
* [fstream](https://en.cppreference.com/w/cpp/header/fstream)
* [filesystem](https://en.cppreference.com/w/cpp/filesystem)
* [nlohmann/json](https://github.com/nlohmann/json) (gets installed automatically)
* [roslib](https://wiki.ros.org/roslib)

## Execution
This package contains two launch files that allow to launch the mission area services and the ROI services.

Both launch files can be configured with following parameters:
* `id` (integer, default: `1`)
  The identifier (ID) of the CPS used for name spacing in simulation.
* `pos_type` (string, default: `local`)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `x` (real, default: `0`)
  The starting position x-coordinate, used as origin for local positioning.
* `y` (real, default: `0`)
  The starting position y-coordinate, used as origin for local positioning.
* `output` (string, default: `log`)
  Whether to show the program output (`screen`) or to write it to a log file (`log`).

In the `param` subdirectory there is the parameter file `area.yaml` that allows to configure the behavior common to both services.

### Mission Area Services
Run the launch file
```
roslaunch area_provider ma_services.launch
```
to launch the `ma_services` node.

In the `param` subdirectory there is the parameter file `ma.yaml` that allows to configure the behavior of the `ma_services` node.

### ROI Services
Run the launch file
```
roslaunch area_provider roi_services.launch
```
to launch the `roi_services` node.

In the `param` subdirectory there is the parameter file `roi.yaml` that allows to configure the behavior of the `roi_services` node.

## Nodes
Both nodes provide several services that allow to access the mission area and the ROIs as map and perform several spatial operations. For more details about theses services see below.

### ma_services
The basis for the mission area services is a map of the mission area which can be provided in two ways:

1. Map server: If a [map server](https://wiki.ros.org/map_server) is running, its map is used.

2. Coordinates: Otherwise, the coordinates given in the `ma.yaml` parameter file are used to create the map.

If both sources are unavailable the `ma_services`, this node will issue a fatal error and shutdown.

#### Subscribed Topics
* `map` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  The occupancy grid map provided by the map server.

#### Published Topics
* `area/map` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  The map provided by the area provider. It is either copied from the map server or created from the given coordinates.

#### Services
* `area/get_area` ([cpswarm_msgs/GetPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoints.html))
  Get the bounding area in which the CPS is allowed to move. Returns a vector of points that defines the bounding polygon coordinates.
* `area/get_center` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the center of the area in which the CPS is allowed to move. Returns the coordinates of the geometric center / centroid of the area.
* `area/get_distance` ([cpswarm_msgs/GetDist](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDist.html))
  Calculates the distance of given point (or the origin in case the given point is empty (0,0)) to the mission area boundary. The point can be either inside or outside the area. Returns the coordinates of the boundary segment, the closest point on the segment, and the distance to the given point. The `coords` variable, both in request and response is ignored.
* `area/get_map` ([nav_msgs/GetMap](https://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
  Get the map that represents the environment in which the CPS is allowed to move. Returns the grid map of the environment. The `coords` variable of the request is ignored. The service request offers several options: If `rotate` is set to true, the map is rotated to align the bottom edge horizontally. The response also returns the angle of rotation. If `translate` is set to true, the map is  translated to make the origin an even number. The response returns the translation vector. If `resolution` is specified, the map will be provided at the desired resolution.
* `area/get_origin` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the origin of the coordinate system. Returns the coordinates of the starting point of the CPS.
* `area/out_of_bounds` ([cpswarm_msgs/OutOfBounds](https://cpswarm.github.io/cpswarm_msgs/html/srv/OutOfBounds.html))
  Determine whether a position is within the area using the winding number algorithm. Returns true, if the position is outside of the area, false otherwise. Points on the boundary are not considered to be out of bounds.

#### Services Called
* `gps/fix_to_pose` ([cpswarm_msgs/FixToPose](https://cpswarm.github.io/cpswarm_msgs/html/srv/FixToPose.html))
  Convert a [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) message to a [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) message.
* `gps/get_gps_origin` ([cpswarm_msgs/GetGpsFix](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetGpsFix.html))
  Get the GPS coordinates of the point at which the CPS started.

#### Parameters
* `pos_type` (string, default: `global`)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `x` (real, default: `0.0`)
  The starting position x-coordinate, used as origin for local positioning.
* `y` (real, default: `0.0`)
  The starting position y-coordinate, used as origin for local positioning.
* `~loop_rate` (real, default: `1.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~publish_map` (boolean, default: `false`)
  Whether to publish the grid map of the mission area.
* `~cell_warn` (integer, default: `1000`)
  Number of grid cells in the map above which a performance warning is issued. Only relevant if no map is provided by the map server but created by this node.
* `~resolution` (real, default: `1.0`)
  Resolution of the grid map created from the given polygon coordinates representing the area in meter / cell.
* `~wait_for_map` (real, default: `2.0`)
  Time in seconds to wait initially for another map provider. If this timeout expires, no map server is assumed to be available and the map is constructed from another source.
* `~area_x` (real list, default: `[]`)
  X-coordinates/longitudes that specify the mission area polygon. Make sure they are given in the same order as `area_y`. Only read if no map server is available.
* `~area_y` (real list, default: `[]`)
  Y-coordinates/latitudes that specify the mission area polygon. Make sure they are given in the same order as `area_x`. Only read if no map server is available.

### roi_services
The basis for the ROI services are coordinates of the ROIs which are provided through several JSON files in a specified directory. The JSON files must follow the definition of the [qGroundControl plan file format](https://dev.qgroundcontrol.com/master/en/file_formats/plan.html).

#### Subscribed Topics
* `bridge/events/roi` ([cpswarm_msgs/PointArrayEvent](https://cpswarm.github.io/cpswarm_msgs/html/msg/PointArrayEvent.html))
  Receive the coordinates of a ROI and import it.

#### Published Topics
* `rois/map_<i>` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  The map representing ROI `i`, where `i` is the ordinal number of the ROIs in the order in which they are imported. Only if the parameter `visualize` is set to `true`.

#### Services
* `rois/get_all` ([cpswarm_msgs/GetMultiPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetMultiPoints.html))
  Get coordinates of all ROIs. Returns a flattened vector of points together with the layout of the original, two-dimensional vector.
* `rois/get_closest` ([cpswarm_msgs/GetDist](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDist.html))
  Find the ROI that is closest to a given point or the origin in case the given point is empty (0,0). It returns the coordinates of the closest boundary segment, the closest point on the segment, the distance to the given point, and the coordinates of the complete ROI.
* `area/get_distance` ([cpswarm_msgs/GetDist](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDist.html))
  Calculates the distance of given point (or the origin in case the given point is empty (0,0)) to the ROI specified by the `coords` variable in the request. The point can be either inside or outside the area. Returns the coordinates of the boundary segment, the closest point on the segment, and the distance to the given point.
* `area/get_map` ([nav_msgs/GetMap](https://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
  Get the map that represents the ROI specified by the `coords` variable in the request. Returns a grid map. The service request offers several options: If `rotate` is set to true, the map is rotated to align the bottom edge horizontally. The response also returns the angle of rotation. If `translate` is set to true, the map is  translated to make the origin an even number. The response returns the translation vector. If `resolution` is specified, the map will be provided at the desired resolution.
* `rois/reload` ([std_srvs/SetBool](https://docs.ros.org/en/api/std_srvs/html/srv/SetBool.html))
  Reload ROIs from files. If set to true, previous ROIs are removed before reloading.

#### Services Called
* `gps/fix_to_pose` ([cpswarm_msgs/FixToPose](https://cpswarm.github.io/cpswarm_msgs/html/srv/FixToPose.html))
  Convert a [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) message to a [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) message.
* `gps/get_gps_origin` ([cpswarm_msgs/GetGpsFix](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetGpsFix.html))
  Get the GPS coordinates of the point at which the CPS started.

#### Parameters
* `pos_type` (string, default: `global`)
  Whether relative (`local`) or GPS (`global`) coordinates are used by the flight controller.
* `x` (real, default: `0.0`)
  The starting position x-coordinate, used as origin for local positioning.
* `y` (real, default: `0.0`)
  The starting position y-coordinate, used as origin for local positioning.
* `~loop_rate` (real, default: `1.0`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `1`)
  The size of the message queue used for publishing and subscribing to topics.
* `~cell_warn` (integer, default: `1000`)
  Number of grid cells in the map above which a performance warning is issued.
* `~resolution` (real, default: `1.0`)
  Resolution of the grid map created from the given polygon coordinates representing the ROI in meter / cell.
* `~roi_dir` (string, default: )
  The directory relative to this package where to look for the ROI files.
* `~duplicates` (boolean, default: `false`)
  Whether to allow duplicate ROIs, i.e., ROIs with identical coordinates.
* `~publish` (boolean, default: `false`)
  Whether to publish any newly imported ROI as event.
* `~visualize` (bolean, default: `false`)
  Whether to publish the map of any newly imported ROI.

## Code API
[area_provider package code API documentation](https://cpswarm.github.io/sensing_actuation/area_provider/docs/html/files.html)
