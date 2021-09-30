# area_provider

This package provides services to access the area of the mission in local coordinates. It is part of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation). The basis for these services is a map of the mission area which can be provided in two ways:

1. Map server: If a [map server](https://wiki.ros.org/map_server) is running, its map is used.

2. Coordinates: Otherwise, the coordinates given in the parameter file of the area provider are used to create the map.

If both sources are unavailable the area provider will not work.

## Dependencies
This package depends on the following message definitions:
* [geometry_msgs](https://wiki.ros.org/geometry_msgs)
* [nav_msgs](https://wiki.ros.org/nav_msgs)
* [cpswarm_msgs](https://cpswarm.github.io/cpswarm_msgs/html/index-msg.html)

The communication between CPSs is based on the [CPSwarm Communication Library](https://github.com/cpswarm/swarmio).

The following packages of the [sensing and actuation library](https://github.com/cpswarm/sensing_actuation) are required:
* mavros_gps

Further required packages are:
* [roscpp](https://wiki.ros.org/roscpp/)

## Execution
Run the launch file
```
roslaunch area_provider area_provider.launch
```
to launch the `area_provider` node.

The launch file can be configured with following parameters:
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

In the `param` subdirectory there is the parameter file `area_provider.yaml` that allows to configure the behavior of the `area_provider` node.

## Nodes

### area_provider
The `area_provider` provides several services that allow to access the mission area as map and perform several spatial operations. For more details about theses services see below. If no mission area can be retrieved, this node will issue a fatal error and shutdown.

#### Subscribed Topics
* `map` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  The occupancy grid map provided by the map server.

#### Published Topics
* `area/map` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))
  The map provided by the area provider. It is either copied from the map server or created from the given coordinates.

#### Services
* `area/closest_bound` ([cpswarm_msgs/ClosestBound](https://cpswarm.github.io/cpswarm_msgs/html/srv/ClosestBound.html))
  Get the boundary segment of the area polygon closest to a given point. It returns the coordinates of the boundary segment and the perpendicular distance to the given point.
* `area/get_area` ([cpswarm_msgs/GetPoints](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoints.html))
  Get the bounding area in which the CPS is allowed to move. Returns a vector of points that defines the bounding polygon coordinates.
* `area/get_center` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the center of the area in which the CPS is allowed to move. Returns the coordinates of the geometric center of the area.
* `area/get_map` ([nav_msgs/GetMap](https://docs.ros.org/en/api/nav_msgs/html/srv/GetMap.html))
  Get the map that represents the environment in which the CPS is allowed to move. Returns the grid map of the environment.
* `area/get_origin` ([cpswarm_msgs/GetPoint](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetPoint.html))
  Get the origin of the coordinate system. Returns the coordinates of the starting point of the CPS.
* `area/get_rotation` ([cpswarm_msgs/GetDouble](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetDouble.html))
  Get the rotation required to align the lower boundary of the area horizontally assuming a quadrilateral area. Returns the angle that the bottom edge of the area is rotated with respect to the x-axis.
* `area/out_of_bounds` ([cpswarm_msgs/OutOfBounds](https://cpswarm.github.io/cpswarm_msgs/html/srv/OutOfBounds.html))
  Determine whether a position is within the area using the winding number algorithm. Returns true, if the position is outside of the area, false otherwise.

#### Services Called
* `gps/fix_to_pose` ([cpswarm_msgs/FixToPose](https://cpswarm.github.io/cpswarm_msgs/html/srv/FixToPose.html))
  Convert a [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) message to a [geometry_msgs/PoseStamped](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) message.
* `gps/get_gps_origin` ([cpswarm_msgs/GetGpsFix](https://cpswarm.github.io/cpswarm_msgs/html/srv/GetGpsFix.html))
  Get the GPS coordinates of the point at which the CPS started.

#### Parameters
* `~loop_rate` (real, default: `1.5`)
  The frequency in Hz at which to run the control loops.
* `~queue_size` (integer, default: `10`)
  The size of the message queue used for publishing and subscribing to topics.
* `~wait_for_map` (real, default: `2.0`)
  Time in seconds to wait initially for another map provider. If this timeout expires, no map server is assumed to be available and the map is constructed from another source.
* `~cell_warn` (integer, default: `1000`)
  Number of grid cells in the map above which a performance warning is issued. Only relevant if no map is provided by the map server but created by this node.
* `resolution` (real, default: `1.0`)
  Resolution of the grid map created from the given polygon coordinates representing the area in meter / cell.
* `area_x` (real list, default: `[-10, 10, 10, -10]`)
  X-coordinates/longitudes that specify the mission area polygon. Make sure they are given in the same order as `area_y`.
* `area_y` (real list, default: `[-10, -10, 10, 10]`)
  Y-coordinates/latitudes that specify the mission area polygon. Make sure they are given in the same order as `area_x`.

## Code API
[area_provider package code API documentation](https://cpswarm.github.io/sensing_actuation/area_provider/docs/html/files.html)
