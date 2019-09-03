#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include "range_sensor.h"
#include "lidar_sensor.h"
#include "swarm_position.h"
#include "angle.h"
#include "sector.h"
#include "cpswarm_msgs/clear_of_obstacles.h"
#include "cpswarm_msgs/danger.h"
#include "cpswarm_msgs/get_occupied_sector.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that offers obstacle detection based on range sensors and swarm communication.
 */
class obstacle_detection
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    obstacle_detection ();

    /**
     * @brief Destructor.
     */
    ~obstacle_detection ();

    /**
     * @brief Check if obstacles are detected.
     * @param req Empty request.
     * @param res True if there are no obstacles detected, false otherwise.
     * @return Whether request succeeded.
     */
    bool clear_of_obstacles (cpswarm_msgs::clear_of_obstacles::Request &req, cpswarm_msgs::clear_of_obstacles::Response &res);

    /**
     * @brief Check if obstacles are detected.
     * @param req Empty request.
     * @param res True if there are is an obstacle dangerously close, false otherwise.
     * @return Whether request succeeded.
     */
    bool danger (cpswarm_msgs::danger::Request &req, cpswarm_msgs::danger::Response &res);

    /**
     * @brief Get the sector that is occupied by obstacles.
     * @param req Empty request.
     * @param res The sector that is occupied.
     * @return Whether request succeeded.
     */
    bool get_occupied_sector (cpswarm_msgs::get_occupied_sector::Request &req, cpswarm_msgs::get_occupied_sector::Response &res);

private:
    /**
     * @brief Callback function for position updates.
     * @param msg Position received from the CPS.
     */
    void pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the position of the CPS.
     */
    Subscriber pose_sub;

    /**
     * @brief Positions of the other CPSs in the swarm.
     */
    swarm_position* swarm;

    /**
     * @brief The CPS's range sensors.
     */
    map<unsigned int, range_sensor*> range_sensors;

    /**
     * @brief The CPS's lidar sensor.
     */
    lidar_sensor* lidar;

    /**
     * @brief The CPS orientation.
     */
    angle yaw;
};

#endif // OBSTACLE_DETECTION_H
