#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <std_msgs/Empty.h>
#include "cpswarm_msgs/PoseToGeo.h"
#include "cpswarm_msgs/Danger.h"

using namespace std;
using namespace ros;

/**
 * @brief The MAVROS FCU connection state.
 */
mavros_msgs::State state;

/**
 * @brief The goal pose.
 */
geometry_msgs::PoseStamped local_goal;

/**
 * @brief The position of the CPS.
 */
geometry_msgs::PoseStamped pose;

/**
 * @brief The initial CPS position in local coordinates.
 */
geometry_msgs::Pose origin;

/*
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Whether the goal is valid and should be pursued.
 */
bool goal_valid;

/**
 * @brief The time in seconds that the turning is allowed to take.
 */
Duration turn_timeout;

/**
 * @brief Whether the CPS should turn its front into movement direction or not.
 */
bool turning;

/**
 * @brief Whether to publish the waypoints on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether global GPS coordinates should be published.
 */
bool global;

/**
 * @brief The angle in radian which the yaw of the CPS can vary around the set point.
 */
double yaw_tolerance;

/**
 * @brief Frequency of the control loops.
 */
Rate* rate;

/**
 * @brief Service client to get the GPS coordinates from the local pose.
 */
ServiceClient pose_to_geo_client;

/**
 * @brief Publisher for the position set point.
 */
Publisher publisher;

/**
 * @brief Publisher to stop velocity set points.
 */
Publisher stop_vel_pub;

/**
 * @brief Publisher to visualize the current waypoint.
 */
Publisher wp_pub;

/**
 * @brief Compute the yaw angle from a given pose.
 * @param pose The pose to compute the angle from.
 * @return An angle that represents the orientation of the pose.
 */
double get_yaw (geometry_msgs::Pose pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

/**
 * @brief Check whether the CPS has to turn at least by the yaw tolerance to reach the current goal.
 * @return True, if the goal is far enough away in terms of turning.
 */
bool enough_yaw()
{
    double delta_yaw = remainder(get_yaw(pose.pose) - get_yaw(local_goal.pose), 2*M_PI);
    return delta_yaw < -yaw_tolerance || yaw_tolerance < delta_yaw;
}

/**
 * @brief Move the CPS by publishing the goal to the FCU.
 * @param goal The goal to move to.
 */
void publish_goal (geometry_msgs::PoseStamped goal) {
    // visualize waypoint
    if (visualize) {
        geometry_msgs::PointStamped wp;
        wp.header.stamp = Time::now();
        wp.header.frame_id = "local_origin_ned";
        wp.point = goal.pose.position;
        wp_pub.publish(wp);
    }

    // move cps using gps coordinates
    if (global) {
        // convert local goal to gps coordinates
        geographic_msgs::GeoPoseStamped global_goal;
        cpswarm_msgs::PoseToGeo p2g;
        p2g.request.pose = goal;
        if (pose_to_geo_client.call(p2g)) {
            // publish goal to fcu
            global_goal = p2g.response.geo;
            global_goal.header.stamp = Time::now();
            publisher.publish(global_goal);

            ROS_DEBUG("Publish set point (%f,%f,%.2f)", global_goal.pose.position.latitude, global_goal.pose.position.longitude, global_goal.pose.position.altitude);
        }
        else {
            ROS_ERROR("Failed to convert global goal");
        }
    }

    // move cps using local coordinates
    else {
        // shift local goal according to origin
        goal.pose.position.x -= origin.position.x;
        goal.pose.position.y -= origin.position.y;
        ROS_DEBUG("Publish set point (%.2f,%.2f,%.2f,%.2f)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, get_yaw(goal.pose));

        // publish goal to fcu
        goal.header.stamp = Time::now();
        publisher.publish(goal);
    }
}

/**
 * @brief Compute local goal coordinates using distance and direction relative to the current pose.
 * @param distance The distance of the goal from start.
 * @param direction The direction of the goal relative to start. It is in radian, counterclockwise starting from east / x-axis.
 * @return The computed goal position.
 */
geometry_msgs::PoseStamped compute_goal (double distance, double direction)
{
    geometry_msgs::PoseStamped goal;

    // calculate position
    goal.pose.position.x = pose.pose.position.x + distance * cos(direction);
    goal.pose.position.y = pose.pose.position.y + distance * sin(direction);
    goal.pose.position.z = pose.pose.position.z;

    // calculate orientation
    if (turning) {
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, direction);
        goal.pose.orientation = tf2::toMsg(orientation);
    }
    else
        goal.pose.orientation = pose.pose.orientation;

    return goal;
}

void turn ()
{
    // create temporary goal pose for cps
    geometry_msgs::PoseStamped turn_goal;
    turn_goal.header.stamp = Time::now();

    // set goal position to previous goal position
    turn_goal.pose.position = pose.pose.position;

    // set goal orientation to new orientation
    turn_goal.pose.orientation = local_goal.pose.orientation;

    // turning start time
    Time turn_time = Time::now();

    // turn until cps reached goal
    while (ok() && enough_yaw() && turn_time + turn_timeout > Time::now()) {
        // send goal to cps controller
        publish_goal(turn_goal);

        // wait
        rate->sleep();

        ROS_DEBUG("Turning %.2f --> %.2f", get_yaw(pose.pose), get_yaw(local_goal.pose));

        // check if reached goal
        spinOnce();
    }

    // timeout for turning expired
    if (enough_yaw() && turn_time + turn_timeout <= Time::now()) {
        ROS_ERROR("Turning timed out");
    }
}

/**
 * Callback function for the CPS goal.
 * @param msg The CPS goal in local coordinates.
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store goal position
    local_goal = *msg;

    // keep orientation if turning is disabled
    if (turning == false)
        local_goal.pose.orientation = pose.pose.orientation;

    // calculate new orientation if turning is enabled
    else {
        // calculate orientation
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, atan2(local_goal.pose.position.y - pose.pose.position.y, local_goal.pose.position.x - pose.pose.position.x));
        local_goal.pose.orientation = tf2::toMsg(orientation);
    }

    ROS_DEBUG("Move to (%.2f,%.2f,%.2f,%.2f)", local_goal.pose.position.x, local_goal.pose.position.y, local_goal.pose.position.z, get_yaw(local_goal.pose));

    // start publishing position set points
    goal_valid = true;

    // stop publishing velocity set points
    std_msgs::Empty stop_msg;
    stop_vel_pub.publish(stop_msg);
}

/**
 * Callback function to receive current pose.
 * @param msg The received CPS pose in local coordinates.
 */
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
    ROS_DEBUG_THROTTLE(10, "Pose (%.2f,%.2f,%2.f,%2.f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, get_yaw(pose.pose));
    pose_valid = true;
}

/**
 * Callback function to receive FCU connection state.
 * @param msg The received state.
 */
void state_callback(const mavros_msgs::State::ConstPtr& msg) {
    state = *msg;
}

/**
 * Callback function to stop publishing set points.
 * @param msg An empty message.
 */
void stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_DEBUG("Stop");

    // stop publishing set points
    goal_valid = false;
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv) {
    // init ros node
    init(argc, argv, "pos_controller");
    NodeHandle nh;

    // read parameters
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    global = pos_type == "local" ? false : true;
    double x,y;
    nh.param(this_node::getName() + "/x", x, 0.0);
    nh.param(this_node::getName() + "/y", y, 0.0);
    origin.position.x = x;
    origin.position.y = y;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    rate = new Rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.1);
    double d_turn_timeout;
    nh.param(this_node::getName() + "/turn_timeout", d_turn_timeout, 5.0);
    turn_timeout = Duration(d_turn_timeout);
    nh.param(this_node::getName() + "/turning", turning, true);
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // wait for valid pose and goal
    pose_valid = false;
    goal_valid = false;

    // subscribers
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber goal_sub = nh.subscribe("pos_controller/goal_position", queue_size, goal_callback);
    Subscriber stop_sub = nh.subscribe("pos_controller/stop", queue_size, stop_callback);
    Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", queue_size, state_callback);

    // publisher
    stop_vel_pub = nh.advertise<std_msgs::Empty>("vel_controller/stop", queue_size, true);
    if (visualize) {
        wp_pub = nh.advertise<geometry_msgs::PointStamped>("pos_controller/waypoint", queue_size, true);
    }

    // wait for fcu connection
    while (ok() && state.connected == false) {
        ROS_DEBUG_ONCE("Waiting for FCU connection");
        spinOnce();
        rate->sleep();
    }

    // wait for valid position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose");
        spinOnce();
        rate->sleep();
    }

    // service clients
    pose_to_geo_client = nh.serviceClient< cpswarm_msgs::PoseToGeo > ("gps/pose_to_geo");
    if (global)
        pose_to_geo_client.waitForExistence();
    ServiceClient danger_client = nh.serviceClient<cpswarm_msgs::Danger>("obstacle_detection/danger");
    danger_client.waitForExistence();

    // goal publisher
    if (global) {
        publisher = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", queue_size, true);
    }
    else {
        publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", queue_size, true);
    }

    ROS_INFO("Ready");

    // provide position controller
    while (ok()) {
        // get position and goal update
        spinOnce();

        // only publish set point if a goal has been received
        if (goal_valid) {
            // turn nose into movement direction, if required
            if (turning)
                turn();

            // check if dangerously close to obstacle
            cpswarm_msgs::Danger danger;
            if (danger_client.call(danger) == false) {
                ROS_ERROR_THROTTLE(1, "Failed to check if obstacle near by, stop moving!");
                local_goal = pose;
            }

            // there are obstacles dangerously close, back off
            else if (danger.response.danger) {
                ROS_ERROR_THROTTLE(1, "Obstacle too close, backoff!");
                local_goal = compute_goal(danger.response.backoff, danger.response.direction + get_yaw(pose.pose));
            }

            // publish set point
            publish_goal(local_goal);
        }

        // sleep rest of cycle
        rate->sleep();
    }

    delete rate;

    return 0;
}
