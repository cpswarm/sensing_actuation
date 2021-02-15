#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ParamPull.h>
#include <mavros_msgs/ParamPush.h>
#include <std_msgs/Empty.h>
#include "mavros_gps/PoseToTarget.h"
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
 * @brief The goal pose during collision avoidance.
 */
geometry_msgs::PoseStamped local_goal_ca;

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
 * @brief The time in seconds without receiving collision avoidance goal messages after which a regular goal message is accepted again.
 */
double ca_timeout;

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
 * @brief The last time a goal for collision avoidance has been received.
 */
Time ca_update;

/**
 * @brief Frequency of the control loops.
 */
Rate* rate;

/**
 * @brief Service client to get the GPS target from the local pose.
 */
ServiceClient pose_to_target_client;

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
        mavros_msgs::GlobalPositionTarget global_goal;
        mavros_gps::PoseToTarget p2t;
        p2t.request.pose = goal;
        if (pose_to_target_client.call(p2t)) {
            // publish goal to fcu
            global_goal = p2t.response.target;
            global_goal.header.stamp = Time::now();
            publisher.publish(global_goal);

            ROS_DEBUG_THROTTLE(1, "Publish set point (%f,%f,%.2f,%.2f)", global_goal.latitude, global_goal.longitude, global_goal.altitude, global_goal.yaw);
        }
        else {
            ROS_ERROR_THROTTLE(1, "Failed to convert global goal");
        }
    }

    // move cps using local coordinates
    else {
        // shift local goal according to origin
        goal.pose.position.x -= origin.position.x;
        goal.pose.position.y -= origin.position.y;
        ROS_DEBUG_THROTTLE(1, "Publish set point (%.2f,%.2f,%.2f)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

        // publish goal to fcu
        goal.header.stamp = Time::now();
        publisher.publish(goal);
    }
}

/**
 * Processing incoming goal.
 * @param goal A reference to the goal.
 */
void process_goal(geometry_msgs::PoseStamped& goal)
{
    // keep orientation if turning is disabled
    if (turning == false)
        goal.pose.orientation = pose.pose.orientation;

    // calculate new orientation if turning is enabled
    else {
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, atan2(goal.pose.position.y - pose.pose.position.y, goal.pose.position.x - pose.pose.position.x));
        goal.pose.orientation = tf2::toMsg(orientation);
    }

    ROS_DEBUG("Move to (%.2f,%.2f,%.2f)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);

    // start publishing position set points
    goal_valid = true;

    // stop publishing velocity set points
    std_msgs::Empty stop_msg;
    stop_vel_pub.publish(stop_msg);
}

/**
 * Callback function for the CPS goal.
 * @param msg The CPS goal in local coordinates.
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store goal position
    local_goal = *msg;
    process_goal(local_goal);
}

/**
 * Callback function for the CPS goal during collision avoidance.
 * @param msg The CPS goal for collision avoidance in local coordinates.
 */
void ca_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_DEBUG_THROTTLE(1, "Collision avoidance");

    // store goal position
    local_goal_ca = *msg;

    // keep track of last received message
    ca_update = msg->header.stamp;

    process_goal(local_goal_ca);
}

/**
 * Callback function to receive current pose.
 * @param msg The received CPS pose in local coordinates.
 */
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = *msg;
    ROS_DEBUG_THROTTLE(10, "Pose (%.2f,%.2f,%2.f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
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
    double max_velocity;
    nh.param(this_node::getName() + "/max_velocity", max_velocity, 1.0);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.1);
    nh.param(this_node::getName() + "/ca_timeout", ca_timeout, 1.0);
    nh.param(this_node::getName() + "/turning", turning, true);
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // wait for valid pose and goal
    pose_valid = false;
    goal_valid = false;

    // subscribers
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);
    Subscriber goal_sub = nh.subscribe("pos_controller/goal_position", queue_size, goal_callback);
    Subscriber ca_goal_sub = nh.subscribe("pos_controller/ca_goal_position", queue_size, ca_goal_callback);
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

    // set maximum velocity
    ServiceClient pull_client = nh.serviceClient<mavros_msgs::ParamPull>("mavros/param/pull");
    ROS_DEBUG("Waiting MAVROS parameter pull service");
    pull_client.waitForExistence();
    mavros_msgs::ParamPull pull;
    if (pull_client.call(pull)) {
        if (pull.response.success) {
            ROS_DEBUG("Pulled %d parameters from FCU", pull.response.param_received);
            nh.setParam("mavros/param/MPC_XY_VEL_MAX", max_velocity);
            ServiceClient push_client = nh.serviceClient<mavros_msgs::ParamPush>("mavros/param/push");
            ROS_DEBUG("Waiting MAVROS parameter push service");
            push_client.waitForExistence();
            mavros_msgs::ParamPush push;
            if (push_client.call(push)) {
                if (push.response.success)
                    ROS_DEBUG("Pushed %d parameters to FCU", push.response.param_transfered);
                else
                    ROS_ERROR("Failed push parameters to FCU, cannot set maximum horizontal velocity!");
            }
            else {
                ROS_ERROR("Failed to push parameters to FCU, cannot set maximum horizontal velocity!");
            }
        }
        else {
            ROS_ERROR("Failed to pull parameters from FCU, cannot set maximum horizontal velocity!");
        }
    }
    else {
        ROS_ERROR("Failed to pull parameters from FCU, cannot set maximum horizontal velocity!");
    }

    // service clients
    pose_to_target_client = nh.serviceClient< mavros_gps::PoseToTarget > ("gps/pose_to_target");
    if (global) {
        ROS_DEBUG("Waiting for pose to target service");
        pose_to_target_client.waitForExistence();
    }
    ServiceClient danger_client = nh.serviceClient<cpswarm_msgs::Danger>("obstacle_detection/danger");
    ROS_DEBUG("Waiting obstacle detection service");
    danger_client.waitForExistence();

    // goal publisher
    if (global) {
        publisher = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", queue_size, true);
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
            // check if collision avoidance is active
            bool ca = false;
            if (ca_update.isZero() == false)
                ca = ca_update + Duration(ca_timeout) > Time::now();

            // publish set point to perform collision avoidance
            if (ca) {
                publish_goal(local_goal_ca);
            }

            // publish set point to reach original goal
            else {
                // reset collision avoidance message stats
                ca_update.fromSec(0);

                publish_goal(local_goal);
            }

        }

        // sleep rest of cycle
        rate->sleep();
    }

    delete rate;

    return 0;
}
