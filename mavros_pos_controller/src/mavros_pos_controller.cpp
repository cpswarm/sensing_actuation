#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <std_msgs/Empty.h>
#include "mavros_gps/PoseToTarget.h"
#include "cpswarm_msgs/ClearOfObstacles.h"
#include "cpswarm_msgs/Danger.h"
#include "cpswarm_msgs/GetSector.h"

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
 * @brief Whether global GPS coordinates should be published.
 */
bool global;

/**
 * @brief The distance in meter which the position of the CPS can vary around the set point.
 */
double goal_tolerance;

/**
 * @brief The angle in radian which the yaw of the CPS can vary around the set point.
 */
double yaw_tolerance;

/**
 * @brief Frequency of the control loops.
 */
Rate* rate;

/**
 * @brief Service client to retrieve the sector occupied by obstacles.
 */
ServiceClient occupied_sector_client;

/**
 * @brief Service client to get the GPS target from the local pose.
 */
ServiceClient PoseToTarget_client;

/**
 * @brief Publisher for the position set point.
 */
Publisher publisher;

/**
 * @brief Publisher to stop velocity set points.
 */
Publisher stop_vel_pub;

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
 * @brief Check whether the current goal is at least the goal tolerance away from the current pose.
 * @return True, if the goal is far enough away.
 */
bool enough_dist ()
{
    double dx = local_goal.pose.position.x - pose.pose.position.x;
    double dy = local_goal.pose.position.y - pose.pose.position.y;
    double dz = local_goal.pose.position.z - pose.pose.position.z;

    return (dx*dx + dy*dy + dz*dz) > (goal_tolerance * goal_tolerance);
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
    // move cps using gps coordinates
    if (global) {
        // convert local goal to gps coordinates
        mavros_msgs::GlobalPositionTarget global_goal;
        mavros_gps::PoseToTarget p2t;
        p2t.request.pose = goal;
        if (PoseToTarget_client.call(p2t)) {
            ROS_DEBUG("POS_CTRL - Publish set point (%f,%f,%.2f,%.2f)", global_goal.latitude, global_goal.longitude, global_goal.altitude, global_goal.yaw);

            // publish goal to fcu
            global_goal = p2t.response.target;
            global_goal.header.stamp = Time::now();
            publisher.publish(global_goal);
        }
        else {
            ROS_ERROR("POS_CTRL - Failed to convert global goal");
        }
    }

    // move cps using local coordinates
    else {
        // shift local goal according to origin
        goal.pose.position.x -= origin.position.x;
        goal.pose.position.y -= origin.position.y;
        ROS_DEBUG("POS_CTRL - Publish set point (%.2f,%.2f,%.2f,%.2f)", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, get_yaw(goal.pose));

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
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, direction);
    goal.pose.orientation = tf2::toMsg(orientation);

    return goal;
}

/**
 * @brief Obstacle avoidance procedure to avoid other CPS with the same controller.
 */
void obstacle_avoidance ()
{
    ROS_INFO("POS_CTRL - Obstacle avoidance");

    // compute direction and distance of goal
    double goal_dist = hypot(pose.pose.position.x - local_goal.pose.position.x, pose.pose.position.y - local_goal.pose.position.y);
    double goal_dir = get_yaw(local_goal.pose);

    // get occupied sector
    cpswarm_msgs::GetSector gos;
    if (occupied_sector_client.call(gos) == false){
        ROS_ERROR("POS_CTRL - Failed to get occupied sector, stop moving!");
        publish_goal(pose);
        return;
    }
    double occ = gos.response.min + (gos.response.max - gos.response.min) / 2.0; // always max > min

    // yaw of occupied sector relative to current yaw
    double rel_yaw = remainder(occ - get_yaw(pose.pose), 2*M_PI) + M_PI; // [0,2π]

    ROS_DEBUG("POS_CTRL - Obstacle at yaw %.2f < %.2f < %.2f", gos.response.min, occ, gos.response.max);
    ROS_DEBUG("POS_CTRL - My yaw %.2f", get_yaw(pose.pose));

    // turn right if cps is coming from ahead
    if (rel_yaw < M_PI / 2.0 || 3.0 * M_PI / 2.0 < rel_yaw) {
        ROS_DEBUG("POS_CTRL - Change yaw %.2f --> %.2f", goal_dir, goal_dir + M_PI / 4.0 * cos(rel_yaw));
        goal_dir += M_PI / 4.0 * cos(rel_yaw);
    }

    // slow down if cps is coming from right
    if (rel_yaw < M_PI) {
        ROS_DEBUG("POS_CTRL - Reduce distance %.2f --> %.2f", goal_dist, goal_dist * (1 - sin(rel_yaw)));
        goal_dist *= (1.0 - sin(rel_yaw));
    }

    // speed up if cps is coming from left
    else {
        ROS_DEBUG("POS_CTRL - Increase distance %.2f --> %.2f", goal_dist, goal_dist * (1 + abs(sin(rel_yaw))));
        goal_dist *= (1.0 + abs(sin(rel_yaw)));
    }

    // set new goal
    publish_goal(compute_goal(goal_dist, goal_dir));
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

    ROS_DEBUG("POS_CTRL - Move to (%.2f,%.2f,%.2f,%.2f)", local_goal.pose.position.x, local_goal.pose.position.y, local_goal.pose.position.z, get_yaw(local_goal.pose));

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
    ROS_DEBUG_THROTTLE(10, "POS_CTRL - Pose (%.2f,%.2f,%2.f,%2.f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, get_yaw(pose.pose));
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
    ROS_DEBUG("POS_CTRL - Stop");

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
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.1);
    double d_turn_timeout;
    nh.param(this_node::getName() + "/turn_timeout", d_turn_timeout, 5.0);
    turn_timeout = Duration(d_turn_timeout);

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

    // wait for fcu connection
    while (ok() && state.connected == false) {
        ROS_DEBUG_ONCE("POS_CTRL - Waiting for FCU connection");
        spinOnce();
        rate->sleep();
    }

    // wait for valid position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("POS_CTRL - Waiting for valid pose");
        spinOnce();
        rate->sleep();
    }

    // service clients
    PoseToTarget_client = nh.serviceClient< mavros_gps::PoseToTarget > ("gps/PoseToTarget");
    if (global)
        PoseToTarget_client.waitForExistence();
    ServiceClient obstacle_client = nh.serviceClient<cpswarm_msgs::ClearOfObstacles>("obstacle_detection/clear_of_obstacles");
    obstacle_client.waitForExistence();
    ServiceClient danger_client = nh.serviceClient<cpswarm_msgs::Danger>("obstacle_detection/danger");
    danger_client.waitForExistence();
    occupied_sector_client = nh.serviceClient<cpswarm_msgs::GetSector>("obstacle_detection/get_occupied_sector");
    occupied_sector_client.waitForExistence();

    // goal publisher
    if (global) {
        publisher = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_position/global", queue_size, true);
    }
    else {
        publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", queue_size, true);
    }

    ROS_INFO("POS_CTRL - Ready");

    // provide position controller
    while (ok()) {
        // get position and goal update
        spinOnce();

        // only publish set point if a goal has been received
        if (goal_valid) {
            // only move if goal is far enough from current pose
            if (enough_dist() || enough_yaw()) {
                // turn nose into movement direction, if required
                if (enough_yaw()) {
                    turn();
                }

                // check if dangerously close to obstacle
                cpswarm_msgs::Danger danger;
                if (danger_client.call(danger) == false) {
                    ROS_ERROR("POS_CTRL - Failed to check if obstacle near by");
                    publish_goal(pose);
                    continue;
                }

                // there are obstacles dangerously close, back off
                else if (danger.response.danger) {
                    ROS_ERROR("POS_CTRL - Obstacle too close, backoff!");
                    publish_goal(compute_goal(danger.response.backoff / 2.0, danger.response.direction));
                    continue;
                }

                // check if clear of obstacles
                else {
                    cpswarm_msgs::ClearOfObstacles obstacle;
                    if (obstacle_client.call(obstacle) == false) {
                        ROS_ERROR("POS_CTRL - Failed to check if clear of obstacles");
                        publish_goal(pose);
                        continue;
                    }

                    // if there is an obstacle, avoid collision
                    else if (obstacle.response.clear == false) {
                        obstacle_avoidance();
                        continue;
                    }
                }
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
