#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <asctec_msgs/GPSData.h>
#include <asctec_gps/PoseToGpsdata.h>
#include <asctec_hl_comm/WaypointAction.h>

using namespace std;
using namespace ros;

/**
 * @brief Definition of the action client for moving the robot.
 */
typedef actionlib::SimpleActionClient<asctec_hl_comm::WaypointAction> WaypointClient;

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

/**
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Whether the goal is valid and should be pursued.
 */
bool goal_valid;

/**
 * @brief Whether the goal is invalid and should be canceled.
 */
bool cancel_goal;

/**
 * @brief The maximum speed in m/s which the UAV should travel in any direction.
 */
double max_speed;

/**
 * @brief The distance in meter that the CPS can be away from a goal while still being considered to have reached that goal. Should be similar to the tolerance of the swarm behavior.
 */
double goal_tolerance;

/**
 * @brief The angle in radian which the yaw of the CPS can vary around the set point.
 */
double yaw_tolerance;

/**
 * @brief The time in seconds that reaching a waypoint is allowed to take. Should be shorter than the timeout of the swarm behavior.
 */
double move_timeout;

/**
 * @brief Whether to publish the waypoints on a topic for visualization.
 */
bool visualize;

/**
 * @brief Whether global GPS coordinates should be published.
 */
bool global;

/**
 * @brief Service client to get the GPS target from the local pose.
 */
ServiceClient pose_to_gpsdata_client;

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
 * @brief Publish the waypoint for visualization.
 * @param goal The goal the CPS moves to.
 */
void publish_wp (geometry_msgs::PoseStamped goal)
{
    geometry_msgs::PointStamped wp;
    wp.header.stamp = Time::now();
    wp.header.frame_id = "local_origin_ned";
    wp.point = goal.pose.position;
    wp_pub.publish(wp);
}

/**
 * @brief Define the goal waypoint from the current local goal variable. Possibly convert to GPS coordinates if required.
 * @return The goal waypoint as goal for asctec UAVs.
 */
asctec_hl_comm::WaypointGoal get_goal ()
{
    // create asctec goal message
    asctec_hl_comm::WaypointGoal goal;

    // set goal position as gps coordinates
    if (global) {
        // convert local goal to gps coordinates
        asctec_msgs::GPSData global_goal;
        asctec_gps::PoseToGpsdata p2g;
        p2g.request.pose = local_goal;
        if (pose_to_gpsdata_client.call(p2g)) {
            global_goal = p2g.response.gpsdata;
            goal.goal_pos.x = global_goal.latitude;
            goal.goal_pos.y = global_goal.longitude;
            goal.goal_pos.z = double(global_goal.height) / 1000;
            goal.goal_yaw = double(global_goal.heading);

            ROS_DEBUG("Global position set point (%f,%f,%.2f,%.2f)", goal.goal_pos.x, goal.goal_pos.y, goal.goal_pos.z, goal.goal_yaw);
        }
        else {
            ROS_ERROR("Failed to convert global goal");
        }
    }

    // set goal position as local coordinates
    else {
        goal.goal_pos.x = local_goal.pose.position.x;
        goal.goal_pos.y = local_goal.pose.position.y;
        goal.goal_pos.z = local_goal.pose.position.z;
        goal.goal_yaw = get_yaw(local_goal.pose);

        // shift local goal according to origin
        goal.goal_pos.x -= origin.position.x;
        goal.goal_pos.y -= origin.position.y;

        ROS_DEBUG("Local position set point (%.2f,%.2f,%.2f,%.2f)", goal.goal_pos.x, goal.goal_pos.y, goal.goal_pos.z, goal.goal_yaw);
    }

    // set meta data
//     goal.header.seq++; // would require global variable
    goal.header.stamp = Time::now();
    //     goal.id = goal.header.seq;
    goal.max_speed.x = max_speed;
    goal.max_speed.y = max_speed;
    goal.max_speed.z = max_speed;
    goal.accuracy_position = goal_tolerance;
    goal.accuracy_orientation = yaw_tolerance;
    goal.timeout = move_timeout;

    return goal;
}

/**
 * Callback function for the CPS goal.
 * @param msg The CPS goal in local coordinates.
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store goal position
    local_goal = *msg;

    // calculate orientation
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, atan2(local_goal.pose.position.y - pose.pose.position.y, local_goal.pose.position.x - pose.pose.position.x));
    local_goal.pose.orientation = tf2::toMsg(orientation);

    ROS_DEBUG("Move to (%.2f,%.2f,%.2f,%.2f)", local_goal.pose.position.x, local_goal.pose.position.y, local_goal.pose.position.z, get_yaw(local_goal.pose));

    // valid goal received
    goal_valid = true;
    cancel_goal = false;
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
 * Callback function to stop publishing set points.
 * @param msg An empty message.
 */
void stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_DEBUG("Stop");

    // stop publishing set points
    goal_valid = false;
    cancel_goal = true;
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
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/max_speed", max_speed, 0.5);
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    nh.param(this_node::getName() + "/yaw_tolerance", yaw_tolerance, 0.1);
    nh.param(this_node::getName() + "/move_timeout", move_timeout, 5.0);
    nh.param(this_node::getName() + "/visualize", visualize, false);

    // wait for valid pose and goal
    pose_valid = false;
    goal_valid = false;
    cancel_goal = false;

    // position subscriber
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);

    // TODO
    Publisher test_pub = nh.advertise<geometry_msgs::Twist>("pelican/cmd", queue_size);

    // waypoint navigation action client in separate thread
    WaypointClient ac("fcu/waypoint", true);

    // wait for valid position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // wait for waypoint navigation action server
//     ac.waitForServer();

    // goal subscribers
    Subscriber goal_sub = nh.subscribe("pos_controller/goal_position", queue_size, goal_callback);
    Subscriber stop_sub = nh.subscribe("pos_controller/stop", queue_size, stop_callback);

    // visualization of waypoints
    if (visualize) {
        wp_pub = nh.advertise<geometry_msgs::PointStamped>("pos_controller/waypoint", queue_size, true);
    }

    // service clients
    pose_to_gpsdata_client = nh.serviceClient<asctec_gps::PoseToGpsdata>("gps/pose_to_gpsdata");
    if (global)
        pose_to_gpsdata_client.waitForExistence();

    ROS_INFO("Ready");

    // provide position controller
    while (ok()) {
        // get position and goal update
        spinOnce();

        geometry_msgs::Twist test_msg;
        test_msg.linear.z = 1.0;

        test_pub.publish(test_msg);

        // only move if goal is far enough from current pose
//         if (goal_valid && (hypot(local_goal.pose.position.x - pose.pose.position.x, local_goal.pose.position.y - pose.pose.position.y) >= goal_tolerance || abs(remainder(get_yaw(pose.pose) - get_yaw(local_goal.pose), 2*M_PI)) >= yaw_tolerance)) {
//
//             ROS_DEBUG("POS_CTRL - Move to (%.2f,%.2f,%.2f).", local_goal.pose.position.x, local_goal.pose.position.y, get_yaw(local_goal.pose));
//
//             // send goal to uav
//             ac.sendGoalAndWait(get_goal());
//
//             // wait while goal is actively pursued
//             while (ac.getState() == actionlib::SimpleClientGoalState::PENDING || ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
//                 // check state
//                 if (ac.getState() == actionlib::SimpleClientGoalState::PENDING)
//                     ROS_DEBUG_THROTTLE(10, "POS_CTRL - Goal pending...");
//                 else
//                     ROS_DEBUG_THROTTLE(10, "POS_CTRL - Goal active...");
//
//                 spinOnce();
//
//                 // stop moving
//                 if (cancel_goal == true || goal_valid == false) {
//                     ac.cancelGoal();
//                     ROS_DEBUG("Goal canceled");
//                     cancel_goal = false;
//                     break;
//                 }
//
//                 rate.sleep();
//             }
//
//             // action server finished successfully
//             if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
//                 ROS_DEBUG("POS_CTRL - Reached goal (%.2f,%.2f,%.2f).", local_goal.pose.position.x, local_goal.pose.position.y, get_yaw(local_goal.pose));
//             }
//
//             // action server failed
//             else {
//                 ROS_ERROR("POS_CTRL - Failed to reach goal (%.2f,%.2f,%.2f), goal %s.", local_goal.pose.position.x, local_goal.pose.position.y, get_yaw(local_goal.pose), ac.getState().toString().c_str());
//             }
//         }
//
//         // stop moving
//         else if (cancel_goal){
//             ac.cancelAllGoals();
//             ROS_DEBUG("Goal canceled");
//             cancel_goal = false;
//             goal_valid = false;
//         }

        // sleep rest of cycle
        rate.sleep();
    }

    return 0;
}
