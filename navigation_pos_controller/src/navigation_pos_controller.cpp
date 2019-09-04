#include <ros/ros.h>
#include <tf2/utils.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief The goal pose.
 */
geometry_msgs::Pose goal;

/**
 * @brief The position of the CPS.
 */
geometry_msgs::Pose pose;

/*
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
 * Callback function to receive current pose.
 * @param msg The received CPS pose in local coordinates.
 */
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = msg->pose;
    ROS_DEBUG_THROTTLE(10, "POS_CTRL - Local pose (%.2f,%.2f,%2.f)", pose.position.x, pose.position.y, get_yaw(pose));
    pose_valid = true;
}

/**
 * Callback function for the CPS goal.
 * @param msg The CPS goal in local coordinates.
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = msg->pose;

    ROS_DEBUG("POS_CTRL - Local goal (%.2f,%.2f,%.2f)", goal.position.x, goal.position.y, get_yaw(goal));

    goal_valid = true;
    cancel_goal = false;
}

/**
 * Callback function to stop the CPS movement.
 * @param msg An empty message.
 */
void stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_DEBUG("POS_CTRL - Stop moving");
    goal_valid = false;
    cancel_goal = true;
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv)
{
    // init ros node
    init(argc, argv, "pos_controller");
    NodeHandle nh;

    // read parameters
    double x,y;
    nh.param(this_node::getName() + "/x", x, 0.0);
    nh.param(this_node::getName() + "/y", y, 0.0);
    geometry_msgs::Pose origin;
    origin.position.x = x;
    origin.position.y = y;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    double goal_tolerance;
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    double goal_tolerance_a;
    nh.param(this_node::getName() + "/goal_tolerance_a", goal_tolerance_a, 0.1);

    // goal action message
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";

    // wait for valid pose and goal
    pose_valid = false;
    goal_valid = false;
    cancel_goal = false;

    // position subscriber
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);

    // goal publisher
    Publisher publisher = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", queue_size);

    // wait for valid position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("POS_CTRL - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // goal subscribers
    Subscriber goal_sub = nh.subscribe("pos_controller/goal_position", queue_size, goal_callback);
    Subscriber stop_sub = nh.subscribe("pos_controller/stop", queue_size, stop_callback);

    ROS_DEBUG("POS_CTRL - Ready");

    // provide position controller
    while (ok()) {
        // get position and goal update
        spinOnce();

        // only move if goal is far enough from current pose
        if (goal_valid && (hypot(goal.position.x - pose.position.x, goal.position.y - pose.position.y) > goal_tolerance || remainder(get_yaw(pose) - get_yaw(goal), 2*M_PI) + M_PI < goal_tolerance_a)) {
            ROS_DEBUG_THROTTLE(10, "POS_CTRL - Move to (%.2f,%.2f,%.2f).", goal.position.x, goal.position.y, get_yaw(goal));

            // publish goal
            goal_msg.header.stamp = Time::now();
            goal_msg.pose = goal;
            goal_msg.pose.position.x -= origin.position.x;
            goal_msg.pose.position.y -= origin.position.y;
            publisher.publish(goal_msg);
        }

        // stop moving
        else if (cancel_goal){
            goal_msg.header.stamp = Time::now();
            goal_msg.pose = pose;
            goal_msg.pose.position.x -= origin.position.x;
            goal_msg.pose.position.y -= origin.position.y;
            publisher.publish(goal_msg);
            ROS_DEBUG("POS_CTRL - Stop at (%.2f,%.2f,%.2f)", pose.position.x, pose.position.y, get_yaw(pose));
            cancel_goal = false;
            goal_valid = false;
        }

        // sleep rest of cycle
        rate.sleep();
    }

    return 0;
}
