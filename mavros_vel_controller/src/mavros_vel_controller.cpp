#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Empty.h>

using namespace std;
using namespace ros;

//Publishers
Publisher setpoint_pub;
Publisher stop_pos_pub;

//Subscribers
Subscriber state_sub;
Subscriber vel_sub;
Subscriber stop_sub;

/**
 * @brief The velocity at which the CPS shall move.
 */
geometry_msgs::Twist target_velocity;

/**
 * @brief The MAVROS FCU connection state.
 */
mavros_msgs::State state;

bool started = false;

/**
 * Callback function to receive FCU connection state.
 * @param msg The received state.
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    state = *msg;
}

/**
 * Callback function to stop publishing set points.
 * @param msg An empty message.
 */
void stop_cb(const std_msgs::Empty::ConstPtr& msg) {
    ROS_DEBUG("VEL_CTRL - Stop publishing velocity");

    // stop publishing set points
    started = false;

    // reset velocity
    target_velocity.linear.x = 0;
    target_velocity.linear.y = 0;
    target_velocity.linear.z = 0;

    // stop moving
    setpoint_pub.publish(target_velocity);
}

/**
 * Callback function to receive target velocity.
 * @param msg The received CPS pose in local coordinates.
 */
void velocity_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    // set target velocity
    target_velocity = *msg;

    ROS_DEBUG("VEL_CTRL - Move with velocity (lin %.2f,%.2f,%.2f / ang %.2f,%.2f,%.2f)", target_velocity.linear.x, target_velocity.linear.y, target_velocity.linear.z, target_velocity.angular.x, target_velocity.angular.y, target_velocity.angular.z);

    // start publishing velocity set points
    started = true;

    // stop publishing position set points
    std_msgs::Empty stop_msg;
    stop_pos_pub.publish(stop_msg);
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv) {
    // init ros node
    init(argc, argv, "vel_controller");
    NodeHandle nh;

    double freq;

    // get loop rate
    nh.getParam(this_node::getName() + "/frequency", freq);
    Rate rate(freq);

    // publishers
    setpoint_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    stop_pos_pub = nh.advertise<std_msgs::Empty>("pos_controller/stop", 1);

    // subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    vel_sub = nh.subscribe<geometry_msgs::Twist>("vel_controller/target_velocity", 1, velocity_cb);
    stop_sub = nh.subscribe<std_msgs::Empty>("vel_controller/stop", 1, stop_cb);

    // wait for fcu connection
    while (ok() && state.connected == false) {
        spinOnce();
        rate.sleep();
    }

    ROS_INFO("VEL_CTRL - Ready");

    // provide velocity controller
    while (ok()) {
        // get updates
        spinOnce();

        // publish set point
        if (started) {
            setpoint_pub.publish(target_velocity);
        }

        // sleep rest of cycle
        rate.sleep();
    }

    return 0;
}
