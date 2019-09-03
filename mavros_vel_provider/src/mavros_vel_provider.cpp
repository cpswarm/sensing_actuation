#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief Current position of the CPS in local coordinates.
 */
geometry_msgs::TwistStamped velocity;

/**
 * @brief Whether a valid velocity has been received.
 */
bool vel_valid;

/**
 * @brief Callback function for velocity updates.
 * @param msg velocity received from the CPS FCU.
 */
void vel_callback (const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    // store new position and orientation in class variables
    velocity = *msg;

    vel_valid = true;
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
    init(argc, argv, "vel_provider");
    NodeHandle nh;

    // read parameters
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    bool global = pos_type == "local" ? false : true;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // wait for valid velocity
    vel_valid = false;

    // init velocity subscriber
    string vel_topic;
    if (global) {
        vel_topic = "mavros/global_position/gp_vel";
    }
    else {
        vel_topic = "mavros/local_position/velocity_local";
    }
    Subscriber vel_sub = nh.subscribe(vel_topic, queue_size, vel_callback);

    // init velocity publisher
    Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel_provider/velocity", queue_size);

    // wait for valid velocity
    while (ok() && vel_valid == false) {
        ROS_DEBUG_ONCE("VEL_PROV - Waiting for valid velocity");
        spinOnce();
        rate.sleep();
    }

    ROS_INFO("VEL_PROV - Ready");

    // publish velocity locally
    while (ok()) {
        // update velocity information
        spinOnce();

        // publish velocity
        vel_pub.publish(velocity);

        // sleep rest of cycle
        rate.sleep();
    }

    return 0;
}
