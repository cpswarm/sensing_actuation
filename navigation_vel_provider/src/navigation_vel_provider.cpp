#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
using namespace ros;

/**
 * @brief Current position of the CPS in local coordinates.
 */
geometry_msgs::PoseStamped pose;

/**
 * @brief Current velocity of the CPS.
 */
geometry_msgs::TwistStamped vel;

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void local_pose_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // elapsed time
    double dt = (msg->header.stamp - pose.header.stamp).toSec();

    // compute momentary velocity
    vel.twist.linear.x = (msg->pose.pose.position.x - pose.pose.position.x) / dt;
    vel.twist.linear.y = (msg->pose.pose.position.y - pose.pose.position.y) / dt;
    vel.twist.linear.z = (msg->pose.pose.position.z - pose.pose.position.z) / dt;
    // TODO: compute angular velocity
    // http://docs.ros.org/kinetic/api/tf2/html/namespacetf2.html
    // dq(t)/dt = .5* v * q(t),
    // => v = 2 * dq(t)/dt * q'(t)
    // where q'(t) is the quaternion conjugate.
    // dq(t) = q(t+dt) - q(t)
//     vel.angular = 2 * (msg->pose.pose.orientation - pose.pose.orientation) / t * conj(pose.pose.orientation)
    vel.twist.angular.x = 0; // rotation around x axis
    vel.twist.angular.y = 0; // ...
    vel.twist.angular.z = 0;

    // store new position and orientation in class variables
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
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
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init pose subscriber
    Subscriber pose_sub = nh.subscribe("amcl_pose", queue_size, local_pose_callback);

    // init velocity publisher
    Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("vel_provider/velocity", queue_size);

    // wait for valid position
    while (ok() && pose.header.stamp.isValid() == false) {
        ROS_DEBUG_ONCE("VEL_PROV - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    ROS_DEBUG("VEL_PROV - Ready");

    // publish velocity locally
    while (ok()) {
        // update position information
        spinOnce();

        // publish velocity
        vel.header.stamp = Time::now();
        vel_pub.publish(vel);

        rate.sleep();
    }

    return 0;
}
