#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "cpswarm_msgs/out_of_bounds.h"
#include "angle.h"

using namespace std;
using namespace ros;

/**
 * @brief Current position of the CPS in local coordinates.
 */
geometry_msgs::PoseStamped pose;

/**
 * @brief Origin of the CPS in local coordinates.
 */
geometry_msgs::Pose origin;

/**
 * @brief Compute the yaw angle from a given pose.
 * @param pose The pose to compute the angle from.
 * @return An angle that represents the orientation of the pose.
 */
angle get_yaw (geometry_msgs::Pose pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return angle(tf2::getYaw(orientation));
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void local_pose_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // store new position and orientation in class variables
    pose.pose = msg->pose.pose;

    // shift pose according to origin
    pose.pose.position.x += origin.position.x;
    pose.pose.position.y += origin.position.y;

    ROS_DEBUG_THROTTLE(1, "POS_PROV - Pose (%.2f,%.2f,%.2f)", pose.pose.position.x, pose.pose.position.y, get_yaw(pose.pose).rad_pos());
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
    init(argc, argv, "pos_provider");
    NodeHandle nh;

    // read parameters
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

    // init pose subscribers
    Subscriber pose_sub = nh.subscribe("amcl_pose", queue_size, local_pose_callback);

    // init pose publisher
    Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size);

    // wait for valid position
    while (ok() && pose.header.stamp.isValid() == false) {
        ROS_DEBUG_ONCE("POS_PROV - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // make sure position is within allowed area
    ServiceClient out_of_bounds_client = nh.serviceClient<cpswarm_msgs::out_of_bounds>("area/out_of_bounds");
    cpswarm_msgs::out_of_bounds oob;
    oob.request.pose = pose.pose;
    if (out_of_bounds_client.call(oob)) {
        if (oob.response.out == true){
            ROS_FATAL("CPS is not within allowed area!");
            shutdown();
        }
    }
    else
        ROS_ERROR("POS_PROV - Failed to check if CPS is within allowed area");

    ROS_DEBUG("POS_PROV - Ready");

    // publish position locally
    while (ok()) {
        // update position information
        spinOnce();

        // publish position
        pose.header.stamp = Time::now();
        pose_pub.publish(pose);

        rate.sleep();
    }

    return 0;
}
