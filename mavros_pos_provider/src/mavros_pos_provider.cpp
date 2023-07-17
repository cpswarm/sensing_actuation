#include <ros/ros.h>
#include <tf2/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include "cpswarm_msgs/OutOfBounds.h"
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/NedToEnu.h"
#include "lib/mavros_imu_mag_sensor.h"
#include "lib/mavros_compass_sensor.h"

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
 * @brief Service client to convert a GPS fix to a local pose.
 */
ServiceClient fix_to_pose_client;

/**
 * Service client to convert the yaw from NED to ENU.
 */
ServiceClient ned_to_enu_client;

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
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void local_pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store new position and orientation in class variables
    pose = *msg;

    // shift pose according to origin
    pose.pose.position.x += origin.position.x;
    pose.pose.position.y += origin.position.y;
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void global_pose_callback (const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // store updated position converted to local coordinates
    cpswarm_msgs::FixToPose f2p;
    f2p.request.fix = *msg;
    if (fix_to_pose_client.call(f2p)) {
        pose = f2p.response.pose;
        ROS_DEBUG("Converted global position %f,%f,%f to local position %f,%f,%f / %f,%f,%f,%f", msg->longitude, msg->latitude, msg->altitude, pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    }
    else
        ROS_ERROR("POS_PROV - Failed to convert global pose");
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
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    bool global = pos_type == "local" ? false : true;
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
    double init_time;
    nh.param(this_node::getName() + "/init_time", init_time, 10.0);

    // init pose subscribers
    mavros_compass_sensor* yaw_sensor;
    Subscriber yaw_sub, pose_sub;
    if (global) {
        pose_sub = nh.subscribe("mavros/global_position/global", queue_size, global_pose_callback);
        yaw_sensor = new mavros_compass_sensor();
    }
    else {
        pose_sub = nh.subscribe("mavros/local_position/pose", queue_size, local_pose_callback);
    }

    // init pose publisher
    Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size);

    // init gps service clients
    fix_to_pose_client = nh.serviceClient<cpswarm_msgs::FixToPose>("gps/fix_to_pose");
    ned_to_enu_client = nh.serviceClient<cpswarm_msgs::NedToEnu>("gps/ned_to_enu");
    if (global) {
        fix_to_pose_client.waitForExistence();
        ned_to_enu_client.waitForExistence();
    }

    // wait for valid position
    ROS_DEBUG("POS_PROV - Delay startup by %.2f s", init_time);
    Duration(init_time).sleep();
    while (ok() && (pose.pose.position.x == 0 && pose.pose.position.y == 0 && pose.pose.position.z == 0)) {
        ROS_DEBUG_THROTTLE(1, "POS_PROV - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // make sure position is within allowed area
    ServiceClient out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    ROS_DEBUG("POS_PROV - Waiting for out of bounds service");
    out_of_bounds_client.waitForExistence();
    cpswarm_msgs::OutOfBounds oob;
    oob.request.pose = pose.pose;
    if (out_of_bounds_client.call(oob)) {
        if (oob.response.out == true){
            ROS_FATAL("CPS (%.2f,%.2f) is not within allowed area!", pose.pose.position.x, pose.pose.position.y);
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

        // convert orientation to local coordinates
        if (global) {
            tf2::Quaternion orientation;
            cpswarm_msgs::NedToEnu n2e;
            n2e.request.yaw = yaw_sensor->get_yaw();
            if (ned_to_enu_client.call(n2e)) {
                ROS_DEBUG("Convert yaw: %.2f --> %.2f", yaw_sensor->get_yaw(), n2e.response.yaw);
                orientation.setRPY(0, 0, n2e.response.yaw);
            }
            else
                ROS_ERROR("POS_PROV - Failed to convert global pose");

            pose.pose.orientation = tf2::toMsg(orientation);
        }

        ROS_DEBUG_THROTTLE(1, "POS_PROV - Current position (%.2f,%.2f,%.2f,%.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, get_yaw(pose.pose));

        // publish position
        pose.header.stamp = Time::now();
        pose.header.frame_id = "map";
        pose_pub.publish(pose);

        rate.sleep();
    }

    return 0;
}
