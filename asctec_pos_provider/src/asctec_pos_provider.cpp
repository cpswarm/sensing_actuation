#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "cpswarm_msgs/OutOfBounds.h"
#include "cpswarm_msgs/NedToEnu.h"
#include "asctec_msgs/GPSData.h"
#include "asctec_gps/GpsdataToPose.h"
#include "lib/asctec_compass_sensor.h"

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
ServiceClient gpsdata_to_pose_client;

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
void local_pose_callback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // store new position and orientation in class variables
    pose.header = msg->header;
    pose.pose = msg->pose.pose;

    // shift pose according to origin
    pose.pose.position.x += origin.position.x;
    pose.pose.position.y += origin.position.y;

    ROS_DEBUG_THROTTLE(1, "At position (%.2f,%.2f)", pose.pose.position.x, pose.pose.position.y);
}

/**
 * @brief Callback function for position updates.
 * @param msg Position received from the CPS.
 */
void global_pose_callback (const asctec_msgs::GPSData::ConstPtr& msg)
{
    // store updated position converted to local coordinates
    asctec_gps::GpsdataToPose g2p;
    g2p.request.gpsdata = *msg;
    if (gpsdata_to_pose_client.call(g2p))
        pose = g2p.response.pose;
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
    asctec_compass_sensor* yaw_sensor;
    Subscriber pose_sub;
    if (global) {
        pose_sub = nh.subscribe("GPS_DATA", queue_size, global_pose_callback);
        yaw_sensor = new asctec_compass_sensor();
    }
    else {
        pose_sub = nh.subscribe("pelican/pose", queue_size, local_pose_callback);
    }

    // init pose publisher
    Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pos_provider/pose", queue_size);

    // init gps service clients
    gpsdata_to_pose_client = nh.serviceClient<asctec_gps::GpsdataToPose>("gps/gpsdata_to_pose");
    ned_to_enu_client = nh.serviceClient<cpswarm_msgs::NedToEnu>("gps/ned_to_enu");
    if (global) {
        gpsdata_to_pose_client.waitForExistence();
        ned_to_enu_client.waitForExistence();
    }

    // init tf publisher
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    // wait for valid position
    Duration(init_time).sleep();
    while (ok() && (pose.pose.position.x == 0 || pose.pose.orientation.x == 0)) {
        ROS_DEBUG_ONCE("POS_PROV - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // make sure position is within allowed area
    ServiceClient out_of_bounds_client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    out_of_bounds_client.waitForExistence();
    cpswarm_msgs::OutOfBounds oob;
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

        // convert orientation to local coordinates
        if (global) {
            tf2::Quaternion orientation;
            cpswarm_msgs::NedToEnu n2e;
            n2e.request.yaw = yaw_sensor->get_yaw();
            if (ned_to_enu_client.call(n2e))
                orientation.setRPY(0, 0, n2e.response.yaw);
            else
                ROS_ERROR("POS_PROV - Failed to convert global pose");
            pose.pose.orientation = tf2::toMsg(orientation);
        }

        ROS_DEBUG_THROTTLE(1, "POS_PROV - Current position (%.2f,%.2f,%.2f,%.2f)", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, get_yaw(pose.pose));

        // publish position
        pose.header.stamp = Time::now();
        pose.header.frame_id = "local_origin_ned";
        pose_pub.publish(pose);

        // broadcast tf
        transformStamped.header.stamp = Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "base_link";
        transformStamped.transform.translation.x = pose.pose.position.x;
        transformStamped.transform.translation.y = pose.pose.position.y;
        transformStamped.transform.translation.z = pose.pose.position.z;
        transformStamped.transform.rotation.x = pose.pose.orientation.x;
        transformStamped.transform.rotation.y = pose.pose.orientation.y;
        transformStamped.transform.rotation.z = pose.pose.orientation.z;
        transformStamped.transform.rotation.w = pose.pose.orientation.w;
        br.sendTransform(transformStamped);

        rate.sleep();
    }

    return 0;
}
