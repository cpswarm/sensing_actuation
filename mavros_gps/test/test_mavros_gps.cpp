#include <gtest/gtest.h>
#include <ros/ros.h>
#include "cpswarm_msgs/FixToPose.h"
#include "cpswarm_msgs/GeoToPose.h"
#include "cpswarm_msgs/GetGpsFix.h"
#include "cpswarm_msgs/NedToEnu.h"
#include "cpswarm_msgs/PoseToFix.h"
#include "cpswarm_msgs/PoseToGeo.h"
#include "mavros_gps/FixToTarget.h"
#include "mavros_gps/TargetToFix.h"

using namespace std;
using namespace ros;

/**
 * @brief Test the service gps/fix_to_pose.
 *
 * Calculations based on https://gps-coordinates.org/distance-between-coordinates.php.
 */
TEST (NodeTestMavrosGps, testFixToPose)
{
    // create service client
    NodeHandle nh;
    ServiceClient f2p_client = nh.serviceClient<cpswarm_msgs::FixToPose>("gps/fix_to_pose");
    f2p_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::FixToPose f2p;

    // test service
    sensor_msgs::NavSatFix fix;
    fix.header.seq = 123;
    fix.header.stamp = Time(456);
    fix.header.frame_id = "foo";
    fix.latitude = 46.614918;
    fix.longitude = 14.268227;
    fix.altitude = 789;
    f2p.request.fix = fix;
    if (f2p_client.call(f2p)) {
        EXPECT_EQ(f2p.response.pose.header.seq, fix.header.seq);
        EXPECT_EQ(f2p.response.pose.header.stamp, fix.header.stamp);
        EXPECT_EQ(f2p.response.pose.header.frame_id, fix.header.frame_id);
        EXPECT_NEAR(f2p.response.pose.pose.position.x, 229.15, 0.5);
        EXPECT_NEAR(f2p.response.pose.pose.position.y, 222.39, 0.5);
        EXPECT_NEAR(f2p.response.pose.pose.position.z, fix.altitude-500, 0.1);
        EXPECT_NEAR(f2p.response.pose.pose.orientation.x, 0.0, 0.01);
        EXPECT_NEAR(f2p.response.pose.pose.orientation.y, 0.0, 0.01);
        EXPECT_NEAR(f2p.response.pose.pose.orientation.z, 0.376, 0.01);
        EXPECT_NEAR(f2p.response.pose.pose.orientation.w, 0.927, 0.01);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/fix_to_target.
 */
TEST (NodeTestMavrosGps, testFixToTarget)
{
    // create service client
    NodeHandle nh;
    ServiceClient f2t_client = nh.serviceClient<mavros_gps::FixToTarget>("gps/fix_to_target");
    f2t_client.waitForExistence(Duration(5.0));
    mavros_gps::FixToTarget f2t;

    // test service
    sensor_msgs::NavSatFix fix;
    fix.header.seq = 123;
    fix.header.stamp = Time(456);
    fix.header.frame_id = "foo";
    fix.latitude = 46.614918;
    fix.longitude = 14.268227;
    fix.altitude = 789;
    f2t.request.fix = fix;
    f2t.request.yaw = 0.567;
    if (f2t_client.call(f2t)) {
        EXPECT_EQ(f2t.response.target.header.seq, fix.header.seq);
        EXPECT_EQ(f2t.response.target.header.stamp, fix.header.stamp);
        EXPECT_EQ(f2t.response.target.header.frame_id, fix.header.frame_id);
        EXPECT_FLOAT_EQ(f2t.response.target.latitude, fix.latitude);
        EXPECT_FLOAT_EQ(f2t.response.target.longitude, fix.longitude);
        EXPECT_FLOAT_EQ(f2t.response.target.altitude, fix.altitude);
        EXPECT_FLOAT_EQ(f2t.response.target.yaw, f2t.request.yaw);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/geo_to_pose.
 *
 * Calculations based on https://gps-coordinates.org/distance-between-coordinates.php and https://geographiclib.sourceforge.io/cgi-bin/GeoidEval?input=46.612918+14.265227.
 */
TEST (NodeTestMavrosGps, testGeoToPose)
{
    // create service client
    NodeHandle nh;
    ServiceClient g2p_client = nh.serviceClient<cpswarm_msgs::GeoToPose>("gps/geo_to_pose");
    g2p_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::GeoToPose g2p;

    // test service
    geographic_msgs::GeoPoseStamped pose;
    pose.header.seq = 123;
    pose.header.stamp = Time(456);
    pose.header.frame_id = "foo";
    pose.pose.position.latitude = 46.614918;
    pose.pose.position.longitude = 14.268227;
    pose.pose.position.altitude = 789;
    pose.pose.orientation.x = 0.123;
    pose.pose.orientation.y = 0.234;
    pose.pose.orientation.z = 0.345;
    pose.pose.orientation.w = 0.456;
    g2p.request.geo = pose;
    if (g2p_client.call(g2p)) {
        EXPECT_EQ(g2p.response.pose.header.seq, pose.header.seq);
        EXPECT_EQ(g2p.response.pose.header.stamp, pose.header.stamp);
        EXPECT_EQ(g2p.response.pose.header.frame_id, pose.header.frame_id);
        EXPECT_NEAR(g2p.response.pose.pose.position.x, 229.15, 0.5);
        EXPECT_NEAR(g2p.response.pose.pose.position.y, 222.39, 0.5);
        EXPECT_NEAR(g2p.response.pose.pose.position.z, pose.pose.position.altitude+47.5-500, 0.5);
        EXPECT_EQ(g2p.response.pose.pose.orientation.x, pose.pose.orientation.x);
        EXPECT_EQ(g2p.response.pose.pose.orientation.y, pose.pose.orientation.y);
        EXPECT_EQ(g2p.response.pose.pose.orientation.z, pose.pose.orientation.z);
        EXPECT_EQ(g2p.response.pose.pose.orientation.w, pose.pose.orientation.w);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/get_gps_origin.
 */
TEST (NodeTestMavrosGps, testGetGpsOrigin)
{
    // create service client
    NodeHandle nh;
    ServiceClient ggo_client = nh.serviceClient<cpswarm_msgs::GetGpsFix>("gps/get_gps_origin");
    ggo_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::GetGpsFix ggo;

    // test service
    if (ggo_client.call(ggo)) {
        EXPECT_EQ(ggo.response.fix.header.frame_id, "base_link");
        EXPECT_EQ(ggo.response.fix.status.status, 0);
        EXPECT_EQ(ggo.response.fix.status.service, 1);
        EXPECT_FLOAT_EQ(ggo.response.fix.latitude, 46.612918);
        EXPECT_FLOAT_EQ(ggo.response.fix.longitude, 14.265227);
        EXPECT_EQ(ggo.response.fix.altitude, 500);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[0], 1.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[1], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[2], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[3], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[4], 1.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[5], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[6], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[7], 0.0);
        EXPECT_FLOAT_EQ(ggo.response.fix.position_covariance[8], 1.0);
        EXPECT_EQ(ggo.response.fix.position_covariance_type, 2);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/ned_to_enu.
 *
 * Calculations based on https://robotics.stackexchange.com/questions/19669/rotating-ned-to-enu.
 */
TEST (NodeTestMavrosGps, testNedToEnu)
{
    // create service client
    NodeHandle nh;
    ServiceClient n2e_client = nh.serviceClient<cpswarm_msgs::NedToEnu>("gps/ned_to_enu");
    n2e_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::NedToEnu n2e;

    // test service
    n2e.request.yaw = 0.0;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw, 0.001); //
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 1.23;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw, 0.001);
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 2.34;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw, 0.001);
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 3.45;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw, 0.001);
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 4.56;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw, 0.001);
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 5.67;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw + 2.0*M_PI, 0.001);
    }
    else {
        ADD_FAILURE();
    }
    n2e.request.yaw = 6.78;
    if (n2e_client.call(n2e)) {
        EXPECT_NEAR(n2e.response.yaw, M_PI/2.0 - n2e.request.yaw + 2.0*M_PI, 0.001);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/pose_to_fix.
 *
 * Calculations based on https://gps-coordinates.org/distance-between-coordinates.php.
 */
TEST (NodeTestMavrosGps, testPoseToFix)
{
    // create service client
    NodeHandle nh;
    ServiceClient p2f_client = nh.serviceClient<cpswarm_msgs::PoseToFix>("gps/pose_to_fix");
    p2f_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::PoseToFix p2f;

    // test service
    geometry_msgs::PoseStamped pose;
    pose.header.seq = 123;
    pose.header.stamp = Time(456);
    pose.header.frame_id = "foo";
    pose.pose.position.x = 123;
    pose.pose.position.y = 234;
    pose.pose.position.z = 345;
    p2f.request.pose = pose;
    if (p2f_client.call(p2f)) {
        EXPECT_EQ(p2f.response.fix.header.seq, pose.header.seq);
        EXPECT_EQ(p2f.response.fix.header.stamp, pose.header.stamp);
        EXPECT_EQ(p2f.response.fix.header.frame_id, pose.header.frame_id);
        EXPECT_NEAR(p2f.response.fix.latitude, 46.6150224, 0.000005);
        EXPECT_NEAR(p2f.response.fix.longitude, 14.2668373, 0.000005);
        EXPECT_FLOAT_EQ(p2f.response.fix.altitude, 500+pose.pose.position.z);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/pose_to_geo.
 *
 * Calculations based on https://gps-coordinates.org/distance-between-coordinates.php and https://geographiclib.sourceforge.io/cgi-bin/GeoidEval?input=46.6150224+14.2668373.
 */
TEST (NodeTestMavrosGps, testPoseToGeo)
{
    // create service client
    NodeHandle nh;
    ServiceClient p2g_client = nh.serviceClient<cpswarm_msgs::PoseToGeo>("gps/pose_to_geo");
    p2g_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::PoseToGeo p2g;

    // test service
    geometry_msgs::PoseStamped pose;
    pose.header.seq = 123;
    pose.header.stamp = Time(456);
    pose.header.frame_id = "foo";
    pose.pose.position.x = 123;
    pose.pose.position.y = 234;
    pose.pose.position.z = 345;
    pose.pose.orientation.x = 0.123;
    pose.pose.orientation.y = 0.234;
    pose.pose.orientation.z = 0.345;
    pose.pose.orientation.w = 0.456;
    p2g.request.pose = pose;
    if (p2g_client.call(p2g)) {
        EXPECT_EQ(p2g.response.geo.header.seq, pose.header.seq);
        EXPECT_EQ(p2g.response.geo.header.stamp, pose.header.stamp);
        EXPECT_EQ(p2g.response.geo.header.frame_id, pose.header.frame_id);
        EXPECT_NEAR(p2g.response.geo.pose.position.latitude, 46.6150224, 0.000005);
        EXPECT_NEAR(p2g.response.geo.pose.position.longitude, 14.2668373, 0.000005);
        EXPECT_NEAR(p2g.response.geo.pose.position.altitude, 500-47.8027+pose.pose.position.z, 0.0001);
        EXPECT_FLOAT_EQ(p2g.response.geo.pose.orientation.x, pose.pose.orientation.x);
        EXPECT_FLOAT_EQ(p2g.response.geo.pose.orientation.y, pose.pose.orientation.y);
        EXPECT_FLOAT_EQ(p2g.response.geo.pose.orientation.z, pose.pose.orientation.z);
        EXPECT_FLOAT_EQ(p2g.response.geo.pose.orientation.w, pose.pose.orientation.w);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Test the service gps/target_to_fix.
 */
TEST (NodeTestMavrosGps, testTargetToFix)
{
    // create service client
    NodeHandle nh;
    ServiceClient t2f_client = nh.serviceClient<mavros_gps::TargetToFix>("gps/target_to_fix");
    t2f_client.waitForExistence(Duration(5.0));
    mavros_gps::TargetToFix t2f;

    // test service
    mavros_msgs::GlobalPositionTarget target;
    target.header.seq = 123;
    target.header.stamp = Time(456);
    target.header.frame_id = "foo";
    target.latitude = 45.6789;
    target.longitude = 12.3456;
    target.altitude = 123;
    t2f.request.target = target;
    if (t2f_client.call(t2f)) {
        EXPECT_EQ(t2f.response.fix.header.seq, target.header.seq);
        EXPECT_EQ(t2f.response.fix.header.stamp, target.header.stamp);
        EXPECT_EQ(t2f.response.fix.header.frame_id, target.header.frame_id);
        EXPECT_FLOAT_EQ(t2f.response.fix.latitude, target.latitude);
        EXPECT_FLOAT_EQ(t2f.response.fix.longitude, target.longitude);
        EXPECT_FLOAT_EQ(t2f.response.fix.altitude, target.altitude);
    }
    else {
        ADD_FAILURE();
    }
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_mavros_gps");
    return RUN_ALL_TESTS();
}
