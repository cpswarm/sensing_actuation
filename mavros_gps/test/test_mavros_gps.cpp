#include <gtest/gtest.h>
#include <ros/ros.h>
#include "cpswarm_msgs/FixToPose.h"

using namespace std;
using namespace ros;

/**
 * @brief Test the service gps/fix_to_pose.
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
    fix.latitude = 1.23;
    fix.longitude = 4.56;
    fix.altitude = 7.89;
    if (f2p_client.call(f2p)){
        EXPECT_EQ(f2p.response.pose.header.seq, 123);
        // EXPECT_EQ(f2p)
        // EXPECT_NEAR(f2p.response.pose.pose.position.x, 0, 0.01);
    }
    else{
        ADD_FAILURE();
    }
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_mavros_gps");
    return RUN_ALL_TESTS();
}
