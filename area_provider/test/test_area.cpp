#include <gtest/gtest.h>
#include <ros/ros.h>
#include "cpswarm_msgs/GetDist.h"

using namespace std;
using namespace ros;

/**
 * @brief Test the area library distance calculation.
 */
TEST (NodeTestArea, testDist)
{
    // create service client
    NodeHandle nh;
    ServiceClient dist_client = nh.serviceClient<cpswarm_msgs::GetDist>("area/get_distance");
    dist_client.waitForExistence(Duration(5.0));
    cpswarm_msgs::GetDist gd;
    geometry_msgs::Point point;

    // test empty point
    gd.request.point = point;
    point.x = 0;
    point.y = 0;
    if (dist_client.call(gd)){
        EXPECT_NEAR(gd.response.closest_point.x, 0, 0.01);
        EXPECT_NEAR(gd.response.closest_point.y, -10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.z, 0, 0.01);
        EXPECT_EQ(gd.response.closest_line.size(), 2);
        if (gd.response.closest_line.size() == 2) {
            EXPECT_NEAR(gd.response.closest_line[0].x, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].z, 0, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].z, 0, 0.01);
        }
        EXPECT_NEAR(gd.response.distance, 10, 0.01);
    }
    else{
        ADD_FAILURE();
    }

    // test point on area boundary edge
    point.x = 10;
    point.y = 0;
    gd.request.point = point;
    if (dist_client.call(gd)){
        EXPECT_NEAR(gd.response.closest_point.x, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.y, 0, 0.01);
        EXPECT_NEAR(gd.response.closest_point.z, 0, 0.01);
        EXPECT_EQ(gd.response.closest_line.size(), 2);
        if (gd.response.closest_line.size() == 2) {
            EXPECT_NEAR(gd.response.closest_line[0].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].z, 0, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].y, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].z, 0, 0.01);
        }
        EXPECT_NEAR(gd.response.distance, 0, 0.01);
    }
    else{
        ADD_FAILURE();
    }

    // test point on area boundary vertex
    point.x = 10;
    point.y = 10;
    gd.request.point = point;
    if (dist_client.call(gd)){
        EXPECT_NEAR(gd.response.closest_point.x, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.y, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.z, 0, 0.01);
        EXPECT_EQ(gd.response.closest_line.size(), 2);
        if (gd.response.closest_line.size() == 2) {
            EXPECT_NEAR(gd.response.closest_line[0].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].z, 0, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].y, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].z, 0, 0.01);
        }
        EXPECT_NEAR(gd.response.distance, 0, 0.01);
    }
    else{
        ADD_FAILURE();
    }

    // test point outside of area boundary (closest point on boundary edge)
    point.x = 20;
    point.y = 0;
    gd.request.point = point;
    if (dist_client.call(gd)){
        EXPECT_NEAR(gd.response.closest_point.x, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.y, 0, 0.01);
        EXPECT_NEAR(gd.response.closest_point.z, 0, 0.01);
        EXPECT_EQ(gd.response.closest_line.size(), 2);
        if (gd.response.closest_line.size() == 2) {
            EXPECT_NEAR(gd.response.closest_line[0].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].z, 0, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].y, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].z, 0, 0.01);
        }
        EXPECT_NEAR(gd.response.distance, 10, 0.01);
    }
    else{
        ADD_FAILURE();
    }

    // test point outside of area boundary (closest point on boundary vertex)
    point.x = 20;
    point.y = 20;
    gd.request.point = point;
    if (dist_client.call(gd)){
        EXPECT_NEAR(gd.response.closest_point.x, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.y, 10, 0.01);
        EXPECT_NEAR(gd.response.closest_point.z, 0, 0.01);
        EXPECT_EQ(gd.response.closest_line.size(), 2);
        if (gd.response.closest_line.size() == 2) {
            EXPECT_NEAR(gd.response.closest_line[0].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].y, -10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[0].z, 0, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].x, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].y, 10, 0.01);
            EXPECT_NEAR(gd.response.closest_line[1].z, 0, 0.01);
        }
        EXPECT_NEAR(gd.response.distance, 14.14, 0.01);
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
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
