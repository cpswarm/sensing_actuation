#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "cpswarm_msgs/GetPoints.h"
#include "cpswarm_msgs/GetPoint.h"
#include "cpswarm_msgs/GetDist.h"
#include "cpswarm_msgs/GetMap.h"
#include "cpswarm_msgs/OutOfBounds.h"

using namespace std;
using namespace ros;

/**
 * @brief Output grid data to the console for visualization/testing purpose.
 * @param grid The occupancy grid map to visualize.
 */
void print_grid (nav_msgs::OccupancyGrid& grid)
{
    for (int i=grid.info.height-1; i>=0; --i) {
        for (int j=0; j<grid.info.width; ++j) {
            int8_t cell = grid.data[i*grid.info.width + j];
            if (cell == 0) {
                if (i == grid.info.height - 1) {
                    if (j == grid.info.width - 1)
                        cerr << "‾⌉";
                    else if (j == 0)
                        cerr << "⌈‾";
                    else
                        cerr << "‾‾";
                }
                else if (i == 0) {
                    if (j == grid.info.width - 1)
                        cerr << "_⌋";
                    else if (j == 0)
                        cerr << "⌊_";
                    else
                        cerr << "__";
                }
                else if (j == 0)
                    cerr << "⎢ ";
                else if (j == grid.info.width - 1)
                    cerr << " ⎥";
                else
                    cerr << "  ";
            }
            else if (cell == 100)
                cerr << "██";
            else
                cerr << "  ";
        }
        cerr << endl;
    }
}

/**
 * @brief Test the mission area get area service.
 */
TEST (NodeTestMissionArea, testGetArea)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetPoints>("area/get_area");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetPoints msg;

    // test response
    ASSERT_TRUE(client.call(msg));
    ASSERT_EQ(msg.response.points.size(), 4);
    EXPECT_NEAR(msg.response.points[0].x, -6, 0.01);
    EXPECT_NEAR(msg.response.points[0].y, -6, 0.01);
    EXPECT_NEAR(msg.response.points[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.points[1].x, 6, 0.01);
    EXPECT_NEAR(msg.response.points[1].y, -4, 0.01);
    EXPECT_NEAR(msg.response.points[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.points[2].x, 6, 0.01);
    EXPECT_NEAR(msg.response.points[2].y, 4, 0.01);
    EXPECT_NEAR(msg.response.points[2].z, 0, 0.01);
    EXPECT_NEAR(msg.response.points[3].x, -6, 0.01);
    EXPECT_NEAR(msg.response.points[3].y, 6, 0.01);
    EXPECT_NEAR(msg.response.points[3].z, 0, 0.01);
}

/**
 * @brief Test the mission area get center service.
 */
TEST (NodeTestMissionArea, testGetCenter)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetPoint>("area/get_center");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetPoint msg;

    // test response
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.point.x, -0.4, 0.01);
    EXPECT_NEAR(msg.response.point.y, 0, 0.01);
    EXPECT_NEAR(msg.response.point.z, 0, 0.01);
}

/**
 * @brief Test the mission area get distance service.
 */
TEST (NodeTestMissionArea, testGetDistance)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetDist>("area/get_distance");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetDist msg;

    // test empty point (origin)
    msg.request.point.x = 0;
    msg.request.point.y = 0;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.closest_point.x, -6, 0.01);
    EXPECT_NEAR(msg.response.closest_point.y, 2.5, 0.01);
    EXPECT_NEAR(msg.response.closest_point.z, 0, 0.01);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_NEAR(msg.response.closest_line[0].x, -6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].y, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].x, -6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].y, -6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.distance, 2.5, 0.01);

    // test point on area boundary edge
    msg.request.point.x = 6;
    msg.request.point.y = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.closest_point.x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_point.y, -1, 0.01);
    EXPECT_NEAR(msg.response.closest_point.z, 0, 0.01);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_NEAR(msg.response.closest_line[0].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].y, -4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.distance, 0, 0.01);

    // test point on area boundary vertex
    msg.request.point.x = 6;
    msg.request.point.y = 4;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.closest_point.x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_point.y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_point.z, 0, 0.01);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_NEAR(msg.response.closest_line[0].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].y, -4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.distance, 0, 0.01);

    // test point outside of area boundary (closest point on boundary edge)
    msg.request.point.x = 12;
    msg.request.point.y = -2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.closest_point.x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_point.y, -2, 0.01);
    EXPECT_NEAR(msg.response.closest_point.z, 0, 0.01);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_NEAR(msg.response.closest_line[0].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].y, -4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.distance, 6, 0.01);

    // test point outside of area boundary (closest point on boundary vertex)
    msg.request.point.x = 12;
    msg.request.point.y = 8;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.closest_point.x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_point.y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_point.z, 0, 0.01);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_NEAR(msg.response.closest_line[0].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].y, -4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[0].z, 0, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].x, 6, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].y, 4, 0.01);
    EXPECT_NEAR(msg.response.closest_line[1].z, 0, 0.01);
    EXPECT_NEAR(msg.response.distance, 7.21, 0.01);
}

/**
 * @brief Test the mission area get map service.
 */
TEST (NodeTestMissionArea, testGetMap)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetMap>("area/get_map");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetMap msg;

    // test resolution higher than parameter server
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 0.25;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.map.info.resolution, 0.25, 0.01);
    EXPECT_NEAR(msg.response.map.info.width, 48, 0.01);
    EXPECT_NEAR(msg.response.map.info.height, 48, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.x, -6, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.y, -6, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.z, 0, 0.01);
    ASSERT_EQ(msg.response.map.data.size(), 2304);
    // count number of unexpected cells
    int err = 0;
    for (int i=0; i<48; ++i) {
        for (int j=0; j<48; ++j) {
            if (i < 24) {
                if ((j+0.5) < 6.0 * (i+0.5)) {
                    if (msg.response.map.data[i*48 + j] != 0)
                        ++err;
                }
                else {
                    if (msg.response.map.data[i*48 + j] != 100)
                        ++err;
                }
            }
            else {
                if ((j+0.5) <= 6 * (48-(i+0.5))) {
                    if (msg.response.map.data[i*48 + j] != 0)
                        ++err;
                }
                else {
                    if (msg.response.map.data[i*48 + j] != 100)
                        ++err;
                }
            }
        }
    }
    EXPECT_LE(err, 2/msg.response.map.info.resolution); // maximum number of unexpected cells
    EXPECT_NEAR(msg.response.rotation, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.x, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.y, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.z, 0, 0.01);

    // test resolution lower than parameter server map
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 2.5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.map.info.resolution, 2.5, 0.01);
    EXPECT_NEAR(msg.response.map.info.width, 5, 0.01);
    EXPECT_NEAR(msg.response.map.info.height, 5, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.x, -6, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.y, -6, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.z, 0, 0.01);
    ASSERT_EQ(msg.response.map.data.size(), 25);
    for (int i=0; i<5; ++i) {
        for (int j=0; j<5; ++j) {
            if (i == 0 && j >2 || i == 4 && j > 1)
                EXPECT_EQ(msg.response.map.data[i*5 + j], 100);
            else
                EXPECT_EQ(msg.response.map.data[i*5 + j], 0);
        }
    }
    EXPECT_NEAR(msg.response.rotation, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.x, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.y, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.z, 0, 0.01);

    // test rotation
    msg.request.rotate = true;
    msg.request.translate = false;
    msg.request.resolution = 0.25;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.map.info.resolution, 0.25, 0.01);
    EXPECT_NEAR(msg.response.map.info.width, 54, 0.01);
    EXPECT_NEAR(msg.response.map.info.height, 48, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.x, -6.90, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.y, -4.93, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.z, 0, 0.01);
    EXPECT_EQ(msg.response.map.data.size(), 2592);
    // dimensions of rotated quadrilateral
    double hl = 12.0 / double(msg.request.resolution);
    double hr = 8.0 / double(msg.request.resolution);
    double bot = hl / cos(1.0/6.0);
    double right = hr * cos(1.0/6.0);
    double left = hl * cos(1.0/6.0) + hl * sin(1.0/6.0) * tan(2.0/6.0) - 0.42543043852201026225; // 26225 < 263 correction factor found by trial and error
    // count number of unexpected cells
    err = 0;
    for (int i=0; i<48; ++i) {
        for (int j=0; j<54; ++j) {
            if ((i+0.5) < right) {
                if ((i+0.5) / 6.0 <= j+0.5 && j+0.5 <= bot + (i+0.5) / 6.0) {
                    if (msg.response.map.data[i*54 + j] != 0)
                        ++err;
                }
                else {
                    if (msg.response.map.data[i*54 + j] != 100)
                        ++err;
                }
            }
            else {
                if ((i+0.5) / 6.0 <= j+0.5 && j+0.5 <= 3.0 * (left - (i+0.5))) {
                    if (msg.response.map.data[i*54 + j] != 0)
                        ++err;
                }
                else {
                    if (msg.response.map.data[i*54 + j] != 100)
                        ++err;
                }
            }
        }
    }
    EXPECT_LE(err, 2/msg.response.map.info.resolution); // maximum number of unexpected cells
    EXPECT_NEAR(msg.response.rotation, -0.165, 0.01);
    EXPECT_NEAR(msg.response.translation.x, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.y, 0, 0.01);
    EXPECT_NEAR(msg.response.translation.z, 0, 0.01);

    // test rotation and translation
    msg.request.rotate = true;
    msg.request.translate = true;
    msg.request.resolution = 0.25;
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.map.info.resolution, 0.25, 0.01);
    EXPECT_NEAR(msg.response.map.info.width, 54, 0.01);
    EXPECT_NEAR(msg.response.map.info.height, 48, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.x, -7, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.y, -5, 0.01);
    EXPECT_NEAR(msg.response.map.info.origin.position.z, 0, 0.01);
    EXPECT_EQ(msg.response.map.data.size(), 2592);
    EXPECT_NEAR(msg.response.rotation, -0.165, 0.01);
    EXPECT_NEAR(msg.response.translation.x, -0.1, 0.01);
    EXPECT_NEAR(msg.response.translation.y, -0.07, 0.01);
    EXPECT_NEAR(msg.response.translation.z, 0, 0.01);
}

/**
 * @brief Test the mission area get origin service.
 */
TEST (NodeTestMissionArea, testGetOrigin)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetPoint>("area/get_origin");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetPoint msg;

    // test response
    ASSERT_TRUE(client.call(msg));
    EXPECT_NEAR(msg.response.point.x, -3.5, 0.01);
    EXPECT_NEAR(msg.response.point.y, 2.5, 0.01);
    EXPECT_NEAR(msg.response.point.z, 0, 0.01);
}

/**
 * @brief Test the mission area out of bounds service.
 */
TEST (NodeTestMissionArea, testOutOfBounds)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::OutOfBounds>("area/out_of_bounds");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::OutOfBounds msg;

    // test points inside
    msg.request.pose.position.x = 0;
    msg.request.pose.position.y = 0;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = 5;
    msg.request.pose.position.y = 3;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = -4.57;
    msg.request.pose.position.y = -4.1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);

    // test points on boundary edges
    msg.request.pose.position.x = 6;
    msg.request.pose.position.y = 3;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = 0;
    msg.request.pose.position.y = 5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = 0;
    msg.request.pose.position.y = -5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = -6;
    msg.request.pose.position.y = -4.1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);

    // test points on boundary vertices
    msg.request.pose.position.x = -6;
    msg.request.pose.position.y = -6;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = 6;
    msg.request.pose.position.y = -4;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = 6;
    msg.request.pose.position.y = 4;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);
    msg.request.pose.position.x = -6;
    msg.request.pose.position.y = 6;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.out);

    // test points outside
    msg.request.pose.position.x = 6.1;
    msg.request.pose.position.y = 3;
    ASSERT_TRUE(client.call(msg));
    EXPECT_TRUE(msg.response.out);
    msg.request.pose.position.x = -7;
    msg.request.pose.position.y = 5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_TRUE(msg.response.out);
    msg.request.pose.position.x = -7;
    msg.request.pose.position.y = -8;
    ASSERT_TRUE(client.call(msg));
    EXPECT_TRUE(msg.response.out);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
