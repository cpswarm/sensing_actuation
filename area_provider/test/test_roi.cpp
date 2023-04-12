#include <queue>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>
#include "cpswarm_msgs/GetMultiPoints.h"
#include "cpswarm_msgs/GetDist.h"
#include "cpswarm_msgs/GetMap.h"
#include "cpswarm_msgs/SetRoiState.h"
#include "cpswarm_msgs/PointArrayEvent.h"

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
 * @brief A queue of ROI event messages to store the ROIs received by the subscriber.
 */
queue<cpswarm_msgs::PointArrayEvent> rois;

/**
 * @brief Callback function to receive ROIs published by the ROI services node.
 *
 * @param roi The ROI event message.
 */
void roi_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& roi)
{
    ROS_ERROR("REceived ROI");
    rois.push(*roi);
}

/**
 * @brief Test the roi get all service.
 */
TEST (NodeTestRoi, testGetAll)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetMultiPoints>("rois/get_all");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetMultiPoints msg;

    // test response
    ASSERT_TRUE(client.call(msg));
    ASSERT_EQ(msg.response.layout.dim.size(), 2);
    EXPECT_EQ(msg.response.layout.dim[0].size, 4);
    EXPECT_EQ(msg.response.layout.dim[0].stride, 20);
    EXPECT_EQ(msg.response.layout.dim[1].size, 5);
    EXPECT_EQ(msg.response.layout.dim[1].stride, 5);
    EXPECT_EQ(msg.response.layout.data_offset, 0);
    ASSERT_EQ(msg.response.points.size(), 20);

    EXPECT_FLOAT_EQ(msg.response.points[0].x, -7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[0].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[0].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[1].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[1].y, 10);
    EXPECT_FLOAT_EQ(msg.response.points[1].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[2].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[2].y, 24.1421356237309);
    EXPECT_FLOAT_EQ(msg.response.points[2].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[3].x, 7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[3].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[3].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[4].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[5].x, -3);
    EXPECT_FLOAT_EQ(msg.response.points[5].y, 4);
    EXPECT_FLOAT_EQ(msg.response.points[5].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[6].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[6].y, 2);
    EXPECT_FLOAT_EQ(msg.response.points[6].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[7].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[7].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[7].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[8].x, -1);
    EXPECT_FLOAT_EQ(msg.response.points[8].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[8].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[9].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[9].y, 3);
    EXPECT_FLOAT_EQ(msg.response.points[9].z, 50);

    EXPECT_FLOAT_EQ(msg.response.points[10].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[10].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[10].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[11].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[11].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[11].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[12].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[12].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[12].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[13].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[13].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[13].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[14].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[15].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[15].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[15].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[16].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[16].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[16].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[17].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[17].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[17].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[18].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[18].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[18].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[19].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].z, 0);
}

/**
 * @brief Test the roi get closest service.
 */
TEST (NodeTestRoi, testGetClosest)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetDist>("rois/get_closest");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // create request message
    cpswarm_msgs::GetDist msg;
    // coordinates should be ignored
    geometry_msgs::Point coord;
    coord.x = 1;
    coord.y = 2;
    msg.request.coords.push_back(coord);
    coord.x = 3;
    coord.y = 4;
    msg.request.coords.push_back(coord);
    coord.x = 5;
    coord.y = 6;
    msg.request.coords.push_back(coord);

    // test empty point, origin will be used
    msg.request.point.x = 0;
    msg.request.point.y = 0;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, -2.5);
    EXPECT_EQ(msg.response.closest_point.y, 3);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, -3);
    EXPECT_EQ(msg.response.closest_line[0].y, 4);
    EXPECT_EQ(msg.response.closest_line[1].x, -2);
    EXPECT_EQ(msg.response.closest_line[1].y, 2);
    EXPECT_FLOAT_EQ(msg.response.distance, sqrt(5)/2.0);
    ASSERT_EQ(msg.response.coords.size(), 5);
    EXPECT_EQ(msg.response.coords[0].x, -3);
    EXPECT_EQ(msg.response.coords[0].y, 4);
    EXPECT_EQ(msg.response.coords[1].x, -2);
    EXPECT_EQ(msg.response.coords[1].y, 2);
    EXPECT_EQ(msg.response.coords[2].x, -2);
    EXPECT_EQ(msg.response.coords[2].y, 5);
    EXPECT_EQ(msg.response.coords[3].x, -1);
    EXPECT_EQ(msg.response.coords[3].y, 5);
    EXPECT_EQ(msg.response.coords[4].x, 0);
    EXPECT_EQ(msg.response.coords[4].y, 3);

    // test points inside rois
    msg.request.point.x = 2;
    msg.request.point.y = -2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 2);
    EXPECT_EQ(msg.response.closest_point.y, -3);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -3);
    EXPECT_EQ(msg.response.closest_line[1].x, 3);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, 1);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 1);
    EXPECT_EQ(msg.response.coords[0].y, -3);
    EXPECT_EQ(msg.response.coords[1].x, 1);
    EXPECT_EQ(msg.response.coords[1].y, -1);
    EXPECT_EQ(msg.response.coords[2].x, 3);
    EXPECT_EQ(msg.response.coords[2].y, -3);
    EXPECT_EQ(msg.response.coords[3].x, 3);
    EXPECT_EQ(msg.response.coords[3].y, -1);
    msg.request.point.x = 3.5;
    msg.request.point.y = 0.5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 3.5);
    EXPECT_EQ(msg.response.closest_point.y, 1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 5);
    EXPECT_EQ(msg.response.closest_line[0].y, 1);
    EXPECT_EQ(msg.response.closest_line[1].x, 3);
    EXPECT_EQ(msg.response.closest_line[1].y, 1);
    EXPECT_FLOAT_EQ(msg.response.distance, 0.5);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 3);
    EXPECT_EQ(msg.response.coords[0].y, -1);
    EXPECT_EQ(msg.response.coords[1].x, 3);
    EXPECT_EQ(msg.response.coords[1].y, 1);
    EXPECT_EQ(msg.response.coords[2].x, 5);
    EXPECT_EQ(msg.response.coords[2].y, -1);
    EXPECT_EQ(msg.response.coords[3].x, 5);
    EXPECT_EQ(msg.response.coords[3].y, 1);

    // test points at roi boundaries
    msg.request.point.x = 1;
    msg.request.point.y = -2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -2);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, 0);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 1);
    EXPECT_EQ(msg.response.coords[0].y, -3);
    EXPECT_EQ(msg.response.coords[1].x, 1);
    EXPECT_EQ(msg.response.coords[1].y, -1);
    EXPECT_EQ(msg.response.coords[2].x, 3);
    EXPECT_EQ(msg.response.coords[2].y, -3);
    EXPECT_EQ(msg.response.coords[3].x, 3);
    EXPECT_EQ(msg.response.coords[3].y, -1);
    msg.request.point.x = 0;
    msg.request.point.y = 3;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 0);
    EXPECT_EQ(msg.response.closest_point.y, 3);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, -2);
    EXPECT_EQ(msg.response.closest_line[0].y, 2);
    EXPECT_EQ(msg.response.closest_line[1].x, 0);
    EXPECT_EQ(msg.response.closest_line[1].y, 3);
    EXPECT_FLOAT_EQ(msg.response.distance, 0);
    ASSERT_EQ(msg.response.coords.size(), 5);
    EXPECT_EQ(msg.response.coords[0].x, -3);
    EXPECT_EQ(msg.response.coords[0].y, 4);
    EXPECT_EQ(msg.response.coords[1].x, -2);
    EXPECT_EQ(msg.response.coords[1].y, 2);
    EXPECT_EQ(msg.response.coords[2].x, -2);
    EXPECT_EQ(msg.response.coords[2].y, 5);
    EXPECT_EQ(msg.response.coords[3].x, -1);
    EXPECT_EQ(msg.response.coords[3].y, 5);
    EXPECT_EQ(msg.response.coords[4].x, 0);
    EXPECT_EQ(msg.response.coords[4].y, 3);

    // test points outside rois
    msg.request.point.x = -1;
    msg.request.point.y = -2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -2);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, 2);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 1);
    EXPECT_EQ(msg.response.coords[0].y, -3);
    EXPECT_EQ(msg.response.coords[1].x, 1);
    EXPECT_EQ(msg.response.coords[1].y, -1);
    EXPECT_EQ(msg.response.coords[2].x, 3);
    EXPECT_EQ(msg.response.coords[2].y, -3);
    EXPECT_EQ(msg.response.coords[3].x, 3);
    EXPECT_EQ(msg.response.coords[3].y, -1);
    msg.request.point.x = -99;
    msg.request.point.y = 4;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, -7.07106781186548);
    EXPECT_EQ(msg.response.closest_point.y, 17.0710678118655);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 0);
    EXPECT_EQ(msg.response.closest_line[0].y, 24.1421356237309);
    EXPECT_EQ(msg.response.closest_line[1].x, -7.07106781186548);
    EXPECT_EQ(msg.response.closest_line[1].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.distance, 92.8535480581815);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, -7.07106781186548);
    EXPECT_EQ(msg.response.coords[0].y, 17.0710678118655);
    EXPECT_EQ(msg.response.coords[1].x, 0);
    EXPECT_EQ(msg.response.coords[1].y, 10);
    EXPECT_EQ(msg.response.coords[2].x, 0);
    EXPECT_EQ(msg.response.coords[2].y, 24.1421356237309);
    EXPECT_EQ(msg.response.coords[3].x, 7.07106781186548);
    EXPECT_EQ(msg.response.coords[3].y, 17.0710678118655);

    // test points at equal distance between rois
    msg.request.point.x = 3;
    msg.request.point.y = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 3);
    EXPECT_EQ(msg.response.closest_point.y, -1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 3);
    EXPECT_EQ(msg.response.closest_line[0].y, -3);
    EXPECT_EQ(msg.response.closest_line[1].x, 3);
    EXPECT_EQ(msg.response.closest_line[1].y, -1);
    EXPECT_FLOAT_EQ(msg.response.distance, 0);
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 1);
    EXPECT_EQ(msg.response.coords[0].y, -3);
    EXPECT_EQ(msg.response.coords[1].x, 1);
    EXPECT_EQ(msg.response.coords[1].y, -1);
    EXPECT_EQ(msg.response.coords[2].x, 3);
    EXPECT_EQ(msg.response.coords[2].y, -3);
    EXPECT_EQ(msg.response.coords[3].x, 3);
    EXPECT_EQ(msg.response.coords[3].y, -1);
    msg.request.point.x = -1;
    msg.request.point.y = 0;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, -2);
    EXPECT_EQ(msg.response.closest_point.y, 2);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, -2);
    EXPECT_EQ(msg.response.closest_line[0].y, 2);
    EXPECT_EQ(msg.response.closest_line[1].x, 0);
    EXPECT_EQ(msg.response.closest_line[1].y, 3);
    EXPECT_FLOAT_EQ(msg.response.distance, hypot(1,2));
    ASSERT_EQ(msg.response.coords.size(), 5);
    EXPECT_EQ(msg.response.coords[0].x, -3);
    EXPECT_EQ(msg.response.coords[0].y, 4);
    EXPECT_EQ(msg.response.coords[1].x, -2);
    EXPECT_EQ(msg.response.coords[1].y, 2);
    EXPECT_EQ(msg.response.coords[2].x, -2);
    EXPECT_EQ(msg.response.coords[2].y, 5);
    EXPECT_EQ(msg.response.coords[3].x, -1);
    EXPECT_EQ(msg.response.coords[3].y, 5);
    EXPECT_EQ(msg.response.coords[4].x, 0);
    EXPECT_EQ(msg.response.coords[4].y, 3);
    msg.request.point.x = 101;
    msg.request.point.y = -99;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 3);
    EXPECT_EQ(msg.response.closest_point.y, -3);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -3);
    EXPECT_EQ(msg.response.closest_line[1].x, 3);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, hypot(98,96));
    ASSERT_EQ(msg.response.coords.size(), 4);
    EXPECT_EQ(msg.response.coords[0].x, 1);
    EXPECT_EQ(msg.response.coords[0].y, -3);
    EXPECT_EQ(msg.response.coords[1].x, 1);
    EXPECT_EQ(msg.response.coords[1].y, -1);
    EXPECT_EQ(msg.response.coords[2].x, 3);
    EXPECT_EQ(msg.response.coords[2].y, -3);
    EXPECT_EQ(msg.response.coords[3].x, 3);
    EXPECT_EQ(msg.response.coords[3].y, -1);
}

/**
 * @brief Test the roi get distance service.
 */
TEST (NodeTestRoi, testGetDistance)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetDist>("rois/get_distance");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // select roi 0
    cpswarm_msgs::GetDist msg;
    geometry_msgs::Point coord;
    coord.x = 3;
    coord.y = -1;
    coord.z = 50;
    msg.request.coords.push_back(coord);
    coord.x = 1;
    coord.y = -1;
    msg.request.coords.push_back(coord);
    coord.x = 1;
    coord.y = -3;
    msg.request.coords.push_back(coord);
    coord.x = 3;
    coord.y = -3;
    msg.request.coords.push_back(coord);

    // test point inside
    msg.request.point.x = 1.5;
    msg.request.point.y = -1.5;
    msg.request.point.z = 50;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1.5);
    EXPECT_EQ(msg.response.closest_point.y, -1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 3);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -1);
    EXPECT_FLOAT_EQ(msg.response.distance, 0.5);

    // test points on boundary
    msg.request.point.x = 1.5;
    msg.request.point.y = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1.5);
    EXPECT_EQ(msg.response.closest_point.y, -1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 3);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -1);
    EXPECT_FLOAT_EQ(msg.response.distance, 0);
    msg.request.point.x = 1;
    msg.request.point.y = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 3);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -1);
    EXPECT_FLOAT_EQ(msg.response.distance, 0);

    // test points outside
    msg.request.point.x = 0;
    msg.request.point.y = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -1);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 3);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -1);
    EXPECT_FLOAT_EQ(msg.response.distance, 1);
    msg.request.point.x = 0;
    msg.request.point.y = -2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -2);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -1);
    EXPECT_EQ(msg.response.closest_line[1].x, 1);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, 1);
    msg.request.point.x = 0;
    msg.request.point.y = -4;
    ASSERT_TRUE(client.call(msg));
    EXPECT_EQ(msg.response.closest_point.x, 1);
    EXPECT_EQ(msg.response.closest_point.y, -3);
    ASSERT_EQ(msg.response.closest_line.size(), 2);
    EXPECT_EQ(msg.response.closest_line[0].x, 1);
    EXPECT_EQ(msg.response.closest_line[0].y, -3);
    EXPECT_EQ(msg.response.closest_line[1].x, 3);
    EXPECT_EQ(msg.response.closest_line[1].y, -3);
    EXPECT_FLOAT_EQ(msg.response.distance, hypot(1,1));
}

/**
 * @brief Test the roi get map service.
 */
TEST (NodeTestRoi, testGetMap)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetMap>("rois/get_map");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds

    // select roi 1
    cpswarm_msgs::GetMap msg;
    geometry_msgs::Point coord;
    coord.x = 5;
    coord.y = 1;
    coord.z = 50;
    msg.request.coords.push_back(coord);
    coord.x = 3;
    coord.y = 1;
    msg.request.coords.push_back(coord);
    coord.x = 3;
    coord.y = -1;
    msg.request.coords.push_back(coord);
    coord.x = 5;
    coord.y = -1;
    msg.request.coords.push_back(coord);

    // test original
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 2);
    EXPECT_EQ(msg.response.map.info.height, 2);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 4);
    for (int i=0; i<msg.response.map.data.size(); ++i)
        EXPECT_EQ(msg.response.map.data[i], 0);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // change rotation and translation, should not change anything
    msg.request.rotate = true;
    msg.request.translate = true;
    msg.request.resolution = -1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 2);
    EXPECT_EQ(msg.response.map.info.height, 2);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 4);
    for (int i=0; i<msg.response.map.data.size(); ++i)
        EXPECT_EQ(msg.response.map.data[i], 0);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // increase resolution
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 0.5;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 0.5);
    EXPECT_EQ(msg.response.map.info.width, 4);
    EXPECT_EQ(msg.response.map.info.height, 4);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 16);
    for (int i=0; i<msg.response.map.data.size(); ++i)
        EXPECT_EQ(msg.response.map.data[i], 0);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // decrease resolution
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 2;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 2);
    EXPECT_EQ(msg.response.map.info.width, 1);
    EXPECT_EQ(msg.response.map.info.height, 1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 1);
    EXPECT_EQ(msg.response.map.data[0], 0);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // decrease resolution too much
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 10;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 2);
    EXPECT_EQ(msg.response.map.info.width, 1);
    EXPECT_EQ(msg.response.map.info.height, 1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -1);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 1);
    EXPECT_EQ(msg.response.map.data[0], 0);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // select roi 2
    msg.request.coords.clear();
    coord.x = 0;
    coord.y = 3;
    msg.request.coords.push_back(coord);
    coord.x = -2;
    coord.y = 2;
    msg.request.coords.push_back(coord);
    coord.x = -2;
    coord.y = 5;
    msg.request.coords.push_back(coord);
    coord.x = -1;
    coord.y = 5;
    msg.request.coords.push_back(coord);
    coord.x = -3;
    coord.y = 4;
    msg.request.coords.push_back(coord);

    // test original
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 3);
    EXPECT_EQ(msg.response.map.info.height, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, -3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, 2);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 9);
    EXPECT_EQ(msg.response.map.data[0], 100);
    EXPECT_EQ(msg.response.map.data[1], 0);
    EXPECT_EQ(msg.response.map.data[2], 100);
    EXPECT_EQ(msg.response.map.data[3], 0);
    EXPECT_EQ(msg.response.map.data[4], 0);
    EXPECT_EQ(msg.response.map.data[5], 0);
    EXPECT_EQ(msg.response.map.data[6], 0);
    EXPECT_EQ(msg.response.map.data[7], 0);
    EXPECT_EQ(msg.response.map.data[8], 100);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // test rotated
    double rot = -atan(0.5);
    msg.request.rotate = true;
    msg.request.translate = false;
    msg.request.resolution = 1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 3);
    EXPECT_EQ(msg.response.map.info.height, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, -2.0*cos(rot)-2*sin(rot));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, -2*sin(rot)+2*cos(rot));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 9);
    EXPECT_EQ(msg.response.map.data[0], 0);
    EXPECT_EQ(msg.response.map.data[1], 0);
    EXPECT_EQ(msg.response.map.data[2], 100);
    EXPECT_EQ(msg.response.map.data[3], 0);
    EXPECT_EQ(msg.response.map.data[4], 0);
    EXPECT_EQ(msg.response.map.data[5], 100);
    EXPECT_EQ(msg.response.map.data[6], 100);
    EXPECT_EQ(msg.response.map.data[7], 0);
    EXPECT_EQ(msg.response.map.data[8], 100);
    EXPECT_FLOAT_EQ(msg.response.rotation, rot);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // test rotated and translated
    rot = -atan(0.5);
    msg.request.rotate = true;
    msg.request.translate = true;
    msg.request.resolution = 1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 3);
    EXPECT_EQ(msg.response.map.info.height, 3);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, round(-2.0*cos(rot)-2*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, round(-2*sin(rot)+2*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 9);
    EXPECT_EQ(msg.response.map.data[0], 0);
    EXPECT_EQ(msg.response.map.data[1], 0);
    EXPECT_EQ(msg.response.map.data[2], 100);
    EXPECT_EQ(msg.response.map.data[3], 0);
    EXPECT_EQ(msg.response.map.data[4], 0);
    EXPECT_EQ(msg.response.map.data[5], 100);
    EXPECT_EQ(msg.response.map.data[6], 100);
    EXPECT_EQ(msg.response.map.data[7], 0);
    EXPECT_EQ(msg.response.map.data[8], 100);
    EXPECT_FLOAT_EQ(msg.response.rotation, rot);
    EXPECT_FLOAT_EQ(msg.response.translation.x, round(-2.0*cos(rot)-2*sin(rot)) - (-2.0*cos(rot)-2*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.y, round(-2*sin(rot)+2*cos(rot)) - (-2*sin(rot)+2*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // test rotated and translated at higher resolution
    rot = -atan(0.5);
    msg.request.rotate = true;
    msg.request.translate = true;
    msg.request.resolution = 0.25;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 0.25);
    EXPECT_EQ(msg.response.map.info.width, 9);
    EXPECT_EQ(msg.response.map.info.height, 11);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, round(-2.0*cos(rot)-2*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, round(-2*sin(rot)+2*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 99);
    for (int i=0; i<9; ++i) {
        for (int j=0; j<9; ++j) {
            EXPECT_EQ(msg.response.map.data[i], 0);
        }
    }
    EXPECT_EQ(msg.response.map.data[81], 100);
    EXPECT_EQ(msg.response.map.data[82], 100);
    EXPECT_EQ(msg.response.map.data[83], 0);
    EXPECT_EQ(msg.response.map.data[84], 0);
    EXPECT_EQ(msg.response.map.data[85], 0);
    EXPECT_EQ(msg.response.map.data[86], 0);
    EXPECT_EQ(msg.response.map.data[87], 0);
    EXPECT_EQ(msg.response.map.data[88], 0);
    EXPECT_EQ(msg.response.map.data[89], 100);
    EXPECT_EQ(msg.response.map.data[90], 100);
    EXPECT_EQ(msg.response.map.data[91], 100);
    EXPECT_EQ(msg.response.map.data[92], 100);
    EXPECT_EQ(msg.response.map.data[93], 100);
    EXPECT_EQ(msg.response.map.data[94], 100);
    EXPECT_EQ(msg.response.map.data[95], 0);
    EXPECT_EQ(msg.response.map.data[96], 100);
    EXPECT_EQ(msg.response.map.data[97], 100);
    EXPECT_EQ(msg.response.map.data[98], 100);
    EXPECT_FLOAT_EQ(msg.response.rotation, rot);
    EXPECT_FLOAT_EQ(msg.response.translation.x, round(-2.0*cos(rot)-2*sin(rot)) - (-2.0*cos(rot)-2*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.y, round(-2*sin(rot)+2*cos(rot)) - (-2*sin(rot)+2*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // select roi 4
    msg.request.coords.clear();
    coord.x = 0;
    coord.y = 10;
    msg.request.coords.push_back(coord);
    coord.x = 0;
    coord.y = 24.1421356237309;
    msg.request.coords.push_back(coord);
    coord.x = -7.07106781186548;
    coord.y = 17.0710678118655;
    msg.request.coords.push_back(coord);
    coord.x = 7.07106781186548;
    coord.y = 17.0710678118655;
    msg.request.coords.push_back(coord);

    // test original (only meta data)
    msg.request.rotate = false;
    msg.request.translate = false;
    msg.request.resolution = 1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 15);
    EXPECT_EQ(msg.response.map.info.height, 15);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, -7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, 10);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 225);
    EXPECT_FLOAT_EQ(msg.response.rotation, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.x, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.y, 0);
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);

    // test rotated and translated
    rot = M_PI / 4.0;
    msg.request.rotate = true;
    msg.request.translate = true;
    msg.request.resolution = 1;
    ASSERT_TRUE(client.call(msg));
    EXPECT_FLOAT_EQ(msg.response.map.info.resolution, 1);
    EXPECT_EQ(msg.response.map.info.width, 11);
    EXPECT_EQ(msg.response.map.info.height, 11);
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.x, round(-7.07106781186548*cos(rot)-17.0710678118655*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.y, round(-7.07106781186548*sin(rot)+17.0710678118655*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.map.info.origin.position.z, 0);
    ASSERT_EQ(msg.response.map.data.size(), 121);
    for (int i=0; i<10; ++i) {
        for (int j=0; j<10; ++j) {
            EXPECT_EQ(msg.response.map.data[i*11 + j], 0);
        }
    }
    for (int i=0; i<11; ++i) {
        EXPECT_EQ(msg.response.map.data[i*11 + 10], 100);
    }
    for (int j=0; j<11; ++j) {
        EXPECT_EQ(msg.response.map.data[10*11 + j], 100);
    }
    EXPECT_FLOAT_EQ(msg.response.rotation, rot);
    EXPECT_FLOAT_EQ(msg.response.translation.x, round(-7.07106781186548*cos(rot)-10*sin(rot)) - (-7.07106781186548*cos(rot)-10*sin(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.y, round(-7.07106781186548*sin(rot)+10*cos(rot)) - (-7.07106781186548*sin(rot)+10*cos(rot)));
    EXPECT_FLOAT_EQ(msg.response.translation.z, 0);
}

/**
 * @brief Test the roi get todo service.
 */
TEST (NodeTestRoi, testGetTodo)
{
    // create service client to get rois
    NodeHandle nh;
    ServiceClient get_client = nh.serviceClient<cpswarm_msgs::GetMultiPoints>("rois/get_todo");
    ASSERT_TRUE(get_client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetMultiPoints get_msg;

    // create service client to change roi state
    ServiceClient set_client = nh.serviceClient<cpswarm_msgs::SetRoiState>("rois/set_state");
    ASSERT_TRUE(set_client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::SetRoiState set_msg;

    // set one roi to in progress
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point p;
    p.x = -3;
    p.y = 4;
    p.z = 50;
    coords.push_back(p);
    p.x = -2;
    p.y = 2;
    coords.push_back(p);
    p.x = -2;
    p.y = 5;
    coords.push_back(p);
    p.x = -1;
    p.y = 5;
    coords.push_back(p);
    p.x = 0;
    p.y = 3;
    coords.push_back(p);
    set_msg.request.coords = coords;
    set_msg.request.state = 1;
    ASSERT_TRUE(set_client.call(set_msg));
    EXPECT_TRUE(set_msg.response.success);

    // set one roi to done
    coords.clear();
    p.x = 1;
    p.y = -3;
    coords.push_back(p);
    p.x = 1;
    p.y = -1;
    coords.push_back(p);
    p.x = 3;
    p.y = -3;
    coords.push_back(p);
    p.x = 3;
    p.y = -1;
    coords.push_back(p);
    set_msg.request.coords = coords;
    set_msg.request.state = 2;
    ASSERT_TRUE(set_client.call(set_msg));
    EXPECT_TRUE(set_msg.response.success);

    // get only todo rois
    ASSERT_TRUE(get_client.call(get_msg));
    ASSERT_EQ(get_msg.response.layout.dim.size(), 2);
    EXPECT_EQ(get_msg.response.layout.dim[0].size, 2);
    EXPECT_EQ(get_msg.response.layout.dim[0].stride, 8);
    EXPECT_EQ(get_msg.response.layout.dim[1].size, 4);
    EXPECT_EQ(get_msg.response.layout.dim[1].stride, 4);
    EXPECT_EQ(get_msg.response.layout.data_offset, 0);
    ASSERT_EQ(get_msg.response.points.size(), 8);

    EXPECT_FLOAT_EQ(get_msg.response.points[0].x, -7.07106781186548);
    EXPECT_FLOAT_EQ(get_msg.response.points[0].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(get_msg.response.points[0].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[1].x, 0);
    EXPECT_FLOAT_EQ(get_msg.response.points[1].y, 10);
    EXPECT_FLOAT_EQ(get_msg.response.points[1].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[2].x, 0);
    EXPECT_FLOAT_EQ(get_msg.response.points[2].y, 24.1421356237309);
    EXPECT_FLOAT_EQ(get_msg.response.points[2].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[3].x, 7.07106781186548);
    EXPECT_FLOAT_EQ(get_msg.response.points[3].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(get_msg.response.points[3].z, 50);

    EXPECT_FLOAT_EQ(get_msg.response.points[4].x, 3);
    EXPECT_FLOAT_EQ(get_msg.response.points[4].y, -1);
    EXPECT_FLOAT_EQ(get_msg.response.points[4].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[5].x, 3);
    EXPECT_FLOAT_EQ(get_msg.response.points[5].y, 1);
    EXPECT_FLOAT_EQ(get_msg.response.points[5].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[6].x, 5);
    EXPECT_FLOAT_EQ(get_msg.response.points[6].y, -1);
    EXPECT_FLOAT_EQ(get_msg.response.points[6].z, 50);
    EXPECT_FLOAT_EQ(get_msg.response.points[7].x, 5);
    EXPECT_FLOAT_EQ(get_msg.response.points[7].y, 1);
    EXPECT_FLOAT_EQ(get_msg.response.points[7].z, 50);
}

/**
 * @brief Test the roi reload service.
 */
TEST (NodeTestRoi, testReload)
{
    // create service client
    NodeHandle nh;
    ServiceClient reload_client = nh.serviceClient<std_srvs::SetBool>("rois/reload");
    ASSERT_TRUE(reload_client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    std_srvs::SetBool reload_msg;

    // reload without clearing
    reload_msg.request.data = false;
    ASSERT_TRUE(reload_client.call(reload_msg));
    EXPECT_TRUE(reload_msg.response.success);

    // retrieve rois
    ServiceClient client = nh.serviceClient<cpswarm_msgs::GetMultiPoints>("rois/get_all");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::GetMultiPoints msg;
    ASSERT_TRUE(client.call(msg));
    ASSERT_EQ(msg.response.layout.dim.size(), 2);
    EXPECT_EQ(msg.response.layout.dim[0].size, 4);
    EXPECT_EQ(msg.response.layout.dim[0].stride, 20);
    EXPECT_EQ(msg.response.layout.dim[1].size, 5);
    EXPECT_EQ(msg.response.layout.dim[1].stride, 5);
    EXPECT_EQ(msg.response.layout.data_offset, 0);
    ASSERT_EQ(msg.response.points.size(), 20);

    EXPECT_FLOAT_EQ(msg.response.points[0].x, -7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[0].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[0].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[1].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[1].y, 10);
    EXPECT_FLOAT_EQ(msg.response.points[1].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[2].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[2].y, 24.1421356237309);
    EXPECT_FLOAT_EQ(msg.response.points[2].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[3].x, 7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[3].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[3].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[4].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[5].x, -3);
    EXPECT_FLOAT_EQ(msg.response.points[5].y, 4);
    EXPECT_FLOAT_EQ(msg.response.points[5].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[6].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[6].y, 2);
    EXPECT_FLOAT_EQ(msg.response.points[6].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[7].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[7].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[7].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[8].x, -1);
    EXPECT_FLOAT_EQ(msg.response.points[8].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[8].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[9].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[9].y, 3);
    EXPECT_FLOAT_EQ(msg.response.points[9].z, 50);

    EXPECT_FLOAT_EQ(msg.response.points[10].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[10].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[10].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[11].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[11].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[11].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[12].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[12].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[12].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[13].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[13].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[13].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[14].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[15].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[15].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[15].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[16].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[16].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[16].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[17].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[17].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[17].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[18].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[18].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[18].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[19].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].z, 0);

    // reload with clearing
    reload_msg.request.data = true;
    ASSERT_TRUE(reload_client.call(reload_msg));
    EXPECT_TRUE(reload_msg.response.success);

    // retrieve rois
    ASSERT_TRUE(client.call(msg));
    ASSERT_EQ(msg.response.layout.dim.size(), 2);
    EXPECT_EQ(msg.response.layout.dim[0].size, 4);
    EXPECT_EQ(msg.response.layout.dim[0].stride, 20);
    EXPECT_EQ(msg.response.layout.dim[1].size, 5);
    EXPECT_EQ(msg.response.layout.dim[1].stride, 5);
    EXPECT_EQ(msg.response.layout.data_offset, 0);
    ASSERT_EQ(msg.response.points.size(), 20);

    EXPECT_FLOAT_EQ(msg.response.points[0].x, -7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[0].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[0].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[1].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[1].y, 10);
    EXPECT_FLOAT_EQ(msg.response.points[1].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[2].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[2].y, 24.1421356237309);
    EXPECT_FLOAT_EQ(msg.response.points[2].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[3].x, 7.07106781186548);
    EXPECT_FLOAT_EQ(msg.response.points[3].y, 17.0710678118655);
    EXPECT_FLOAT_EQ(msg.response.points[3].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[4].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[4].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[5].x, -3);
    EXPECT_FLOAT_EQ(msg.response.points[5].y, 4);
    EXPECT_FLOAT_EQ(msg.response.points[5].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[6].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[6].y, 2);
    EXPECT_FLOAT_EQ(msg.response.points[6].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[7].x, -2);
    EXPECT_FLOAT_EQ(msg.response.points[7].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[7].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[8].x, -1);
    EXPECT_FLOAT_EQ(msg.response.points[8].y, 5);
    EXPECT_FLOAT_EQ(msg.response.points[8].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[9].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[9].y, 3);
    EXPECT_FLOAT_EQ(msg.response.points[9].z, 50);

    EXPECT_FLOAT_EQ(msg.response.points[10].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[10].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[10].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[11].x, 1);
    EXPECT_FLOAT_EQ(msg.response.points[11].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[11].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[12].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[12].y, -3);
    EXPECT_FLOAT_EQ(msg.response.points[12].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[13].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[13].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[13].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[14].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[14].z, 0);

    EXPECT_FLOAT_EQ(msg.response.points[15].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[15].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[15].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[16].x, 3);
    EXPECT_FLOAT_EQ(msg.response.points[16].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[16].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[17].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[17].y, -1);
    EXPECT_FLOAT_EQ(msg.response.points[17].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[18].x, 5);
    EXPECT_FLOAT_EQ(msg.response.points[18].y, 1);
    EXPECT_FLOAT_EQ(msg.response.points[18].z, 50);
    EXPECT_FLOAT_EQ(msg.response.points[19].x, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].y, 0);
    EXPECT_FLOAT_EQ(msg.response.points[19].z, 0);
}

/**
 * @brief Test the roi set state service.
 */
TEST (NodeTestRoi, testSetState)
{
    // create service client
    NodeHandle nh;
    ServiceClient client = nh.serviceClient<cpswarm_msgs::SetRoiState>("rois/set_state");
    ASSERT_TRUE(client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    cpswarm_msgs::SetRoiState msg;

    // empty request
    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.success);

    // invalid roi and state
    vector<geometry_msgs::Point> coords;
    geometry_msgs::Point p;
    p.x = 1;
    p.y = 2;
    p.z = 0;
    coords.push_back(p);
    msg.request.coords = coords;
    msg.request.state = -2;

    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.success);

    // invalid roi only
    msg.request.state = 1;

    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.success);

    // invalid state only
    coords.clear();
    p.x = -3;
    p.y = 4;
    p.z = 50;
    coords.push_back(p);
    p.x = -2;
    p.y = 2;
    coords.push_back(p);
    p.x = -2;
    p.y = 5;
    coords.push_back(p);
    p.x = -1;
    p.y = 5;
    coords.push_back(p);
    p.x = 0;
    p.y = 3;
    coords.push_back(p);
    msg.request.coords = coords;
    msg.request.state = 10;

    ASSERT_TRUE(client.call(msg));
    EXPECT_FALSE(msg.response.success);

    // valid roi and state
    msg.request.state = 2;

    ASSERT_TRUE(client.call(msg));
    EXPECT_TRUE(msg.response.success);
}

/**
 * @brief Test the roi publisher.
 */
TEST (NodeTestRoi, testPublish)
{
    NodeHandle nh;

    // create subscriber for ROI event messages
    Subscriber roi_subscriber = nh.subscribe("rois/roi", 10, roi_callback);
    Duration(1).sleep(); // give subscriber some time to connect

    // create service client to reload ROIs from files
    ServiceClient reload_client = nh.serviceClient<std_srvs::SetBool>("rois/reload");
    ASSERT_TRUE(reload_client.waitForExistence(Duration(5.0))); // failure, if server does not respond within 5 seconds
    std_srvs::SetBool reload_msg;

    // reload rois, clearing existing ones
    reload_msg.request.data = true;
    ASSERT_TRUE(reload_client.call(reload_msg));
    EXPECT_TRUE(reload_msg.response.success);

    // execute subscriber callback
    spinOnce();

    // make sure all four rois are received
    ASSERT_EQ(rois.size(), 4);
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    init(argc, argv, "test_area");
    return RUN_ALL_TESTS();
}
