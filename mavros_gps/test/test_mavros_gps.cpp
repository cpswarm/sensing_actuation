#include <gtest/gtest.h>
#include "mavros_gps_lib.h"

/**
 * @brief Test the gps helper library distance calculation.
 */
TEST (UnitTestHelperGps, testDist)
{
    // test the distance between two nav sat fixes
    sensor_msgs::NavSatFix test1;
    test1.latitude = 50.5;
    test1.longitude = 7.5;
    sensor_msgs::NavSatFix test2;
    test2.latitude = 50.6;
    test2.longitude = 7.6;
    EXPECT_NEAR(13189.1, mavros_gps_lib::dist(test1, test2), 0.05);

    // test the distance between nav sat fix and global position target
    sensor_msgs::NavSatFix test3;
    test3.latitude = 50.5;
    test3.longitude = 7.5;
    mavros_msgs::GlobalPositionTarget test4;
    test4.latitude = 50.5;
    test4.longitude = 7.6;
    EXPECT_NEAR(7080.8, mavros_gps_lib::dist(test3, test4), 0.05);

    // test the distance between global position target and nav sat fix
    mavros_msgs::GlobalPositionTarget test5;
    test5.latitude = 50.5;
    test5.longitude = 7.5;
    sensor_msgs::NavSatFix test6;
    test6.latitude = 50.5;
    test6.longitude = 7.5;
    EXPECT_NEAR(0.0, mavros_gps_lib::dist(test5, test6), 0.05);

    // test the distance between two global position targets
    mavros_msgs::GlobalPositionTarget test7;
    test7.latitude = 0.0;
    test7.longitude = 0.0;
    mavros_msgs::GlobalPositionTarget test8;
    test8.latitude = -45.0;
    test8.longitude = -45.0;
    EXPECT_NEAR(6679169.4, mavros_gps_lib::dist(test7, test8), 0.05);
}

/**
 * @brief Test the gps helper library goal calculation.
 */
TEST (UnitTestHelperGps, testGoal)
{
    // test the goal for nav sat fix
    sensor_msgs::NavSatFix test1;
    test1.latitude = 50.5;
    test1.longitude = 7.5;
    double test2 = 1000.0;
    double test3 = M_PI / 4.0;
    sensor_msgs::NavSatFix test4 = mavros_gps_lib::goal(test1, test2, test3);
    EXPECT_NEAR(50.5, test4.latitude, 0.05);
    EXPECT_NEAR(7.5, test4.longitude, 0.05);

    // test the goal for nav sat fix for very large distance
    sensor_msgs::NavSatFix test5;
    test5.latitude = 50.5;
    test5.longitude = 7.5;
    double test6 = 10000000.0;
    double test7 = M_PI / 4.0;
    sensor_msgs::NavSatFix test8 = mavros_gps_lib::goal(test5, test6, test7);
    EXPECT_NEAR(26.9, test8.latitude, 0.05);
    EXPECT_NEAR(135.1, test8.longitude, 0.05);

    // test the goal for global position target
    mavros_msgs::GlobalPositionTarget test9;
    test9.latitude = -25.4;
    test9.longitude = -60.3;
    double test10 = 1000000.0;
    double test11 = 3.0 * M_PI / 2.0;
    sensor_msgs::NavSatFix test12 = mavros_gps_lib::goal(test9, test10, test11);
    EXPECT_NEAR(-34.4, test12.latitude, 0.05);
    EXPECT_NEAR(-60.3, test12.longitude, 0.05);

    // test the goal for global position target with no distance
    mavros_msgs::GlobalPositionTarget test13;
    test13.latitude = -25.4;
    test13.longitude = -60.3;
    double test14 = 0.0;
    double test15 = 3.0 * M_PI / 2.0;
    sensor_msgs::NavSatFix test16 = mavros_gps_lib::goal(test13, test14, test15);
    EXPECT_NEAR(-25.4, test16.latitude, 0.05);
    EXPECT_NEAR(-60.3, test16.longitude, 0.05);
}

/**
 * @brief Test the gps helper library yaw calculation.
 */
TEST (UnitTestHelperGps, testYaw)
{
    // test the yaw for two nav sat fixes
    sensor_msgs::NavSatFix test1;
    test1.latitude = 0.0;
    test1.longitude = 0.0;
    sensor_msgs::NavSatFix test2;
    test2.latitude = 90.0;
    test2.longitude = 90.0;
    EXPECT_NEAR(0.0, mavros_gps_lib::yaw(test1, test2), 0.05);

    // test the yaw for nav sat fix and global position target
    sensor_msgs::NavSatFix test3;
    test3.latitude = 90.0;
    test3.longitude = 90.0;
    mavros_msgs::GlobalPositionTarget test4;
    test4.latitude = 0.0;
    test4.longitude = 0.0;
    EXPECT_FLOAT_EQ(3.0 * M_PI / 2.0, mavros_gps_lib::yaw(test3, test4));

    // test the yaw for global position target and nav sat fix
    mavros_msgs::GlobalPositionTarget test5;
    test5.latitude = 52.1;
    test5.longitude = 6.5;
    sensor_msgs::NavSatFix test6;
    test6.latitude = 53.2;
    test6.longitude = 5.6;
    EXPECT_NEAR(5.8, mavros_gps_lib::yaw(test5, test6), 0.05);

    // test the yaw for two global position targets
    mavros_msgs::GlobalPositionTarget test7;
    test7.latitude = 12.3;
    test7.longitude = -4.56;
    mavros_msgs::GlobalPositionTarget test8;
    test8.latitude = -45.6;
    test8.longitude = 1.23;
    EXPECT_NEAR(3.1, mavros_gps_lib::yaw(test7, test8), 0.05);
}

/**
 * @brief Test the gps helper library conversion.
 */
TEST (UnitTestHelperGps, testConversion)
{
    // test conversion from nav sat fix to global position target
    sensor_msgs::NavSatFix test1;
    test1.latitude = 1.23;
    test1.longitude = 4.56;
    test1.altitude = 7.89;
    double test2 = 0.12;
    mavros_msgs::GlobalPositionTarget test3 = mavros_gps_lib::fix_to_target(test1, test2);
    EXPECT_FLOAT_EQ(test1.latitude, test3.latitude);
    EXPECT_FLOAT_EQ(test1.longitude, test3.longitude);
    EXPECT_FLOAT_EQ(test1.altitude, test3.altitude);
    EXPECT_FLOAT_EQ(M_PI / 2.0  - test2, test3.yaw);

    // test conversion from nav sat fix to global position target
    mavros_msgs::GlobalPositionTarget test4;
    test4.latitude = 1.23;
    test4.longitude = 4.56;
    test4.altitude = 7.89;
    sensor_msgs::NavSatFix test5 = mavros_gps_lib::target_to_fix(test4);
    EXPECT_FLOAT_EQ(test4.latitude, test5.latitude);
    EXPECT_FLOAT_EQ(test4.longitude, test5.longitude);
    EXPECT_FLOAT_EQ(test4.altitude, test5.altitude);

    // test conversion of yaw from ned to enu
    double test6 = - M_PI / 2.0;
    EXPECT_FLOAT_EQ(M_PI, mavros_gps_lib::ned_to_enu(test6));

    // test conversion of yaw from enu to ned
    double test7 = 0.0;
    EXPECT_FLOAT_EQ(M_PI / 2.0, mavros_gps_lib::ned_to_enu(test7));
}

/**
 * @brief Main function that runs all tests that were declared with TEST().
 */
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
