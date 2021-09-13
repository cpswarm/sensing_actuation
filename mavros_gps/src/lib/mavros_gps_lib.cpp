#include "lib/mavros_gps_lib.h"

mavros_gps_lib::mavros_gps_lib ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init origin
    pose_sub = nh.subscribe("mavros/global_position/global", queue_size, &mavros_gps_lib::pose_callback, this);
    while (ok() && origin.latitude == 0) {
        ROS_DEBUG_THROTTLE(10, "Waiting for valid GPS...");
        rate.sleep();
        spinOnce();
    }
}

bool mavros_gps_lib::fix_to_pose (cpswarm_msgs::FixToPose::Request &req, cpswarm_msgs::FixToPose::Response &res)
{
    // copy header
    res.pose.header = req.fix.header;

    // compute local coordinates
    double dist = this->dist(origin, req.fix);
    double head = ned_to_enu(yaw(origin, req.fix));
    res.pose.pose.position.x = dist * cos(head);
    res.pose.pose.position.y = dist * sin(head);
    res.pose.pose.position.z = req.fix.altitude;

    // compute orientation
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, head);
    res.pose.pose.orientation = tf2::toMsg(orientation);

    return true;
}

bool mavros_gps_lib::fix_to_target (mavros_gps::FixToTarget::Request &req, mavros_gps::FixToTarget::Response &res)
{
    res.target = fix_to_target(req.fix, req.yaw);
    return true;
}

bool mavros_gps_lib::get_gps_origin (cpswarm_msgs::GetGpsFix::Request &req, cpswarm_msgs::GetGpsFix::Response &res)
{
    res.fix = origin;
    return true;
}

bool mavros_gps_lib::ned_to_enu (cpswarm_msgs::NedToEnu::Request &req, cpswarm_msgs::NedToEnu::Response &res)
{
    res.yaw = ned_to_enu(req.yaw);
    return true;
}

bool mavros_gps_lib::pose_to_fix (cpswarm_msgs::PoseToFix::Request &req, cpswarm_msgs::PoseToFix::Response &res)
{
    // compute gps coordinates
    double dist = hypot(req.pose.pose.position.x, req.pose.pose.position.y);
    double head = atan2(req.pose.pose.position.y, req.pose.pose.position.x);
    res.fix = goal(origin, dist, head);

    // copy header
    res.fix.header = req.pose.header;

    // copy altitude
    res.fix.altitude = req.pose.pose.position.z;

    return true;
}

bool mavros_gps_lib::pose_to_target (mavros_gps::PoseToTarget::Request &req, mavros_gps::PoseToTarget::Response &res)
{
    // compute gps coordinates
    double dist = hypot(req.pose.pose.position.x, req.pose.pose.position.y);
    double head = atan2(req.pose.pose.position.y, req.pose.pose.position.x);
    tf2::Quaternion orientation;
    tf2::fromMsg(req.pose.pose.orientation, orientation);
    double yaw = tf2::getYaw(orientation);
    res.target = fix_to_target(goal(origin, dist, head), yaw);

    // copy header
    res.target.header = req.pose.header;

    // copy altitude
    res.target.altitude = req.pose.pose.position.z;

    return true;
}

bool mavros_gps_lib::target_to_fix (mavros_gps::TargetToFix::Request &req, mavros_gps::TargetToFix::Response &res)
{
    res.fix = target_to_fix(req.target);
    return true;
}

double mavros_gps_lib::dist (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
{
    // convert to radian
    double lat1 = start.latitude / 180.0 * M_PI;
    double lon1 = start.longitude / 180.0 * M_PI;
    double lat2 = goal.latitude / 180.0 * M_PI;
    double lon2 = goal.longitude / 180.0 * M_PI;

    // compute great circle distance
    return 2 * R * asin(sqrt(pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2)));
}

double mavros_gps_lib::dist (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const
{
    return dist(start, target_to_fix(goal));
}

double mavros_gps_lib::dist (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const
{
    return dist(target_to_fix(start), goal);
}

double mavros_gps_lib::dist (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const
{
    return dist(target_to_fix(start), target_to_fix(goal));
}

mavros_msgs::GlobalPositionTarget mavros_gps_lib::fix_to_target (sensor_msgs::NavSatFix fix, double yaw) const
{
    // create target message
    mavros_msgs::GlobalPositionTarget target;
    target.header = fix.header;

    // copy coordinates
    target.latitude = fix.latitude;
    target.longitude = fix.longitude;
    target.altitude = fix.altitude;

    // set orientation for nose in forward movement direction
    target.yaw = yaw;

    return target;
}

sensor_msgs::NavSatFix mavros_gps_lib::goal (sensor_msgs::NavSatFix start, double distance, double yaw) const
{

    // convert to radian
    double lat1 = start.latitude / 180.0 * M_PI;
    double lon1 = start.longitude / 180.0 * M_PI;

    // compute goal
    double lat2 = asin(sin(lat1) * cos(distance / R) + cos(lat1) * sin(distance / R) * cos(ned_to_enu(yaw)));
    double lon2 = remainder(lon1 + atan2(sin(ned_to_enu(yaw)) * sin(distance / R) * cos(lat1), cos(distance / R) - sin(lat1) * sin(lat2)), 2*M_PI);

    // goal coordinates
    sensor_msgs::NavSatFix goal;
    goal.latitude = lat2 / M_PI * 180.0;
    goal.longitude = lon2 / M_PI * 180.0;
    goal.altitude = start.altitude; // keep altitude

    return goal;
}

sensor_msgs::NavSatFix mavros_gps_lib::goal (mavros_msgs::GlobalPositionTarget start, double distance, double yaw) const
{
    return goal(target_to_fix(start), distance, yaw);
}

double mavros_gps_lib::ned_to_enu (double yaw) const
{
    return remainder(2 * M_PI - yaw + M_PI / 2, 2*M_PI);
}

sensor_msgs::NavSatFix mavros_gps_lib::target_to_fix (mavros_msgs::GlobalPositionTarget target) const
{
    // create message and copy header
    sensor_msgs::NavSatFix fix;
    fix.header = target.header;

    // copy coordinates
    fix.latitude = target.latitude;
    fix.longitude = target.longitude;
    fix.altitude = target.altitude;

    return fix;
}

double mavros_gps_lib::yaw (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
{
    // convert to radian
    double lat1 = start.latitude / 180.0 * M_PI;
    double lon1 = start.longitude / 180.0 * M_PI;
    double lat2 = goal.latitude / 180.0 * M_PI;
    double lon2 = goal.longitude / 180.0 * M_PI;

    // compute initial yaw
    double dx = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
    double dy = sin(lon2 - lon1) * cos(lat2);
    return atan2(dy, dx) + M_PI;
}

double mavros_gps_lib::yaw (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const
{
    return yaw(start, target_to_fix(goal));
}

double mavros_gps_lib::yaw (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const
{
    return yaw(target_to_fix(start), goal);
}

double mavros_gps_lib::yaw (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const
{
    return yaw(target_to_fix(start), target_to_fix(goal));
}

void mavros_gps_lib::pose_callback (const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // store pose as origin class variable
    origin = *msg;

	ROS_DEBUG("Got GPS coordinates");
    ROS_INFO("Set origin at [%f, %f, %.2f]", origin.latitude, origin.longitude, origin.altitude);

    // unsubscribe
    if (origin.latitude != 0)
        pose_sub.shutdown();
}
