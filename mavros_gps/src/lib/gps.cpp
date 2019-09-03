#include "lib/gps.h"

gps::gps ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init origin
    pose_sub = nh.subscribe("mavros/global_position/global", queue_size, &gps::pose_callback, this);
    while (ok() && origin.latitude == 0) {
        rate.sleep();
        spinOnce();
    }
}

bool gps::fix_to_pose (cpswarm_msgs::fix_to_pose::Request &req, cpswarm_msgs::fix_to_pose::Response &res)
{
    // copy header
    res.pose.header = req.fix.header;

    // compute local coordinates
    double dist = this->dist(origin, req.fix);
    double head = ned_to_enu(yaw(origin, req.fix));
    res.pose.pose.position.x = dist * cos(head);
    res.pose.pose.position.y = dist * sin(head);
    res.pose.pose.position.z = req.fix.altitude;

    return true;
}

bool gps::fix_to_target (mavros_gps::fix_to_target::Request &req, mavros_gps::fix_to_target::Response &res)
{
    res.target = fix_to_target(req.fix, req.yaw);
    return true;
}

bool gps::get_gps_origin (cpswarm_msgs::get_gps_origin::Request &req, cpswarm_msgs::get_gps_origin::Response &res)
{
    res.origin = origin;
    return true;
}

bool gps::ned_to_enu (cpswarm_msgs::ned_to_enu::Request &req, cpswarm_msgs::ned_to_enu::Response &res)
{
    res.yaw = ned_to_enu(req.yaw);
    return true;
}

bool gps::pose_to_fix (cpswarm_msgs::pose_to_fix::Request &req, cpswarm_msgs::pose_to_fix::Response &res)
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

bool gps::pose_to_target (mavros_gps::pose_to_target::Request &req, mavros_gps::pose_to_target::Response &res)
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

bool gps::target_to_fix (mavros_gps::target_to_fix::Request &req, mavros_gps::target_to_fix::Response &res)
{
    res.fix = target_to_fix(req.target);
    return true;
}

double gps::dist (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
{
    // convert to angle
    angle lat1 = angle(start.latitude, false);
    angle lon1 = angle(start.longitude, false);
    angle lat2 = angle(goal.latitude, false);
    angle lon2 = angle(goal.longitude, false);

    // compute great circle distance
    return 2 * R * asin(sqrt(pow(sin((lat2.rad() - lat1.rad()) / 2), 2) + cos(lat1.rad()) * cos(lat2.rad()) * pow(sin((lon2.rad() - lon1.rad()) / 2), 2)));
}

double gps::dist (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const
{
    return dist(start, target_to_fix(goal));
}

double gps::dist (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const
{
    return dist(target_to_fix(start), goal);
}

double gps::dist (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const
{
    return dist(target_to_fix(start), target_to_fix(goal));
}

mavros_msgs::GlobalPositionTarget gps::fix_to_target (sensor_msgs::NavSatFix fix, double yaw) const
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

sensor_msgs::NavSatFix gps::goal (sensor_msgs::NavSatFix start, double distance, double yaw) const
{

    // convert to angle
    angle lat1 = angle(start.latitude, false);
    angle lon1 = angle(start.longitude, false);

    // compute goal
    angle lat2 = angle(asin(sin(lat1.rad()) * cos(distance / R) + cos(lat1.rad()) * sin(distance / R) * cos(ned_to_enu(yaw))));
    angle lon2 = angle(lon1.rad() + atan2(sin(ned_to_enu(yaw)) * sin(distance / R) * cos(lat1.rad()), cos(distance / R) - sin(lat1.rad()) * sin(lat2.rad())));

    // goal coordinates
    sensor_msgs::NavSatFix goal;
    goal.latitude = lat2.deg();
    goal.longitude = lon2.deg();
    goal.altitude = start.altitude; // keep altitude

    return goal;
}

sensor_msgs::NavSatFix gps::goal (mavros_msgs::GlobalPositionTarget start, double distance, double yaw) const
{
    return goal(target_to_fix(start), distance, yaw);
}

double gps::ned_to_enu (double yaw) const
{
    return angle(2 * M_PI - yaw + M_PI / 2).rad_pos();
}

sensor_msgs::NavSatFix gps::target_to_fix (mavros_msgs::GlobalPositionTarget target) const
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

double gps::yaw (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
{
    // convert to angle
    angle lat1 = angle(start.latitude, false);
    angle lon1 = angle(start.longitude, false);
    angle lat2 = angle(goal.latitude, false);
    angle lon2 = angle(goal.longitude, false);

    // compute initial yaw
    double dx = cos(lat1.rad()) * sin(lat2.rad()) - sin(lat1.rad()) * cos(lat2.rad()) * cos(lon2.rad() - lon1.rad());
    double dy = sin(lon2.rad() - lon1.rad()) * cos(lat2.rad());
    angle yaw = angle(atan2(dy, dx));
    return yaw.rad_pos();
}

double gps::yaw (sensor_msgs::NavSatFix start, mavros_msgs::GlobalPositionTarget goal) const
{
    return yaw(start, target_to_fix(goal));
}

double gps::yaw (mavros_msgs::GlobalPositionTarget start, sensor_msgs::NavSatFix goal) const
{
    return yaw(target_to_fix(start), goal);
}

double gps::yaw (mavros_msgs::GlobalPositionTarget start, mavros_msgs::GlobalPositionTarget goal) const
{
    return yaw(target_to_fix(start), target_to_fix(goal));
}

void gps::pose_callback (const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    // store pose as origin class variable
    origin = *msg;

    ROS_INFO("Origin [%f, %f, %.2f]", origin.latitude, origin.longitude, origin.altitude);

    // unsubscribe
    pose_sub.shutdown();
}
