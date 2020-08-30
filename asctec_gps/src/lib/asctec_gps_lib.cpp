#include "lib/asctec_gps_lib.h"

asctec_gps_lib::asctec_gps_lib ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);

    // init origin
    pose_sub = nh.subscribe("GPS_DATA", queue_size, &asctec_gps_lib::pose_callback, this);
    while (ok() && origin.latitude == 0) {
        rate.sleep();
        spinOnce();
    }
}

bool asctec_gps_lib::fix_to_pose (cpswarm_msgs::FixToPose::Request &req, cpswarm_msgs::FixToPose::Response &res)
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

bool asctec_gps_lib::fix_to_gpsdata (asctec_gps::FixToGpsdata::Request &req, asctec_gps::FixToGpsdata::Response &res)
{
    res.gpsdata = fix_to_gpsdata(req.fix, req.yaw);
    return true;
}

bool asctec_gps_lib::get_gps_origin (cpswarm_msgs::GetGpsFix::Request &req, cpswarm_msgs::GetGpsFix::Response &res)
{
    res.fix = origin;
    return true;
}

bool asctec_gps_lib::gpsdata_to_fix (asctec_gps::GpsdataToFix::Request &req, asctec_gps::GpsdataToFix::Response &res)
{
    res.fix = gpsdata_to_fix(req.gpsdata);
    return true;
}

bool asctec_gps_lib::gpsdata_to_pose (asctec_gps::GpsdataToPose::Request &req, asctec_gps::GpsdataToPose::Response &res)
{
    res.pose = gpsdata_to_pose(req.gpsdata);
    return true;
}

bool asctec_gps_lib::ned_to_enu (cpswarm_msgs::NedToEnu::Request &req, cpswarm_msgs::NedToEnu::Response &res)
{
    res.yaw = ned_to_enu(req.yaw);
    return true;
}

bool asctec_gps_lib::pose_to_fix (cpswarm_msgs::PoseToFix::Request &req, cpswarm_msgs::PoseToFix::Response &res)
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

bool asctec_gps_lib::pose_to_gpsdata (asctec_gps::PoseToGpsdata::Request &req, asctec_gps::PoseToGpsdata::Response &res)
{
    // compute gps coordinates
    double dist = hypot(req.pose.pose.position.x, req.pose.pose.position.y);
    double head = atan2(req.pose.pose.position.y, req.pose.pose.position.x);
    tf2::Quaternion orientation;
    tf2::fromMsg(req.pose.pose.orientation, orientation);
    double yaw = tf2::getYaw(orientation);
    res.gpsdata = fix_to_gpsdata(goal(origin, dist, head), yaw);

    // copy header
    res.gpsdata.header = req.pose.header;

    // copy altitude
    res.gpsdata.height = int(req.pose.pose.position.z * 1000);

    return true;
}

double asctec_gps_lib::dist (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
{
    // convert to radian
    double lat1 = start.latitude / 180.0 * M_PI;
    double lon1 = start.longitude / 180.0 * M_PI;
    double lat2 = goal.latitude / 180.0 * M_PI;
    double lon2 = goal.longitude / 180.0 * M_PI;

    // compute great circle distance
    return 2 * R * asin(sqrt(pow(sin((lat2 - lat1) / 2), 2) + cos(lat1) * cos(lat2) * pow(sin((lon2 - lon1) / 2), 2)));
}

double asctec_gps_lib::dist (sensor_msgs::NavSatFix start, asctec_msgs::GPSData goal) const
{
    return dist(start, gpsdata_to_fix(goal));
}

double asctec_gps_lib::dist (asctec_msgs::GPSData start, sensor_msgs::NavSatFix goal) const
{
    return dist(gpsdata_to_fix(start), goal);
}

double asctec_gps_lib::dist (asctec_msgs::GPSData start, asctec_msgs::GPSData goal) const
{
    return dist(gpsdata_to_fix(start), gpsdata_to_fix(goal));
}

asctec_msgs::GPSData asctec_gps_lib::fix_to_gpsdata (sensor_msgs::NavSatFix fix, double yaw) const
{
    // create gpsdata message
    asctec_msgs::GPSData gpsdata;
    gpsdata.header = fix.header;

    // copy coordinates
    gpsdata.latitude = fix.latitude;
    gpsdata.longitude = fix.longitude;
    gpsdata.height = int(fix.altitude * 1000);

    // set orientation for nose in forward movement direction
    gpsdata.heading = int(yaw * 100 / M_PI * 180);

    return gpsdata;
}

sensor_msgs::NavSatFix asctec_gps_lib::goal (sensor_msgs::NavSatFix start, double distance, double yaw) const
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

sensor_msgs::NavSatFix asctec_gps_lib::goal (asctec_msgs::GPSData start, double distance, double yaw) const
{
    return goal(gpsdata_to_fix(start), distance, yaw);
}

sensor_msgs::NavSatFix asctec_gps_lib::gpsdata_to_fix (asctec_msgs::GPSData gpsdata) const
{
    // create message and copy header
    sensor_msgs::NavSatFix fix;
    fix.header = gpsdata.header;

    // copy coordinates
    fix.latitude = gpsdata.latitude;
    fix.longitude = gpsdata.longitude;
    fix.altitude = double(gpsdata.height) / 1000;

    return fix;
}

geometry_msgs::PoseStamped asctec_gps_lib::gpsdata_to_pose (asctec_msgs::GPSData gpsdata) const
{
    // create message and copy header
    geometry_msgs::PoseStamped pose;
    pose.header = gpsdata.header;

    // compute local coordinates
    double dist = this->dist(origin, gpsdata);
    double head = ned_to_enu(yaw(origin, gpsdata));
    pose.pose.position.x = dist * cos(head);
    pose.pose.position.y = dist * sin(head);
    pose.pose.position.z = double(gpsdata.height) / 1000;

    return pose;
}

double asctec_gps_lib::ned_to_enu (double yaw) const
{
    return remainder(2 * M_PI - yaw + M_PI / 2, 2*M_PI);
}

double asctec_gps_lib::yaw (sensor_msgs::NavSatFix start, sensor_msgs::NavSatFix goal) const
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

double asctec_gps_lib::yaw (sensor_msgs::NavSatFix start, asctec_msgs::GPSData goal) const
{
    return yaw(start, gpsdata_to_fix(goal));
}

double asctec_gps_lib::yaw (asctec_msgs::GPSData start, sensor_msgs::NavSatFix goal) const
{
    return yaw(gpsdata_to_fix(start), goal);
}

double asctec_gps_lib::yaw (asctec_msgs::GPSData start, asctec_msgs::GPSData goal) const
{
    return yaw(gpsdata_to_fix(start), gpsdata_to_fix(goal));
}

void asctec_gps_lib::pose_callback (const asctec_msgs::GPSData::ConstPtr& msg)
{
    // store pose as origin class variable
    origin = gpsdata_to_fix(*msg);

    ROS_INFO("Origin [%f, %f, %.2f]", origin.latitude, origin.longitude, origin.altitude);

    // unsubscribe
    pose_sub.shutdown();
}
