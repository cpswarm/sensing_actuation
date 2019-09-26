#include "lib/obstacle_detection.h"

obstacle_detection::obstacle_detection ()
{
    // init swarm position
    swarm = new swarm_position();

    // add range sensors
    vector<string> range_sensor_topics;
    nh.getParam(this_node::getName() + "/range_sensor_topics", range_sensor_topics);
    vector<double> range_sensor_angles;
    nh.getParam(this_node::getName() + "/range_sensor_angles", range_sensor_angles);
    if (range_sensor_topics.size() != range_sensor_angles.size()) {
        ROS_FATAL("Invalid range sensor specification!");
    }
    for (int i = 0; i < range_sensor_topics.size(); ++i)
        range_sensors.emplace(i, new range_sensor(range_sensor_topics[i], range_sensor_angles[i]));

    // add lidar
    string lidar_sensor_topic = "";
    nh.getParam(this_node::getName() + "/lidar_sensor_topic", lidar_sensor_topic);
    if (lidar_sensor_topic == "")
        lidar = nullptr;
    else
        lidar = new lidar_sensor(lidar_sensor_topic);

    // position subscriber
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &obstacle_detection::pose_callback, this);
}

obstacle_detection::~obstacle_detection ()
{
    delete swarm;
    for (auto sensor : range_sensors)
        delete sensor.second;
    if (lidar != nullptr)
        delete lidar;
}

bool obstacle_detection::clear_of_obstacles (cpswarm_msgs::ClearOfObstacles::Request &req, cpswarm_msgs::ClearOfObstacles::Response &res)
{
    // clear of obstacles by default
    res.clear = true;

    // check range sensors
    for (auto sensor : range_sensors) {
        // at least one range sensor detected an obstacles
        if (sensor.second->obstacle()) {
            ROS_DEBUG("OBST_DETECT - Range sensor obstacle");
            res.clear = false;
            break;
        }
    }

    // check lidar sensor
    if (lidar != nullptr) {
        if (lidar->clear_ahead() == false) {
            ROS_DEBUG("OBST_DETECT - Lidar sensor obstacle");
            res.clear = false;
        }
    }

    // another cps detected by communication
    if (swarm->clear_ahead(yaw.rad_pos()) == false) {
        ROS_DEBUG("OBST_DETECT - Swarm sensor obstacle");
        res.clear = false;
    }

    return true;
}

bool obstacle_detection::danger (cpswarm_msgs::Danger::Request &req, cpswarm_msgs::Danger::Response &res)
{
    // no danger by default
    res.danger = false;
    res.backoff = 0;

    // check range sensors
    for (auto sensor : range_sensors) {
        // return most critical sensor
        if (sensor.second->danger() > res.backoff) {
            res.danger = true;
            res.backoff = sensor.second->danger();
            res.direction = sensor.second->get_angle() + M_PI;
        }
    }

    // check lidar sensor
    if (lidar != nullptr) {
        if (lidar->danger() > res.backoff) {
            res.danger = true;
            res.backoff = lidar->danger();
            res.direction = lidar->occupied_region().inverse().center();
        }
    }

    // another cps detected by communication
    if (swarm->danger() > res.backoff) {
        res.danger = true;
        res.backoff = swarm->danger();
        res.direction = swarm->occupied_region().inverse().center();
    }

    return true;
}

bool obstacle_detection::get_clear_sector (cpswarm_msgs::GetSector::Request &req, cpswarm_msgs::GetSector::Response &res)
{
    get_occupied_sector(req, res);

    sector occupied = sector(res.min, res.max);
    sector clear = occupied.inverse();

    res.min = clear.min();
    res.max = clear.max_ord();

    return true;
}

bool obstacle_detection::get_occupied_sector (cpswarm_msgs::GetSector::Request &req, cpswarm_msgs::GetSector::Response &res)
{
    // compute minimum and maximum bearing of sector occupied by obstacles according to range sensors
    double obst_min = 0;
    double obst_max = 0;
    for (auto sensor : range_sensors) {
        if (sensor.second->obstacle()) {
            double obst_min_temp = (yaw + angle(sensor.second->get_angle()) - angle(sensor.second->get_fov() / 2.0)).rad_pos(); // add fov and not fov/2 to have safety margin
            double obst_max_temp = (yaw + angle(sensor.second->get_angle()) + angle(sensor.second->get_fov() / 2.0)).rad_pos();
            if (obst_min_temp < obst_min || obst_min == 0)
                obst_min = obst_min_temp;
            if (obst_max_temp > obst_max || obst_max == 0)
                obst_max = obst_max_temp;
        }
    }
    sector occupied = sector(obst_min, obst_max);

    // compute minimum and maximum bearing of sector occupied by obstacles according to lidar
    if (lidar != nullptr) {
        sector lidar_sect = lidar->inflated_region();
        occupied.join(lidar_sect);
    }

    // compute minimum and maximum bearing of sector occupied by other cpss
    sector swarm_sect = swarm->inflated_region();

    // join all sectors
    occupied.join(swarm_sect);

    // return minimum and maximum angle (min < max)
    res.min = occupied.min();
    res.max = occupied.max_ord();

    return true;
}

void obstacle_detection::pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // store new orientation in class variable
    tf2::Quaternion orientation;
    tf2::fromMsg(msg->pose.orientation, orientation);
    yaw = angle(tf2::getYaw(orientation));
}
