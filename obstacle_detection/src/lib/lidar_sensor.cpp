#include "lib/lidar_sensor.h"

lidar_sensor::lidar_sensor (string topic)
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/avoidance_dist", avoidance_dist, 2.0);
    nh.param(this_node::getName() + "/critical_dist", critical_dist, 1.0);
    nh.param(this_node::getName() + "/clear_ang", clear_ang, 0.785);
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);

    // no laser scan received yet
    data_valid = false;

    // publishers and subscribers
    subscriber = nh.subscribe(topic, queue_size, &lidar_sensor::callback, this);

    while (ok() && data_valid == false){
        ROS_DEBUG_ONCE("OBST_DETECT - Waiting for lidar data...");
        rate.sleep();
        spinOnce();
    }
}

bool lidar_sensor::clear () const
{
    // check ranges
    for (auto range : data.ranges) {
        // obstacle is close enough to start avoidance procedure
        if (data.range_min < range && range < avoidance_dist && range < data.range_max) {
            return false;
        }
    }

    // no obstacle near by
    return true;
}

bool lidar_sensor::clear_ahead () const
{
    // first and last index of scan sector that must be clear of obstacles
    int i_mid = int(-data.angle_min / data.angle_increment);
    int i_min = max(0, i_mid - int(clear_ang / data.angle_increment));
    int i_max = min(int(data.ranges.size()), i_mid + int(clear_ang / data.angle_increment));

    ROS_DEBUG("OBST_DETECT - Lidar clear ahead indexes %i --> %i --> %i", i_min, i_mid, i_max);

    // check current sensor reading
    for (int i=i_min; i<=i_max; ++i) {
        // obstacle is close enough to start avoidance procedure
        if (data.range_min < data.ranges[i] && data.ranges[i] < avoidance_dist && data.ranges[i] < data.range_max) {
            ROS_DEBUG("OBST_DETECT - Lidar clear ahead range %.2f at index %i", data.ranges[i], i);
            return false;
        }
    }

    // no obstacle near by
    return true;
}

double lidar_sensor::danger () const
{
    // check ranges
    for (auto range : data.ranges) {
        // obstacle is dangerously close
        if (data.range_min < range && range < critical_dist && range < data.range_max) {
            return critical_dist - range;
        }
    }

    // no cps near by
    return 0.0;
}

sector lidar_sensor::inflated_region () const
{
    // occupied sector
    sector occ = occupied_region();

    // inflate by clear_ang
    if (occ.size() > 0)
        occ.inflate(clear_ang);

    // return inflated sector
    return occ;
}

sector lidar_sensor::inflated_region (double heading) const
{
    // occupied sector
    sector occ = occupied_region(heading);

    // inflate by clear_ang
    if (occ.size() > 0)
        occ.inflate(clear_ang);

    // return inflated sector
    return occ;
}

sector lidar_sensor::occupied_region () const
{
    // minimum bearing occupied by obstacles
    double obst_min = 0;
    for (int i=obst_min; i<data.ranges.size(); ++i) {
        if (data.range_min < data.ranges[i] && data.ranges[i] < avoidance_dist && data.ranges[i] < data.range_max) {
            obst_min = i;
            break;
        }
    }

    // maximum bearing occupied by obstacles
    double obst_max = data.ranges.size()-1;
    for (int i=obst_max; i>=0; --i) {
        if (data.range_min < data.ranges[i] && data.ranges[i] < avoidance_dist && data.ranges[i] < data.range_max) {
            obst_max = i;
            break;
        }
    }

    // return sector
    return sector(obst_min, obst_max);
}

sector lidar_sensor::occupied_region (double heading) const
{
    // minimum bearing occupied by obstacles
    double obst_min = 0;
    for (int i=obst_min; i<data.ranges.size(); ++i) {
        if (data.range_min < data.ranges[i] && data.ranges[i] < avoidance_dist && data.ranges[i] < data.range_max) {
            obst_min = i;
            break;
        }
    }
    
    // maximum bearing occupied by obstacles
    double obst_max = data.ranges.size()-1;
    for (int i=obst_max; i>=0; --i) {
        if (data.range_min < data.ranges[i] && data.ranges[i] < avoidance_dist && data.ranges[i] < data.range_max) {
            obst_max = i;
            break;
        }
    }

    // return sector
    return sector(obst_min-heading, obst_max-heading);
}

void lidar_sensor::callback (const sensor_msgs::LaserScan::ConstPtr& msg)
{
    data = *msg;
    data_valid = true;
}
