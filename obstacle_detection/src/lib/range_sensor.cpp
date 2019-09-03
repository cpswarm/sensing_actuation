#include "lib/range_sensor.h"

range_sensor::range_sensor (string topic, double angle) : angle(angle)
{
    // read parameters
    nh.param(this_node::getName() + "/obstacle_dist_min", min_dist, 0.0);
    nh.param(this_node::getName() + "/obstacle_dist_max", max_dist, 1.5);
    nh.param(this_node::getName() + "/critical_dist", critical_dist, 1.5);
    int num_msgs;
    nh.param(this_node::getName() + "/num_msgs", num_msgs, 1);
    nh.param(this_node::getName() + "/threshold", threshold, 0.5);

    // initialize parameters
    fov = 0;
    range = -1;
    cur_msg = 0;
    dirty = false;

    // subscribe to sensor readings
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    subscriber = nh.subscribe(topic, queue_size, &range_sensor::callback, this);

    // initialize range sensor messages
    for (int i = 0; i < num_msgs; ++i) {
        sensor_msgs::Range range;
        range.range = -1;
        msgs.push_back(range);
    }
}

double range_sensor::danger ()
{
    // process new sensor messages
    if (dirty)
        process();

    // obstacle is dangerously close
    if (0 < range && range < critical_dist)
        return critical_dist - range;

    // no obstacle near by
    else
        return 0.0;
}

double range_sensor::get_angle () const
{
    return angle;
}

double range_sensor::get_fov () const
{
    return fov;
}

double range_sensor::get_range()
{
    // process new sensor messages
    if (dirty)
        process();

    return range;
}

bool range_sensor::obstacle ()
{
    // process new sensor messages
    if (dirty)
        process();

    return range > 0;
}

void range_sensor::process ()
{
    // count number of messages with obstacle detection
    double total_range = 0;
    unsigned int obstacle = 0;
    unsigned int free = 0;
    for (int i = 0; i < msgs.size(); ++i) {
        if (msgs[i].min_range < msgs[i].range && msgs[i].range < msgs[i].max_range && min_dist < msgs[i].range && msgs[i].range < max_dist) {
            ++obstacle;
            total_range += msgs[i].range;
        }
        else {
            ++free;
        }
    }

    // enough messages report an obstacle, compute average range
    if (double(obstacle) / double(free + obstacle) >= threshold)
        range = total_range / double(obstacle);

    // no obstacle
    else
        range = -1;

    // all messages have been processed
    dirty = false;
}

void range_sensor::callback (const sensor_msgs::Range::ConstPtr& msg)
{
    // increase current position of message in vector
    cur_msg++;
    cur_msg %= msgs.size();

    // save fov
    fov = msg->field_of_view;

    // save range message
    msgs[cur_msg] = *msg;

    // sensor needs processing
    dirty = true;
}
