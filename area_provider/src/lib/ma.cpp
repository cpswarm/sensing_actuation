#include "lib/ma.h"

ma::ma ()
{
    // init map publisher
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("area/map", queue_size, true);

    // subscribe to map
    map_exists = false;
    map_subscriber = nh.subscribe("map", queue_size, &ma::map_callback, this);

    // wait a bit to see if there is a map provided by another node
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 1.0);
    Rate rate(loop_rate);
    double wait_for_map;
    nh.param(this_node::getName() + "/wait_for_map", wait_for_map, 2.0);
    Time start = Time::now();
    while (ok() && Time::now() < start + Duration(wait_for_map)) {
        spinOnce();
        rate.sleep();
    }

    // there is a map, extract it's coordinates
    if (map_exists)
        map_to_coords();

    // if there is no map, import coordinates
    else {
        read_coords();

        // convert global coordinates
        if (global) {
            global_to_local ();
        }

        set_origin();
    }

    // publish grid map
    if (map_exists || create_map)
        map_publisher.publish(get_gridmap());
    else
        ROS_INFO("Not publishing map");
}

void ma::map_to_coords ()
{
    // extract coordinates from map
    pair<double,double> c;

    // top right
    c.first += gridmap.info.height * gridmap.info.resolution;
    coords.insert(c);

    // top left
    c.first = gridmap.info.origin.position.x;
    coords.insert(c);

    // bottom left
    c.first = gridmap.info.origin.position.x;
    c.second = gridmap.info.origin.position.y;
    coords.insert(c);

    // bottom right
    c.first += gridmap.info.width * gridmap.info.resolution;
    coords.insert(c);
}

void ma::read_coords ()
{
    // read area coordinates
    vector<double> area_x;
    vector<double> area_y;
    nh.getParam(this_node::getName() + "/area_x", area_x);
    nh.getParam(this_node::getName() + "/area_y", area_y);
    if (map_exists == false && (area_x.size() != area_y.size() || area_x.size() < 3)) {
        ROS_FATAL("AREA_PROV - Invalid area, it must contain at least three coordinates! Exiting...");
        shutdown();
    }

    // store area coordinates in right format
    for (int i = 0; i < area_x.size(); ++i) {
        coords.emplace(area_x[i], area_y[i]);
    }

    sort_coords();
}

void ma::map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // store map
    gridmap = *msg;
    map_exists = true;

    // use only initial map
    map_subscriber.shutdown();
}
