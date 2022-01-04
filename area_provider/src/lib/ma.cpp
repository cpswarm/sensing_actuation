#include "lib/ma.h"

ma::ma ()
{
    // init map publisher
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("area/map", queue_size, true);

    // subscribe to map
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
    if (gridmaps.count(0) > 0)
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
    if (gridmaps.count(0) > 0 || create_map)
        map_publisher.publish(get_gridmap());
    else
        ROS_INFO("Not publishing map");
}

void ma::map_to_coords ()
{
    // extract coordinates from (un-rotated) map
    pair<double,double> c;

    // top right
    c.first += gridmaps[0][resolution].info.height * resolution;
    coords[0].insert(c);

    // top left
    c.first = gridmaps[0][resolution].info.origin.position.x;
    coords[0].insert(c);

    // bottom left
    c.first = gridmaps[0][resolution].info.origin.position.x;
    c.second = gridmaps[0][resolution].info.origin.position.y;
    coords[0].insert(c);

    // bottom right
    c.first += gridmaps[0][resolution].info.width * resolution;
    coords[0].insert(c);
}

void ma::read_coords ()
{
    // read area coordinates
    vector<double> area_x;
    vector<double> area_y;
    nh.getParam(this_node::getName() + "/area_x", area_x);
    nh.getParam(this_node::getName() + "/area_y", area_y);
    if (gridmaps.count(0) == 0 && gridmaps[0].count(resolution) == 0 && (area_x.size() != area_y.size() || area_x.size() < 3)) {
        ROS_FATAL("AREA_PROV - Invalid area, it must contain at least three coordinates! Exiting...");
        shutdown();
    }

    // store area coordinates in right format
    for (int i = 0; i < area_x.size(); ++i) {
        coords[0].emplace(area_x[i], area_y[i]);
    }

    sort_coords();
}

void ma::map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // store resolution of given map
    resolution = msg->info.resolution;

    // store un-rotated map
    gridmaps[0][resolution] = *msg;

    // use only initial map
    map_subscriber.shutdown();
}
