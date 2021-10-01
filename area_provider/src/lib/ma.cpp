#include "lib/ma.h"

ma::ma ()
{
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

    // mission area coordinates
    vector<pair<double,double>> raw_coords;

    // there is a map, extract it's coordinates
    if (map_exists)
        map_to_coords();

    // if there is no map, import coordinates
    else
        read_coords();
}

void ma::map_to_coords ()
{
    // extract coordinates from map
    geometry_msgs::Point c;

    // bottom left
    c = map.info.origin.position;
    coords.push_back(c);

    // bottom right
    c.x += map.info.width * map.info.resolution;
    coords.push_back(c);

    // top right
    c.y += map.info.height * map.info.resolution;
    coords.push_back(c);

    // top left
    c.x = map.info.origin.position.x;
    coords.push_back(c);
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

    // global positioning
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    bool global = pos_type == "local" ? false : true;
    if (global) {
        // service client for converting GPS to local coordinates
        ServiceClient fix_to_pose_client = nh.serviceClient<cpswarm_msgs::FixToPose>("gps/fix_to_pose");
        ROS_DEBUG("Wait for fix_to_pose service...");
        fix_to_pose_client.waitForExistence();
        cpswarm_msgs::FixToPose f2p;

        // convert given area to local coordinates
        for (int i = 0; i < area_x.size(); ++i) {
            f2p.request.fix.longitude = area_x[i];
            f2p.request.fix.latitude = area_y[i];
            if (fix_to_pose_client.call(f2p)) {
                coords.push_back(f2p.response.pose.pose.position);
            }
            else
                ROS_FATAL("AREA_PROV - Failed to convert area bounds to local coordinates");
        }

        // get origin from gps node
        ServiceClient get_gps_origin_client = nh.serviceClient<cpswarm_msgs::GetGpsFix>("gps/get_gps_origin");
        ROS_DEBUG("Wait for get_gps_origin service...");
        get_gps_origin_client.waitForExistence();
        cpswarm_msgs::GetGpsFix gpso;
        if (get_gps_origin_client.call(gpso)) {
            // convert origin to local coordinates
            f2p.request.fix = gpso.response.fix;
            if (fix_to_pose_client.call(f2p)) {
                origin = f2p.response.pose.pose.position;
            }
            else
                ROS_FATAL("AREA_PROV - Failed to convert origin to local coordinates");
        }
        else
            ROS_FATAL("AREA_PROV - Failed get GPS coordinates of origin");
    }

    // local positioning
    else {
        for (int i = 0; i < area_x.size(); ++i)
            coords.emplace_back(area_x[i], area_y[i], 0);

        // read origin from parameters
        double x,y;
        nh.param(this_node::getName() + "/x", x, 0.0);
        nh.param(this_node::getName() + "/y", y, 0.0);
        origin.x = x;
        origin.y = y;
    }
}

void ma::map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // store map
    map = *msg;
    map_exists = true;

    // use only initial map
    map_subscriber.shutdown();
}
