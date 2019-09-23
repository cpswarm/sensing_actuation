#include "lib/area.h"

area::area ()
{
    // positioning type
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    bool global = pos_type == "local" ? false : true;

    // grid map resolution
    nh.getParam(this_node::getName() + "/resolution", resolution);

    // read area coordinates
    vector<double> area_x;
    vector<double> area_y;
    nh.getParam(this_node::getName() + "/area_x", area_x);
    nh.getParam(this_node::getName() + "/area_y", area_y);
    if (area_x.size() != area_y.size() || area_x.size() < 3) {
        ROS_FATAL("AREA_PROV - Invalid area, it must contain at least three coordinates! Exiting...");
        shutdown();
    }
    vector<pair<double,double>> raw_coords;
    for (int i = 0; i < area_x.size(); ++i)
        raw_coords.emplace_back(area_x[i], area_y[i]);

    // global positioning
    if (global) {
        // service client for converting GPS to local coordinates
        ServiceClient fix_to_pose_client = nh.serviceClient<cpswarm_msgs::fix_to_pose>("gps/fix_to_pose");
        cpswarm_msgs::fix_to_pose f2p;

        // convert given area to local coordinates
        for (int i = 0; i < raw_coords.size(); ++i) {
            f2p.request.fix.longitude = raw_coords[i].first;
            f2p.request.fix.latitude = raw_coords[i].second;
            if (fix_to_pose_client.call(f2p)) {
                coords.push_back(f2p.response.pose.pose.position);
            }
            else
                ROS_FATAL("AREA_PROV - Failed to convert area bounds to local coordinates");
        }

        // get origin from gps node
        ServiceClient get_gps_origin_client = nh.serviceClient<cpswarm_msgs::get_gps_origin>("gps/get_gps_origin");
        cpswarm_msgs::get_gps_origin gpso;
        if (get_gps_origin_client.call(gpso)) {
            // convert origin to local coordinates
            f2p.request.fix = gpso.response.origin;
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
        for (int i = 0; i < raw_coords.size(); ++i) {
            // copy given area coordinates
            geometry_msgs::Point p;
            p.x = raw_coords[i].first;
            p.y = raw_coords[i].second;
            coords.push_back(p);

            // read origin from parameters
            double x,y;
            nh.param(this_node::getName() + "/x", x, 0.0);
            nh.param(this_node::getName() + "/y", y, 0.0);
            origin.x = x;
            origin.y = y;
        }
    }
}

bool area::closest_bound (cpswarm_msgs::closest_bound::Request &req, cpswarm_msgs::closest_bound::Response &res)
{
    // use origin
    if (req.point.x == 0 && req.point.y == 0) {
        req.point.x = origin.x;
        req.point.y = origin.y;
    }

    // find minimal distance to any area bound
    for (int i = 0; i < coords.size(); ++i) {
        // coordinates of two neighboring polygon points
        geometry_msgs::Point p1;
        p1.x = coords[i].x;
        p1.y = coords[i].y;
        geometry_msgs::Point p2;
        p2.x = coords[(i+1)%coords.size()].x;
        p2.y = coords[(i+1)%coords.size()].y;

        // minimal distance to line connecting the two points
        double dist = abs((p2.y - p1.y) * req.point.x - (p2.x - p1.x) * req.point.y + p2.x * p1.y - p2.y * p1.x) / hypot(p2.x - p1.x, p2.y - p1.y);

        // found smaller distance
        if (res.dist == 0 || dist < res.dist) {
            // return indexes
            res.index.clear();
            res.index.push_back(i);
            res.index.push_back((i+1) % coords.size());

            // return actual coordinates
            res.coords.clear();
            res.coords.push_back(p1);
            res.coords.push_back(p2);

            // return distance
            res.dist = dist;
        }
    }

    return true;
}

bool area::get_area (cpswarm_msgs::get_area::Request &req, cpswarm_msgs::get_area::Response &res)
{
    res.area = coords;
    return true;
}

nav_msgs::OccupancyGrid area::get_gridmap ()
{
    // create grid map
    nav_msgs::OccupancyGrid map;

    // get coordinates
    double xmin = numeric_limits<double>::max();
    double xmax = numeric_limits<double>::min();
    double ymin = numeric_limits<double>::max();
    double ymax = numeric_limits<double>::min();
    for (auto p : coords) {
        if (p.x < xmin)
            xmin = p.x;
        if (p.x > xmax)
            xmax = p.x;
        if (p.y < ymin)
            ymin = p.y;
        if (p.y > ymax)
            ymax = p.y;
    }
    int x = int(ceil((xmax - xmin) / resolution));
    int y = int(ceil((ymax - ymin) / resolution));

    // generate grid map data
    cpswarm_msgs::out_of_bounds::Request req;
    cpswarm_msgs::out_of_bounds::Response res;
    vector<int8_t> data;
    for (int i=0; i<y; ++i) { // row major order
        for (int j=0; j<x; ++j) {
            // check if cell is within area
            req.pose.position.x = j * resolution + xmin;
            req.pose.position.y = i * resolution + ymin;
            out_of_bounds(req, res);

            // out of bounds
            if (res.out)
                data.push_back(100); // occupied

            // inside area
            else
                data.push_back(-1); // unknown
        }
    }
    map.data = data;

    // set map header
    map.header.stamp = Time::now();
    map.header.frame_id = "local_origin_ned";

    // set map meta data
    map.info.map_load_time == Time::now();
    map.info.resolution = resolution;
    map.info.width = x;
    map.info.height = y;
    // position of cell (0,0)
    map.info.origin.position.x = xmin;
    map.info.origin.position.y = ymin;

    return map;
}

bool area::get_origin (cpswarm_msgs::get_origin::Request &req, cpswarm_msgs::get_origin::Response &res)
{
    res.origin = origin;
    return true;
}

bool area::out_of_bounds (cpswarm_msgs::out_of_bounds::Request &req, cpswarm_msgs::out_of_bounds::Response &res)
{
    // sum of the angles made between pose and each pair of points making up the polygon
    double angle_sum = 0;

    // compute the angle sum
    for (int i = 0; i < coords.size(); ++i) {
        // coordinates of two neighboring polygon points relative to the given pose
        geometry_msgs::Point p1;
        p1.x = coords[i].x - req.pose.position.x;
        p1.y = coords[i].y - req.pose.position.y;
        geometry_msgs::Point p2;
        p2.x = coords[(i+1)%coords.size()].x - req.pose.position.x;
        p2.y = coords[(i+1)%coords.size()].y - req.pose.position.y;

        // angle between the two points
        angle_sum += remainder(atan2(p2.y, p2.x) - atan2(p1.y, p1.x), 2*M_PI);
    }

    // pose is outside (sum = 0)
    if (abs(angle_sum) < M_PI)
        res.out = true;

    // pose is inside (sum = 2π)
    else
        res.out = false;

    return true;
}
