#include "lib/area.h"

area::area ()
{
    // read parameters
    nh.param(this_node::getName() + "/cell_warn", cell_warn, 1000);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);

    // init map publisher
    nh.param(this_node::getName() + "/create_map", create_map, false);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("area/map", queue_size, true);

    // subscribe to map
    map_exists = false;
    map_subscriber = nh.subscribe("map", queue_size, &area::map_callback, this);

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

    // if there is no map, initialize area from coordinates
    if (map_exists == false)
        init_area();
}

bool area::closest_bound (cpswarm_msgs::ClosestBound::Request &req, cpswarm_msgs::ClosestBound::Response &res)
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

bool area::get_area (cpswarm_msgs::GetPoints::Request &req, cpswarm_msgs::GetPoints::Response &res)
{
    res.points = coords;
    return true;
}

bool area::get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    // compute centroid / barycenter
    for (auto c : coords) {
        res.point.x += c.x;
        res.point.y += c.y;
    }
    res.point.x /= coords.size();
    res.point.y /= coords.size();
    return true;
}

nav_msgs::OccupancyGrid area::get_gridmap ()
{
    // no map existing yet, i.e., no map server
    if (map_exists == false && create_map == true) {
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

        // warn about large grids
        if (x*y > cell_warn)
            ROS_WARN("Given coordinates seem wrong, grid map extremley large: %d cells!", x*y);

        // generate grid map data
        cpswarm_msgs::OutOfBounds::Request req;
        cpswarm_msgs::OutOfBounds::Response res;
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
                    data.push_back(0); // free
            }
        }
        map.data = data;

        // set map header
        map.header.stamp = Time::now();
        map.header.frame_id = "map";

        // set map meta data
        map.info.map_load_time == Time::now();
        map.info.resolution = resolution;
        map.info.width = x;
        map.info.height = y;

        // position of cell (0,0)
        map.info.origin.position.x = xmin;
        map.info.origin.position.y = ymin;

        map_exists = true;
    }

    return map;
}

bool area::get_map (nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    if (map_exists == false && create_map == false)
        ROS_WARN("Providing empty map!");
    res.map = get_gridmap();
    return true;
}

bool area::get_origin (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    res.point = origin;
    return true;
}

bool area::get_rotation (cpswarm_msgs::GetDouble::Request &req, cpswarm_msgs::GetDouble::Response &res)
{
    // get coordinates
    geometry_msgs::Point pl, pb, pr;
    pl.x = numeric_limits<double>::max();
    pb.y = numeric_limits<double>::max();
    pr.x = numeric_limits<double>::min();
    for (auto p : coords) {
        // left most point
        if (p.x < pl.x || (p.x == pl.x && p.y < pl.y))
            pl = p;

        // bottom most point
        if (p.y < pb.y)
            pb = p;

        // right most point
        if (p.x > pr.x || (p.x == pr.x && p.y < pr.y))
            pr = p;
    }

    // no rotation required
    if ((pl.x == pb.x && pl.y == pb.y) || (pr.x == pb.x && pr.y == pb.y))
        return false;

    // rotate clockwise
    if (pr.y < pl.y) {
        res.value = -atan2(pr.y - pb.y, pr.x - pb.x);
    }

    // rotate counter clockwise
    else {
        res.value = -atan2(pb.y - pl.y, pb.x - pl.x);
    }

    return true;
}

bool area::out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res)
{
    // the winding number counter, i.e., how often a ray from the point to the right crosses the boundary
    int wn = 0;

    // loop through all edges of the polygon
    for (int i = 0; i < coords.size(); ++i) {
        // ray crosses upward edge
        if (coords[i].y <= req.pose.position.y && coords[(i+1)%coords.size()].y  > req.pose.position.y && is_left(coords[i], coords[(i+1)%coords.size()], req.pose.position))
            ++wn;

        // ray crosses downward edge
        else if (coords[i].y > req.pose.position.y && coords[(i+1)%coords.size()].y  <= req.pose.position.y && is_right(coords[i], coords[(i+1)%coords.size()], req.pose.position))
            --wn;
    }

    // pose is outside
    if (wn == 0)
        res.out = true;

    // pose is inside
    else
        res.out = false;

    return true;
}

void area::init_area ()
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
    vector<pair<double,double>> raw_coords;
    for (int i = 0; i < area_x.size(); ++i)
        raw_coords.emplace_back(area_x[i], area_y[i]);

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
        for (int i = 0; i < raw_coords.size(); ++i) {
            // copy given area coordinates
            geometry_msgs::Point p;
            p.x = raw_coords[i].first;
            p.y = raw_coords[i].second;
            coords.push_back(p);
        }

        // read origin from parameters
        double x,y;
        nh.param(this_node::getName() + "/x", x, 0.0);
        nh.param(this_node::getName() + "/y", y, 0.0);
        origin.x = x;
        origin.y = y;
    }

    // no coordinates given, extract from map
    if (map_exists && coords.size() < 3) {
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

    // publish grid map
    if (map_exists || create_map)
        map_publisher.publish(get_gridmap());
    else
        ROS_INFO("Not publishing map");
}

bool area::is_left (geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    // >0 for p2 left of the line through p0 and p1
    // =0 for p2 on the line
    // <0 for p2 right of the line
    return ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x -  p0.x) * (p1.y - p0.y)) > 0;
}

bool area::is_right (geometry_msgs::Point p0, geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    // >0 for p2 left of the line through p0 and p1
    // =0 for p2 on the line
    // <0 for p2 right of the line
    return ((p1.x - p0.x) * (p2.y - p0.y) - (p2.x -  p0.x) * (p1.y - p0.y)) < 0;
}

void area::map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // store map
    map = *msg;
    map_exists = true;

    // use only initial map
    map_subscriber.shutdown();

    // initialize area coordinates
    init_area();
}
