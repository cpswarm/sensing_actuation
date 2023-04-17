#include "lib/area.h"

area::area ()
{
    // read parameters
    nh.param(this_node::getName() + "/cell_warn", cell_warn, 1000);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);

    // check of global (gps) coordinates are used
    string pos_type = "global";
    nh.param(this_node::getName() + "/pos_type", pos_type, pos_type);
    global = pos_type == "local" ? false : true;

    // service client for converting gps to local coordinates
    if (global) {
        fix_to_pose_client = nh.serviceClient<cpswarm_msgs::FixToPose>("gps/fix_to_pose");
        ROS_DEBUG("Wait for fix_to_pose service...");
        fix_to_pose_client.waitForExistence();
    }

    // map not yet rotated
    rotation = 0;
}

bool area::get_area (cpswarm_msgs::GetPoints::Request &req, cpswarm_msgs::GetPoints::Response &res)
{
    for (auto c : coords_sorted[0])
        res.points.push_back(tuple2point(get<1>(c)));
    return true;
}

bool area::get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    double area = 0;

    // compute centroid in 2d
    map<double, tuple<double,double,double>>::iterator ne;
    for (map<double, tuple<double,double,double>>::iterator it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted[0].end())
            ne = coords_sorted[0].begin();

        // sum up coordinates
        res.point.x += (get<0>(it->second) + get<0>(ne->second)) * (get<0>(it->second) * get<1>(ne->second) - get<0>(ne->second) * get<1>(it->second));
        res.point.y += (get<1>(it->second) + get<1>(ne->second)) * (get<0>(it->second) * get<1>(ne->second) - get<0>(ne->second) * get<1>(it->second));
        res.point.z += get<2>(it->second);

        // sum up area
        area += get<0>(it->second) * get<1>(ne->second) - get<0>(ne->second) * get<1>(it->second);
    }

    // normalize area
    area /= 2;

    // normalize coordinates
    res.point.x /= 6 * area;
    res.point.y /= 6 * area;

    // normalize altitude
    res.point.z /= coords_sorted[0].size();

    return true;
}

bool area::get_distance (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res)
{
    bool init = true;

    // point to check
    geometry_msgs::Point p0;

    // use origin
    if (req.point.x == 0 && req.point.y == 0)
        p0 = origin;

    // use given point
    else
        p0 = req.point;

    ROS_DEBUG("Calculate distance for point (%.2f,%.2f)", p0.x, p0.y);

    // find minimal distance to any area bound
    map<double, tuple<double,double,double>>::iterator ne;
    for (map<double, tuple<double,double,double>>::iterator it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted[0].end())
            ne = coords_sorted[0].begin();


        // coordinates of two neighboring polygon points
        geometry_msgs::Point p1 = tuple2point(it->second);
        geometry_msgs::Point p2 = tuple2point(ne->second);

        // difference between all three points
        geometry_msgs::Point p02;
        p02.x = p2.x - p0.x;
        p02.y = p2.y - p0.y;
        geometry_msgs::Point p12;
        p12.x = p2.x - p1.x;
        p12.y = p2.y - p1.y;
        geometry_msgs::Point p10;
        p10.x = p0.x - p1.x;
        p10.y = p0.y - p1.y;

        // calculate where on the line p12 the closest point lies
        double dot = p12.x * p10.x + p12.y * p10.y;
        double dis = p12.x * p12.x + p12.y * p12.y;
        double r = dot / dis;

        // distance and closest point
        double dist;
        geometry_msgs::Point closest;
        // p1 is closest point
        if (r < 0) {
            closest = p1;
            dist = hypot(p10.x, p10.y);
        }
        // p2 is closest point
        else if (r > 1) {
            closest = p2;
            dist = hypot(p02.x, p02.y);
        }
        // closest point is between p1 and p2
        else {
            closest.x = p1.x + r * p12.x;
            closest.y = p1.y + r * p12.y;
            closest.z = p1.z + r * p12.z;
            dist = sqrt((p10.x * p10.x + p10.y * p10.y) - dis * r * r);
        }

        // found smaller distance
        if (init || dist < res.distance) {
            ROS_DEBUG("Found closest point (%.2f,%.2f) on line (%.2f,%.2f)--(%.2f,%.2f) at distance %.2f", closest.x, closest.y, p1.x, p1.y, p2.x, p2.y, dist);

            // return closest point
            res.closest_point = closest;

            // return line segment
            res.closest_line.clear();
            res.closest_line.push_back(p1);
            res.closest_line.push_back(p2);

            // return distance
            res.distance = dist;

            init = false;
        }
    }

    return true;
}

bool area::get_map (cpswarm_msgs::GetMap::Request &req, cpswarm_msgs::GetMap::Response &res)
{
    // get/create map
    res.map = get_gridmap(req.rotate, req.resolution);

    // angle of rotation
    if (req.rotate)
        res.rotation = rotate();

    // translate map if requested
    if (req.translate)
        res.translation = translate(res.map);

    return true;
}

bool area::get_origin (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    res.point = origin;
    return true;
}

bool area::out_of_bounds (cpswarm_msgs::OutOfBounds::Request &req, cpswarm_msgs::OutOfBounds::Response &res)
{
    res.out = out_of_bounds(point2pair(req.pose.position));

    return true;
}

string area::to_string ()
{
    stringstream ss;
    for (auto c : coords_sorted[0])
        ss << "(" << get<0>(get<1>(c)) << "," << get<1>(get<1>(c)) << "," << get<2>(get<1>(c)) << ") ";
    return ss.str();
}

nav_msgs::OccupancyGrid area::get_gridmap (bool rotated, double resolution)
{
    // empty grid map
    nav_msgs::OccupancyGrid gridmap;

    // rotation of map
    double angle = 0;
    if (rotated) {
        angle = rotate();
    }

    // use default resolution if not specified
    if (resolution <= 0)
        resolution = this->resolution;

    // requested map not existing yet
    if (gridmaps.count(angle) == 0 || gridmaps[angle].count(resolution) == 0) {
        // get extreme coordinates
        double xmin = numeric_limits<double>::max();
        double xmax = numeric_limits<double>::lowest();
        double ymin = numeric_limits<double>::max();
        double ymax = numeric_limits<double>::lowest();
        for (auto p : coords[angle]) {
            if (get<0>(p) < xmin)
                xmin = get<0>(p);
            if (get<0>(p) > xmax)
                xmax = get<0>(p);
            if (get<1>(p) < ymin)
                ymin = get<1>(p);
            if (get<1>(p) > ymax)
                ymax = get<1>(p);
        }

        // calculate dimensions
        int x = int(ceil((xmax - xmin) / resolution));
        int y = int(ceil((ymax - ymin) / resolution));

        // fix minimal resolution
        if (xmax - xmin < resolution || ymax - ymin < resolution ) {
            resolution = min(xmax - xmin, ymax - ymin);
            x = int(ceil((xmax - xmin) / resolution));
            y = int(ceil((ymax - ymin) / resolution));
            ROS_WARN("Changing resolution to minimal possible %.2f, grid size %dx%d!", resolution, x, y);
        }

        // warn about large grids
        if (x*y > cell_warn)
            ROS_WARN("Given coordinates seem wrong, grid map extremley large: %d cells!", x*y);

        // generate grid map data
        vector<int8_t> data;
        for (int i=0; i<y; ++i) { // row major order
            for (int j=0; j<x; ++j) {
                // check if center of cell is outside of area
                if (out_of_bounds(make_pair(j * resolution + xmin + resolution / 2.0, i * resolution + ymin + resolution / 2.0), angle))
                    data.push_back(CELL_OCCUPIED); // occupied

                // inside area
                else
                    data.push_back(CELL_FREE); // free
            }
        }
        gridmap.data = data;

        // set map header
        gridmap.header.stamp = Time::now();
        gridmap.header.frame_id = "map";

        // set map meta data
        gridmap.info.map_load_time == Time::now();
        gridmap.info.resolution = resolution;
        gridmap.info.width = x;
        gridmap.info.height = y;

        // position of cell (0,0)
        gridmap.info.origin.position.x = xmin;
        gridmap.info.origin.position.y = ymin;

        gridmaps[angle][resolution] = gridmap;
    }

    // return existing or newly generated grid map
    if (gridmaps.count(angle) > 0 && gridmaps[angle].count(resolution) > 0)
        return gridmaps[angle][resolution];

    // return empty grid map
    ROS_WARN("Providing empty map!");
    return gridmap;
}

void area::global_to_local ()
{
    // keep global coordinates
    coords_global = coords[0];

    // convert given area to local coordinates
    cpswarm_msgs::FixToPose f2p;
    set<tuple<double,double,double>> local;
    for (auto c : coords[0]) {
        f2p.request.fix.longitude = get<0>(c);
        f2p.request.fix.latitude = get<1>(c);
        f2p.request.fix.altitude = get<2>(c);
        if (fix_to_pose_client.call(f2p)) {
            local.emplace(f2p.response.pose.pose.position.x, f2p.response.pose.pose.position.y, get<2>(c)); // altitude of area is always above ground level
        }
        else
            ROS_FATAL("AREA_PROV - Failed to convert area bounds to local coordinates");
    }
    coords[0] = local;

    sort_coords();
}

void area::set_origin ()
{
    if (global) {
        cpswarm_msgs::FixToPose f2p;

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

    else {
        // read origin from parameters
        double x,y,z;
        nh.param(this_node::getName() + "/x", x, 0.0);
        nh.param(this_node::getName() + "/y", y, 0.0);
        nh.param(this_node::getName() + "/z", z, 0.0);
        origin.x = x;
        origin.y = y;
        origin.z = z;
    }
}

vector<geometry_msgs::Point> area::set2vector (set<tuple<double,double,double>> set)
{
    vector<geometry_msgs::Point> vector;
    for (auto tuple : set) {
        vector.push_back(tuple2point(tuple));
    }
    return vector;
}

void area::sort_coords ()
{
    coords_sorted.clear();

    // coordinates of different rotations
    for (auto c : coords) {
        // rotation of coordinates
        double rot = get<0>(c);

        // compute centroid / barycenter
        pair<double,double> center;
        for (auto cc : get<1>(c)) {
            center.first += get<0>(cc);
            center.second += get<1>(cc);
        }
        center.first /= get<1>(c).size();
        center.second /= get<1>(c).size();

        // sort by angle around center
        double angle;
        for (auto cc : get<1>(c)) {
            angle = atan2(get<1>(cc)-center.second, get<0>(cc)-center.first);
            coords_sorted[rot][angle] = cc;
        }
    }
}

set<tuple<double,double,double>> area::vector2set (vector<geometry_msgs::Point> vector)
{
    set<tuple<double,double,double>> set;
    for (auto point : vector) {
        set.insert(point2tuple(point));
    }
    return set;
}

bool area::is_left (pair<double,double> p0, pair<double,double> p1, pair<double,double> p2)
{
    // >0 for p2 left of the line through p0 and p1
    // =0 for p2 on the line
    // <0 for p2 right of the line
    return ((p1.first - p0.first) * (p2.second - p0.second) - (p2.first -  p0.first) * (p1.second - p0.second)) > 0;
}

bool area::is_right (pair<double,double> p0, pair<double,double> p1, pair<double,double> p2)
{
    // >0 for p2 left of the line through p0 and p1
    // =0 for p2 on the line
    // <0 for p2 right of the line
    return ((p1.first - p0.first) * (p2.second - p0.second) - (p2.first -  p0.first) * (p1.second - p0.second)) < 0;
}

bool area::on_bound (pair<double,double> pos, double angle)
{
    double d01x, d01y, d01, d02x, d02y, d02, d12x, d12y, d12;

    // loop through all edges of the polygon
    map<double, tuple<double,double,double>>::iterator ne;
    for (map<double, tuple<double,double,double>>::iterator it = coords_sorted[angle].begin(); it != coords_sorted[angle].end(); ++it) {
        // next element in set
        ne = next(it);
        if (ne == coords_sorted[angle].end())
            ne = coords_sorted[angle].begin();

        // distances between points
        d01x = get<0>(ne->second) - get<0>(it->second);
        d01y = get<1>(ne->second) - get<1>(it->second);
        d01 = hypot(d01x, d01y);
        d02x = pos.first - get<0>(it->second);
        d02y = pos.second - get<1>(it->second);
        d02 = hypot(d02x, d02y);
        d12x = pos.first - get<0>(ne->second);
        d12y = pos.second - get<1>(ne->second);
        d12 = hypot(d12x, d12y);

        // close enough to boundary
        if (abs(d02 + d12 - d01) < 0.0001)
            return true;
    }

    // not on boundary
    return false;
}

bool area::out_of_bounds (pair<double,double> pos, double angle)
{
    // points on boundary are not out of bounds
    if (on_bound(pos, angle))
        return false;

    // the winding number counter, i.e., how often a ray from the point to the right crosses the boundary
    int wn = 0;

    // loop through all edges of the polygon
    map<double, tuple<double,double,double>>::iterator ne;
    for (map<double, tuple<double,double,double>>::iterator it = coords_sorted[angle].begin(); it != coords_sorted[angle].end(); ++it) {
        // next element in set
        ne = next(it);
        if (ne == coords_sorted[angle].end())
            ne = coords_sorted[angle].begin();

        // ray crosses upward edge
        if (get<1>(it->second) <= pos.second && pos.second < get<1>(ne->second) && is_left(make_pair(get<0>(it->second), get<1>(it->second)), make_pair(get<0>(ne->second), get<1>(ne->second)), pos))
            ++wn;

        // ray crosses downward edge
        else if (get<1>(ne->second) <= pos.second && pos.second < get<1>(it->second) && is_right(make_pair(get<0>(it->second), get<1>(it->second)), make_pair(get<0>(ne->second), get<1>(ne->second)), pos))
            --wn;
    }

    // pose is outside
    if (wn == 0)
        return true;

    // pose is inside
    return false;
}

geometry_msgs::Point area::tuple2point (tuple<double,double,double> tuple)
{
    geometry_msgs::Point point;
    point.x = get<0>(tuple);
    point.y = get<1>(tuple);
    point.z = get<2>(tuple);
    return point;
}

pair<double,double> area::point2pair (geometry_msgs::Point point)
{
    pair<double,double> pair{point.x, point.y};
    return pair;
}

tuple<double,double,double> area::point2tuple (geometry_msgs::Point point)
{
    tuple<double,double,double> tuple{point.x, point.y, point.z};
    return tuple;
}

double area::rotate ()
{
    // map has already been rotated
    if (rotation != 0)
        return rotation;

    // bottom most point
    map<double, tuple<double,double,double>>::iterator bot = coords_sorted[0].begin();
    for (auto it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it)
        if (get<1>(it->second) < get<1>(bot->second))
            bot = it;

    // previous point
    map<double, tuple<double,double,double>>::iterator pr;
    if (bot == coords_sorted[0].begin())
        pr = prev(coords_sorted[0].end());
    else
        pr = prev(bot);

    // next point
    map<double, tuple<double,double,double>>::iterator ne = next(bot);
    if (ne == coords_sorted[0].end())
        ne = coords_sorted[0].begin();

    // calculate rotation
    if (get<1>(pr->second) < get<1>(ne->second) || get<1>(pr->second) == get<1>(ne->second) && get<1>(pr->second) != get<1>(bot->second)) // prefer counter-clockwise rotation in case of tie
        rotation = -atan2(get<1>(bot->second) - get<1>(pr->second), get<0>(bot->second) - get<0>(pr->second));
    else if (get<1>(pr->second) > get<1>(ne->second))
        rotation = -atan2(get<1>(ne->second) - get<1>(bot->second), get<0>(ne->second) - get<0>(bot->second));
    else
        return 0;

    ROS_DEBUG("Rotate map by %.2f...", rotation);

    // rotate coordinates
    if (coords.count(rotation) == 0) {
        stringstream css;
        for (auto c : coords[0]) {
            tuple<double,double,double> rot;
            get<0>(rot) = get<0>(c) * cos(rotation) - get<1>(c) * sin(rotation);
            get<1>(rot) = get<0>(c) * sin(rotation) + get<1>(c) * cos(rotation);
            get<2>(rot) = get<2>(c);
            coords[rotation].insert(rot);
            css << "(" << get<0>(rot) << "," << get<1>(rot) << "," << get<2>(rot) << ") ";
        }
        ROS_DEBUG("Rotated coordinates: %s", css.str().c_str());
    }

    // rotate sorted coordinates
    if (coords_sorted.count(rotation) == 0)
        for (auto c : coords_sorted[0]) {
            tuple<double,double,double> rot;
            get<0>(rot) = get<0>(c.second) * cos(rotation) - get<1>(c.second) * sin(rotation);
            get<1>(rot) = get<0>(c.second) * sin(rotation) + get<1>(c.second) * cos(rotation);
            get<2>(rot) = get<2>(c.second);
            coords_sorted[rotation][c.first] = rot;
        }

    return rotation;
}

geometry_msgs::Vector3 area::translate (nav_msgs::OccupancyGrid& map)
{
    // compute required translation
    geometry_msgs::Vector3 translation;
    translation.x = round(map.info.origin.position.x) - map.info.origin.position.x;
    translation.y = round(map.info.origin.position.y) - map.info.origin.position.y;
    ROS_DEBUG("Translate map by (%.2f,%.2f)...", translation.x, translation.y);

    // translate origin
    map.info.origin.position.x += translation.x;
    map.info.origin.position.y += translation.y;

    // update meta data
    map.info.map_load_time = Time::now();

    return translation;
}
