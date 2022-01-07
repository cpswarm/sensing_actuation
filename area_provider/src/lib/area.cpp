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
    res.points = set2vector(coords[0]);
    return true;
}

bool area::get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    double area = 0;

    // compute centroid
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted[0].end())
            ne = coords_sorted[0].begin();

        // sum up coordinates
        res.point.x += (it->second.first + ne->second.first) * (it->second.first * ne->second.second - ne->second.first * it->second.second);
        res.point.y += (it->second.second + ne->second.second) * (it->second.first * ne->second.second - ne->second.first * it->second.second);

        // sum up area
        area += it->second.first * ne->second.second - ne->second.first * it->second.second;
    }

    // normalize area
    area /= 2;

    // normalize coordinates
    res.point.x /= 6 * area;
    res.point.y /= 6 * area;

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
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted[0].end())
            ne = coords_sorted[0].begin();


        // coordinates of two neighboring polygon points
        geometry_msgs::Point p1 = pair2point(it->second);
        geometry_msgs::Point p2 = pair2point(ne->second);

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
        ss << "(" << c.second.first << "," << c.second.second << ") ";
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
        double xmax = numeric_limits<double>::min();
        double ymin = numeric_limits<double>::max();
        double ymax = numeric_limits<double>::min();
        for (auto p : coords[angle]) {
            if (p.first < xmin)
                xmin = p.first;
            if (p.first > xmax)
                xmax = p.first;
            if (p.second < ymin)
                ymin = p.second;
            if (p.second > ymax)
                ymax = p.second;
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
        cpswarm_msgs::OutOfBounds::Request req;
        cpswarm_msgs::OutOfBounds::Response res;
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
    cpswarm_msgs::FixToPose f2p;

    // convert given area to local coordinates
    set<pair<double,double>> local;
    for (auto c : coords[0]) {
        f2p.request.fix.longitude = c.first;
        f2p.request.fix.latitude = c.second;
        if (fix_to_pose_client.call(f2p)) {
            local.emplace(f2p.response.pose.pose.position.x, f2p.response.pose.pose.position.y);
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
        double x,y;
        nh.param(this_node::getName() + "/x", x, 0.0);
        nh.param(this_node::getName() + "/y", y, 0.0);
        origin.x = x;
        origin.y = y;
    }
}

vector<geometry_msgs::Point> area::set2vector (set<pair<double,double>> set)
{
    vector<geometry_msgs::Point> vector;
    for (auto pair : set) {
        vector.push_back(pair2point(pair));
    }
    return vector;
}

void area::sort_coords ()
{
    coords_sorted.clear();

    // coordinates of different rotations
    for (auto c : coords) {
        // rotation of coordinates
        double rot = c.first;

        // compute centroid / barycenter
        pair<double,double> center;
        for (auto cc : c.second) {
            center.first += cc.first;
            center.second += cc.second;
        }
        center.first /= c.second.size();
        center.second /= c.second.size();

        // sort by angle around center
        double angle;
        for (auto cc : c.second) {
            angle = atan2(cc.second-center.second, cc.first-center.first);
            coords_sorted[rot][angle] = cc;
        }
    }
}

set<pair<double,double>> area::vector2set (vector<geometry_msgs::Point> vector)
{
    set<pair<double,double>> set;
    for (auto point : vector) {
        set.insert(point2pair(point));
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
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted[angle].begin(); it != coords_sorted[angle].end(); ++it) {
        // next element in set
        ne = next(it);
        if (ne == coords_sorted[angle].end())
            ne = coords_sorted[angle].begin();

        // distances between points
        d01x = ne->second.first - it->second.first;
        d01y = ne->second.second - it->second.second;
        d01 = hypot(d01x, d01y);
        d02x = pos.first - it->second.first;
        d02y = pos.second - it->second.second;
        d02 = hypot(d02x, d02y);
        d12x = pos.first - ne->second.first;
        d12y = pos.second - ne->second.second;
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
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted[angle].begin(); it != coords_sorted[angle].end(); ++it) {
        // next element in set
        ne = next(it);
        if (ne == coords_sorted[angle].end())
            ne = coords_sorted[angle].begin();

        // ray crosses upward edge
        if (it->second.second <= pos.second && pos.second < ne->second.second && is_left(it->second, ne->second, pos))
            ++wn;

        // ray crosses downward edge
        else if (ne->second.second <= pos.second && pos.second < it->second.second && is_right(it->second, ne->second, pos))
            --wn;
    }

    // pose is outside
    if (wn == 0)
        return true;

    // pose is inside
    return false;
}

geometry_msgs::Point area::pair2point (pair<double,double> pair)
{
    geometry_msgs::Point point;
    point.x = pair.first;
    point.y = pair.second;
    return point;
}

pair<double,double> area::point2pair (geometry_msgs::Point point)
{
    pair<double,double> pair;
    pair.first = point.x;
    pair.second = point.y;
    return pair;
}

double area::rotate ()
{
    // map has already been rotated
    if (rotation != 0)
        return rotation;

    // bottom most point
    map<double, pair<double,double>>::iterator bot = coords_sorted[0].begin();
    for (auto it = coords_sorted[0].begin(); it != coords_sorted[0].end(); ++it)
        if (it->second.second < bot->second.second)
            bot = it;

    // previous point
    map<double, pair<double,double>>::iterator pr;
    if (bot == coords_sorted[0].begin())
        pr = prev(coords_sorted[0].end());
    else
        pr = prev(bot);

    // next point
    map<double, pair<double,double>>::iterator ne = next(bot);
    if (ne == coords_sorted[0].end())
        ne = coords_sorted[0].begin();

    // calculate rotation
    if (pr->second.second < ne->second.second || pr->second.second == ne->second.second && pr->second.second != bot->second.second) // prefer counter-clockwise rotation in case of tie
        rotation = -atan2(bot->second.second - pr->second.second, bot->second.first - pr->second.first);
    else if (pr->second.second > ne->second.second)
        rotation = -atan2(ne->second.second - bot->second.second, ne->second.first - bot->second.first);
    else
        return 0;

    ROS_DEBUG("Rotate map by %.2f...", rotation);

    // rotate coordinates
    pair<double,double> rot;
    if (coords.count(rotation) == 0) {
        stringstream css;
        for (auto c : coords[0]) {
            rot.first = c.first * cos(rotation) - c.second * sin(rotation);
            rot.second = c.first * sin(rotation) + c.second * cos(rotation);
            coords[rotation].insert(rot);
            css << "(" << rot.first << "," << rot.second << ") ";
        }
        ROS_DEBUG("Rotated coordinates: %s", css.str().c_str());
    }

    // rotate sorted coordinates
    if (coords_sorted.count(rotation) == 0)
        for (auto c : coords_sorted[0]) {
            rot.first = c.second.first * cos(rotation) - c.second.second * sin(rotation);
            rot.second = c.second.first * sin(rotation) + c.second.second * cos(rotation);
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
