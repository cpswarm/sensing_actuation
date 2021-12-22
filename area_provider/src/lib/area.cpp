#include "lib/area.h"

area::area ()
{
    // read parameters
    nh.param(this_node::getName() + "/cell_warn", cell_warn, 1000);
    nh.param(this_node::getName() + "/resolution", resolution, 1.0);
    nh.param(this_node::getName() + "/create_map", create_map, false);

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

    // map has not yet been modified
    map_is_rotated = false;
    map_downsampled_resolution = -1;
}

bool area::get_area (cpswarm_msgs::GetPoints::Request &req, cpswarm_msgs::GetPoints::Response &res)
{
    res.points = set2vector(coords);
    return true;
}

bool area::get_center (cpswarm_msgs::GetPoint::Request &req, cpswarm_msgs::GetPoint::Response &res)
{
    double area = 0;

    // compute centroid
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted.begin(); it != coords_sorted.end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted.end())
            ne = coords_sorted.begin();


        // coordinates of two neighboring polygon points
        geometry_msgs::Point p1 = pair2point(it->second);
        geometry_msgs::Point p2 = pair2point(ne->second);

        // sum up coordinates
        res.point.x += (p1.x + p2.x) * (p1.x * p2.y - p2.x * p1.y);
        res.point.y += (p1.y + p2.y) * (p1.x * p2.y - p2.x * p1.y);

        // sum up area
        area += p1.x * p2.y - p2.x * p1.y;
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

    // find minimal distance to any area bound
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted.begin(); it != coords_sorted.end(); ++it) {
        // next element
        ne = next(it);
        if (ne == coords_sorted.end())
            ne = coords_sorted.begin();


        // coordinates of two neighboring polygon points
        geometry_msgs::Point p1 = pair2point(it->second);
        geometry_msgs::Point p2 = pair2point(ne->second);

        // minimal distance to line connecting the two points
        geometry_msgs::Point p02;
        p02.x = p2.x - p0.x;
        p02.y = p2.y - p0.y;
        geometry_msgs::Point p12;
        p12.x = p2.x - p1.x;
        p12.y = p2.y - p1.y;
        geometry_msgs::Point p10;
        p10.x = p0.x - p1.x;
        p10.y = p0.y - p1.y;
        double dot = p12.x * p10.x + p12.y * p10.y;
        double dis = hypot(p12.x, p12.y);
        double r = dot / dis / dis;

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
            dist = sqrt((p10.x*p10.x + p10.y*p10.y) - pow(r * hypot(p12.x, p12.y), 2));
            double d1c = sqrt((p10.x*p10.x + p10.y*p10.y) - dist*dist); // distance from p1 to closest point
            double d12 = hypot(p12.x, p12.y);
            closest.x = p1.x + p12.x / d12 * d1c;
            closest.y = p1.y + p12.y / d12 * d1c;
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
    if (map_exists == false && create_map == false)
        ROS_WARN("Providing empty map!");

    // get/create original map
    res.map = get_gridmap();

    // rotate map if requested
    if (req.rotate)
        res.rotation = rotate(res.map);

    // downsample resolution if requested
    if (req.resolution > 0 && req.resolution > res.map.info.resolution)
        downsample(res.map, req.resolution);

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
    // request's position
    pair<double,double> pos = point2pair(req.pose.position);

    // the winding number counter, i.e., how often a ray from the point to the right crosses the boundary
    int wn = 0;

    // loop through all edges of the polygon
    map<double, pair<double,double>>::iterator ne;
    for (map<double, pair<double,double>>::iterator it = coords_sorted.begin(); it != coords_sorted.end(); ++it) {
        // next element in set
        ne = next(it);
        if (ne == coords_sorted.end())
            ne = coords_sorted.begin();

        // ray crosses upward edge
        if (it->second.second <= req.pose.position.y && ne->second.second  > req.pose.position.y && is_left(it->second, ne->second, pos))
            ++wn;

        // ray crosses downward edge
        else if (it->second.second > req.pose.position.y && ne->second.second  <= req.pose.position.y && is_right(it->second, ne->second, pos))
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

void area::downsample (nav_msgs::OccupancyGrid& map, double resolution)
{
    // do not increase resolution, use cached map
    if (resolution <= map_downsampled_resolution) {
        map = map_downsampled;
        return;
    }

    // reduction factor
    int f = int(round(resolution / map.info.resolution));

    ROS_DEBUG("Downsample map by %d...", f);

    // downsample map data
    vector<signed char> lr;
    for (int i=0; i+f<=map.info.height; i+=f) {
        for (int j=0; j+f<=map.info.width; j+=f) {
            // count frequency of map data values
            vector<unsigned int> values(256, 0);
            for (int m=i; m<i+f; ++m) {
                for (int n=j; n<j+f; ++n) {
                    values[map.data[m*map.info.width + n]]++;
                }
            }

            // choose value with highest frequency
            unsigned char value = 0;
            unsigned int freq = 0;
            for (int k=0; k<values.size(); ++k) {
                if (values[k] > freq) {
                    value = k;
                    freq = values[k];
                }
            }

            // push back most seen value
            lr.push_back(value);
        }
    }
    map.data = lr;

    // update meta data
    map.info.map_load_time = Time::now();
    map.info.resolution = double(f) * map.info.resolution;
    map.info.width = int(floor(double(map.info.width) / double(f)));
    map.info.height = int(floor(double(map.info.height) / double(f)));

    // remove rows with obstacles only
    for (int i=0; i<map.info.height; ++i) {
        // count number of occupied cells in a row
        int obst = 0;
        for (int j=0; j<map.info.width; ++j) {
            if (map.data[i*map.info.width + j] == CELL_OCCUPIED) {
                ++obst;
            }
        }

        // remove row
        if (obst == map.info.width) {
            // delete grid cells
            map.data.erase(map.data.begin() + i*map.info.width, map.data.begin() + (i+1)*map.info.width);

            // update meta data
            map.info.map_load_time = Time::now();
            --map.info.height;
            map.info.origin.position.y += map.info.resolution;

            // stay in current row
            --i;
        }
    }

    // cache downsampled map
    map_downsampled_resolution = resolution;
    map_downsampled = map;
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
            if (p.first < xmin)
                xmin = p.first;
            if (p.first > xmax)
                xmax = p.first;
            if (p.second < ymin)
                ymin = p.second;
            if (p.second > ymax)
                ymax = p.second;
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

        map_exists = true;
    }

    return gridmap;
}

void area::global_to_local ()
{
    cpswarm_msgs::FixToPose f2p;

    // convert given area to local coordinates
    set<pair<double,double>> local;
    for (auto c : coords) {
        f2p.request.fix.longitude = c.first;
        f2p.request.fix.latitude = c.second;
        if (fix_to_pose_client.call(f2p)) {
            local.emplace(f2p.response.pose.pose.position.x, f2p.response.pose.pose.position.y);
        }
        else
            ROS_FATAL("AREA_PROV - Failed to convert area bounds to local coordinates");
    }
    coords = local;

    sort_coords();
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

double area::rotate (nav_msgs::OccupancyGrid& map)
{
    // use cached map
    if (map_is_rotated)
        map = map_rotated;

    // determine angle to rotate by
    double angle;

    // determine extreme coordinates
    pair<double,double> pl, pb, pr;
    pl.first = numeric_limits<double>::max();
    pb.second = numeric_limits<double>::max();
    pr.first = numeric_limits<double>::min();
    for (auto p : coords) {
        // left most point
        if (p.first < pl.first || (p.first == pl.first && p.second < pl.second))
            pl = p;

        // bottom most point
        if (p.second < pb.second)
            pb = p;

        // right most point
        if (p.first > pr.first || (p.first == pr.first && p.second < pr.second))
            pr = p;
    }

    // no rotation required
    if ((pl.first == pb.first && pl.second == pb.second) || (pr.first == pb.first && pr.second == pb.second))
        return 0;

    // rotate clockwise
    if (pr.second < pl.second)
        angle = -atan2(pr.second - pb.second, pr.first - pb.first);

    // rotate counter clockwise
    else
        angle = -atan2(pb.second - pl.second, pb.first - pl.first);

    ROS_DEBUG("Rotate map by %.2f...", angle);

    // rotate origin
    geometry_msgs::Pose origin_new;
    origin_new.position.x = map.info.origin.position.x*cos(angle) - map.info.origin.position.y*sin(angle);
    origin_new.position.y = map.info.origin.position.x*sin(angle) + map.info.origin.position.y*cos(angle);

    // create empty rotated map extra large
    vector<vector<signed char>> rt;
    for (int i=0; i<2*map.info.height; ++i) {
        vector<signed char> row(2*map.info.width, CELL_OCCUPIED);
        rt.push_back(row);
    }

    // rotate map
    int i_new, j_new, width_new=0, height_new=0;
    double x, y, x_new, y_new;
    for (int i=0; i<map.info.height; ++i) {
        for (int j=0; j<map.info.width; ++j) {
            // rotate coordinates
            x = double(j) * map.info.resolution + map.info.origin.position.x;
            y = double(i) * map.info.resolution + map.info.origin.position.y;
            x_new = x*cos(angle) - y*sin(angle);
            y_new = x*sin(angle) + y*cos(angle);
            j_new = int(round((x_new - origin_new.position.x) / map.info.resolution));
            i_new = int(round((y_new - origin_new.position.y) / map.info.resolution));

            // skip negative indexes
            if (i_new >= rt.size()) {
                continue;
            }
            if (j_new >= rt[i_new].size()) {
                continue;
            }

            // assign grid cell value
            rt[i_new][j_new] = map.data[i*map.info.width + j];

            // measure maximum required size
            if (rt[i_new][j_new] == 0) {
                if (i_new > height_new)
                    height_new = i_new;
                if (j_new > width_new)
                    width_new = j_new;
            }
        }
    }

    // truncate rotated map
    rt.resize(height_new);
    for (int i=0; i<rt.size(); ++i)
        rt[i].resize(width_new);

    // collapse map to one dimensional vector
    vector<signed char> rt_col;
    for (int i=0; i<rt.size(); ++i) {
        for (int j=0; j<rt[i].size(); ++j) {
            rt_col.push_back(rt[i][j]);
        }
    }

    // assign map data
    map.data = rt_col;

    // update meta data
    map.info.map_load_time = Time::now();
    map.info.width = width_new;
    map.info.height= height_new;
    map.info.origin = origin_new;

    // cache rotated map
    map_is_rotated = true;
    map_rotated = map;

    return angle;
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

    // compute centroid / barycenter
    pair<double,double> center;
    for (auto c : coords) {
        center.first += c.first;
        center.second += c.second;
    }
    center.first /= coords.size();
    center.second /= coords.size();

    // sort by angle around center
    double angle;
    for (auto c : coords) {
        angle = atan2(c.second-center.second, c.first-center.first);
        coords_sorted[angle] = c;
    }
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
