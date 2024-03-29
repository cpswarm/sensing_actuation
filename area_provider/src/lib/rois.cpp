#include "lib/rois.h"

rois::rois ()
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/duplicates", duplicates, false);
    nh.param(this_node::getName() + "/visualize", visualize, false);

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

    // get rois from incoming event messages
    roi_subscriber = nh.subscribe("bridge/events/rois/roi", queue_size, &rois::roi_callback, this);
    assignment_subscriber = nh.subscribe("bridge/events/rois/assignment", queue_size, &rois::roi_callback, this);

    // get roi state from incoming event messages
    state_subscriber = nh.subscribe("bridge/events/rois/state", queue_size, &rois::state_callback, this);

    // publish new rois
    nh.param(this_node::getName() + "/publish", publish, false);
    if (publish)
        roi_publisher = nh.advertise<cpswarm_msgs::PointArrayEvent>("rois/roi", queue_size);
        Duration(1).sleep(); // give subscribers some time to connect

    // import rois from files
    from_file();
}

bool rois::get_all (cpswarm_msgs::GetMultiPoints::Request &req, cpswarm_msgs::GetMultiPoints::Response &res)
{
    // collection of all roi coordinates
    vector<vector<geometry_msgs::Point>> coords;

    // maximum number of coordinates of any roi
    int max_num_coords = 0;

    // iterate all rois
    for (auto roi : regions) {
        // store maximum number of coordinates
        if (roi.coords[0].size() > max_num_coords)
            max_num_coords = roi.coords[0].size();

        // append coordinates
        coords.push_back(roi.set2vector(roi.coords[0]));
    }

    // pad all rois with empty coordinates at the end
    geometry_msgs::Point empty;
    for (auto& roi : coords) {
        for (int i=roi.size(); i<max_num_coords; ++i) {
            roi.push_back(empty);
        }
    }

    // flatten coords
    vector<geometry_msgs::Point> coords_flat;
    std_msgs::MultiArrayLayout layout;
    flatten_vector(coords, coords_flat, layout);

    // create service response
    res.layout = layout;
    res.points = coords_flat;

    return true;
}

bool rois::get_closest (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res)
{
    string closest_str;
    cpswarm_msgs::GetDist::Response closest;
    cpswarm_msgs::GetDist::Response response;

    for (auto roi : regions) {
        closest_str = roi.to_string();
        ROS_DEBUG("Calculate distance to ROI %s", closest_str.c_str());

        if (roi.get_distance(req, response)) {
            if (closest.closest_line.size() <= 0 || response.distance < closest.distance) {
                closest = response;
                closest.coords = roi.set2vector(roi.coords[0]);
            }
        }
        else
            ROS_ERROR("Failed to retrieve distance for ROI %s!", closest_str.c_str());
    }

    ROS_INFO("Closest ROI %s", closest_str.c_str());

    res = closest;

    return true;
}

bool rois::get_distance (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res)
{
    cpswarm_msgs::GetDist::Response closest;
    cpswarm_msgs::GetDist::Response response;

    // find roi according to given coordinates
    for (auto roi : regions) {
        // forward request
        if (roi.coords[0] == roi.vector2set(req.coords)) {
            roi.get_distance(req, res);
            return true;
        }
    }

    // create string for warning
    stringstream coords_ss;
    for (auto p : req.coords)
        coords_ss << "(" << p.x << "," << p.y << "," << p.z << ") ";
    ROS_WARN("Could not find ROI %s", coords_ss.str().c_str());

    return false;
}

bool rois::get_map (cpswarm_msgs::GetMap::Request &req, cpswarm_msgs::GetMap::Response &res)
{
    // find roi according to given coordinates
    for (auto roi : regions) {
        // forward request
        if (roi.coords[0] == roi.vector2set(req.coords)) {
            roi.get_map(req, res);
            return true;
        }
    }

    // create string for warning
    stringstream coords_ss;
    for (auto p : req.coords)
        coords_ss << "(" << p.x << "," << p.y << "," << p.z << ") ";
    ROS_WARN("Could not find ROI %s", coords_ss.str().c_str());

    return false;
}

bool rois::get_todo (cpswarm_msgs::GetMultiPoints::Request &req, cpswarm_msgs::GetMultiPoints::Response &res)
{
    // collection of roi coordinates
    vector<vector<geometry_msgs::Point>> coords;

    // maximum number of coordinates of any roi
    int max_num_coords = 0;

    // iterate all rois
    for (auto roi : regions) {
        // only consider rois in todo state
        if (roi.state == ROI_TODO) {
            // store maximum number of coordinates
            if (roi.coords[0].size() > max_num_coords)
                max_num_coords = roi.coords[0].size();

            // append coordinates
            coords.push_back(roi.set2vector(roi.coords[0]));
        }
    }

    // pad all rois with empty coordinates at the end
    geometry_msgs::Point empty;
    for (auto& roi : coords) {
        for (int i=roi.size(); i<max_num_coords; ++i) {
            roi.push_back(empty);
        }
    }

    // flatten coords
    vector<geometry_msgs::Point> coords_flat;
    std_msgs::MultiArrayLayout layout;
    flatten_vector(coords, coords_flat, layout);

    // create service response
    res.layout = layout;
    res.points = coords_flat;

    return true;
}

bool rois::reload (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // reset old rois
    if (req.data) {
        regions.clear();
        map_publisher.clear();
    }

    // import rois from files
    from_file();

    res.success = true;

    return true;
}

bool rois::set_state (cpswarm_msgs::SetRoiState::Request &req, cpswarm_msgs::SetRoiState::Response &res)
{
    // convert coordinates to set
    set<tuple<double,double,double>> coords;
    for (auto point : req.coords)
        coords.emplace(point.x, point.y, point.z);

    res.success = set_state(coords, (roi_state_t)req.state);

    return true;
}

void rois::add_roi (vector<double> x, vector<double> y, vector<double> z)
{
    // equal number of x, y, and z coordinates required
    if (x.size() != y.size() || x.size() != z.size() || y.size() != z.size()) {
        ROS_ERROR("Cannot add ROI, number of x and y coordinates do not match (%lu != %lu != %lu)", x.size(), y.size(), z.size());
    }

    // not enough coordinates
    else if (x.size() < 3) {
        ROS_ERROR("Cannot add ROI, not enough coordinates: %lu", x.size());
    }

    else {
        // add roi
        roi roi(x, y, z);

        // check if roi already exists
        if (duplicates == false && exists(roi)) {
            ROS_INFO("Skipped ROI %s, already existing", roi.to_string().c_str());
            return;
        }

        ROS_INFO("Added ROI %s", roi.to_string().c_str());
        regions.insert(roi);

        // publish roi
        if (publish) {
            cpswarm_msgs::PointArrayEvent event;
            event.header.stamp = Time::now();
            event.swarmio.name = "roi";
            tuple<vector<double>, vector<double>, vector<double>> coords = roi.get_global();
            event.x = get<0>(coords);
            event.y = get<1>(coords);
            event.z = get<2>(coords);
            roi_publisher.publish(event);
        }

        if (visualize) {
            int queue_size;
            nh.param(this_node::getName() + "/queue_size", queue_size, 1);
            map_publisher.push_back(nh.advertise<nav_msgs::OccupancyGrid>("rois/map_" + to_string(map_publisher.size()), queue_size, true));
            map_publisher.back().publish(roi.get_gridmap());
        }
    }
}

bool rois::exists (roi roi)
{
    // look through all regions
    for (auto r : regions) {
        if (r == roi)
            return true;
    }

    // no matching region found
    return false;
}

void rois::flatten_vector (vector<vector<geometry_msgs::Point>> vector_2d, vector<geometry_msgs::Point>& vector_flat, std_msgs::MultiArrayLayout& layout)
{
    // flatten roi coordinates
    for (auto roi : vector_2d) {
        vector_flat.insert(vector_flat.end(), roi.begin(), roi.end());
    }

    // define multi-array layout
    vector<std_msgs::MultiArrayDimension> dim;

    std_msgs::MultiArrayDimension dim0;
    dim0.label = "roi";
    dim0.size = vector_2d.size();
    if (vector_2d.size() > 0)
        dim0.stride = vector_2d.size() * vector_2d[0].size();
    else
        dim0.stride = 0;
    dim.push_back(dim0);

    std_msgs::MultiArrayDimension dim1;
    dim1.label = "coords";
    if (vector_2d.size() > 0){
        dim1.size = vector_2d[0].size();
        dim1.stride = vector_2d[0].size();
    }
    else {
        dim1.size = 0;
        dim1.stride = 0;
    }
    dim.push_back(dim1);

    layout.dim = dim;
    layout.data_offset = 0;
}

void rois::from_file ()
{
    // directory with roi coordinate files
    string roi_dir_str = "";
    nh.param(this_node::getName() + "/roi_dir", roi_dir_str, roi_dir_str);

    // no roi files
    if (roi_dir_str.empty()) {
        ROS_INFO("No directory for ROI files given!");
        return;
    }

    // iterate all files in given directory
    try {
        // path relative to this ros package
        filesystem::path package_dir = package::getPath("area_provider");
        filesystem::path roi_dir = roi_dir_str;
        filesystem::path roi_dir_abs = package_dir / roi_dir;

        // iterate all files
        for(auto const& roi_file : filesystem::directory_iterator{roi_dir_abs}) {
            string roi_file_name = roi_file.path().filename().string();

            try {
                // convert file contents to json object
                ifstream roi_stream(roi_file.path().string());
                json roi_json;
                roi_stream >> roi_json;

                // qgc plan file
                if (roi_json.contains("fileType") and roi_json["fileType"] == "Plan") {
                    // no (or invalid) mission specified
                    if (roi_json.contains("mission") == false or roi_json["mission"].contains("items") == false or roi_json["mission"]["items"].size() < 3) {
                        ROS_ERROR("Skipping file %s, invalid syntax: no mission specified!", roi_file_name.c_str());
                        continue;
                    }

                    // extract coordinates
                    ROS_DEBUG("Extract ROI coordinates from %s...", roi_file_name.c_str());
                    vector<double> x;
                    vector<double> y;
                    vector<double> z;
                    for (auto c : roi_json["mission"]["items"]) {
                        if (c["params"].size() < 7) {
                            ROS_ERROR("Skipping file %s, invalid syntax: number of params must be 7 for each mission item!", roi_file_name.c_str());
                            break;
                        }
                        if (c["command"] != 16) {
                            ROS_DEBUG("%s: Skipping item (%f,%f), not a waypoint!", roi_file_name.c_str(), double(c["params"][4]), double(c["params"][5]));
                            continue;
                        }
                        x.push_back(c["params"][5]);
                        y.push_back(c["params"][4]);
                        z.push_back(c["params"][6]);
                    }

                    // create roi object
                    ROS_INFO("Add ROI from file %s...", roi_file_name.c_str());
                    add_roi(x, y, z);
                }

                // geo json
                else if (roi_json.contains("type") and roi_json["type"] == "FeatureCollection") {
                    // no (or invalid) mission specified
                    if (roi_json.contains("features") == false || roi_json["features"].size() < 1) {
                        ROS_ERROR("Skipping file %s, invalid syntax: no mission specified!", roi_file_name.c_str());
                        continue;
                    }

                    // add roi for each feature
                    ROS_DEBUG("Extract ROI coordinates from %s...", roi_file_name.c_str());
                    for (auto c : roi_json["features"]) {
                        // invalid feature
                        if (c.contains("geometry") == false || c["geometry"].contains("coordinates") == false || c["geometry"]["coordinates"].size() < 1 || c["geometry"]["coordinates"][0].size() < 3) {
                            ROS_ERROR("Skipping feature in file %s, invalid syntax: not enough coordinates specified!", roi_file_name.c_str());
                            continue;
                        }

                        // altitude
                        double alt = 0;
                        if (c.contains("properties") && c["properties"].contains("agl") && c["properties"]["agl"] > 0)
                            alt = c["properties"]["agl"];

                        // extract coordinates
                        vector<double> x;
                        vector<double> y;
                        vector<double> z;
                        for (auto p : c["geometry"]["coordinates"][0]) {
                            x.push_back(p[0]);
                            y.push_back(p[1]);
                            z.push_back(alt);
                        }

                        // create roi object
                        ROS_INFO("Add ROI from file %s...", roi_file_name.c_str());
                        add_roi(x, y, z);
                    }
                }

                // unknown file type
                else {
                    ROS_ERROR("Skipping file %s, unknown file type, must be QGC plan or GeoJSON!", roi_file_name.c_str());
                    continue;
                }
            }

            catch (json::exception &e) {
                ROS_DEBUG("Skipping file %s, invalid syntax: %s", roi_file_name.c_str(), e.what());
            }

            catch (exception &e) {
                ROS_DEBUG("Skipping file %s: %s", roi_file_name.c_str(), e.what());
            }
        }
    }

    catch (filesystem::filesystem_error &e) {
        ROS_ERROR("Cannot read ROI files from %s: %s", roi_dir_str.c_str(), e.what());
    }

    if (regions.size() < 1) {
        ROS_INFO("No ROIs specified in %s!", roi_dir_str.c_str());
    }
}

bool rois::set_state (set<tuple<double,double,double>> roi, roi_state_t state)
{
    bool unknown_roi = true;

    // find roi by coordinates
    for (auto r : regions) {
        // roi exists
        if (r.coords[0] == roi) {
            unknown_roi = false;

            // valid state
            if (ROI_TODO <= state && state <= ROI_DONE) {
                // take roi out of set
                auto temp = regions.extract(r);

                // change state
                temp.value().state = state;

                // reinsert it into the set
                regions.insert(move(temp));

                // state change succeeded
                return true;
            }
        }
    }

    // unknown roi
    if (unknown_roi) {
        stringstream coords_ss;
        for (auto p : roi)
            coords_ss << "(" << get<0>(p) << "," << get<1>(p) << "," << get<2>(p) << ") ";
        ROS_WARN("Could not find ROI %s", coords_ss.str().c_str());
    }

    // invalid state
    else {
        ROS_WARN("Invalid ROI state %d!", state);
    }

    // assume either unknown roi or wrong state by default
    return false;
}

void rois::roi_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& event)
{
    // create roi object
    ROS_INFO("Received ROI from swarm member %s", event->swarmio.node.c_str());
    add_roi(event->x, event->y, event->z);
}

void rois::state_callback (const cpswarm_msgs::PointArrayStateEvent::ConstPtr& event)
{
    // equal number of x and y coordinates required
    if (event->x.size() != event->y.size() || event->x.size() != event->z.size() || event->y.size() != event->z.size()) {
        ROS_ERROR("Cannot change ROI state, number of x, y, and z coordinates do not match (%lu != %lu != %lu)", event->x.size(), event->y.size(), event->z.size());
        return;
    }

    // not enough coordinates
    else if (event->x.size() < 3) {
        ROS_ERROR("Cannot change ROI state, not enough coordinates: %lu", event->x.size());
        return;
    }

    // convert coordinates to set
    set<tuple<double,double,double>> coords;
    cpswarm_msgs::FixToPose f2p;
    for (int i=0; i<event->x.size(); ++i) {
        // convert global to local coordinates
        if (global) {
            f2p.request.fix.longitude = event->x[i];
            f2p.request.fix.latitude = event->y[i];
            f2p.request.fix.altitude = event->z[i];
            if (fix_to_pose_client.call(f2p)) {
                coords.emplace(f2p.response.pose.pose.position.x, f2p.response.pose.pose.position.y, event->z[i]); // altitude of roi is always above ground level
            }
            else
                ROS_FATAL("AREA_PROV - Failed to convert area bounds to local coordinates");
        }

        // copy local coordinates
        else
            coords.emplace(event->x[i], event->y[i], event->z[i]);

    }

    // set state
    if (set_state(coords, (roi_state_t)event->state))
        ROS_INFO("Changed ROI state to %d as requested by swarm member %s", event->state, event->swarmio.node.c_str());
    else
        ROS_ERROR("ROI state change requested by swarm member %s failed! ", event->swarmio.node.c_str());
}
