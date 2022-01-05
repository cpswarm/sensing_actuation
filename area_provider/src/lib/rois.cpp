#include "lib/rois.h"

rois::rois ()
{
    // read parameters
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    nh.param(this_node::getName() + "/duplicates", duplicates, false);

    // get rois from incoming event messages
    roi_subscriber = nh.subscribe("bridge/events/roi", queue_size, &rois::roi_callback, this);

    // publish new rois
    nh.param(this_node::getName() + "/publish", publish, false);
    if (publish)
        roi_publisher = nh.advertise<cpswarm_msgs::PointArrayEvent>("rois/roi", queue_size, true); // latched

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

    // flatten roi coordinates
    vector<geometry_msgs::Point> coords_flat;
    for (auto roi : coords) {
        coords_flat.insert(coords_flat.end(), roi.begin(), roi.end());
    }

    // define multi-array layout
    vector<std_msgs::MultiArrayDimension> dim;

    std_msgs::MultiArrayDimension dim0;
    dim0.label = "roi";
    dim0.size = coords.size();
    dim0.stride = coords.size() * max_num_coords;
    dim.push_back(dim0);

    std_msgs::MultiArrayDimension dim1;
    dim1.label = "coords";
    dim1.size = max_num_coords;
    dim1.stride = max_num_coords;
    dim.push_back(dim1);

    std_msgs::MultiArrayLayout layout;
    layout.dim = dim;
    layout.data_offset = 0;

    // create service response
    res.layout = layout;
    res.points = coords_flat;

    return true;
}

bool rois::get_closest (cpswarm_msgs::GetDist::Request &req, cpswarm_msgs::GetDist::Response &res)
{
    cpswarm_msgs::GetDist::Response closest;
    cpswarm_msgs::GetDist::Response response;

    for (auto roi : regions) {
        if (roi.get_distance(req, response))
            if (closest.closest_line.size() <= 0 || response.distance < closest.distance)
                closest = response;
        else
            ROS_ERROR("Failed to retrieve distance for ROI %s!", roi.to_string().c_str());
    }

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
    return false;
}

bool rois::reload (std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // reset old rois
    if (req.data)
        regions.clear();

    // import rois from files
    from_file();

    res.success = true;

    return true;
}

void rois::add_roi (vector<double> x, vector<double> y)
{
    // equal number of x and y coordinates required
    if (x.size() != y.size()) {
        ROS_ERROR("Cannot add ROI, number of x and y coordinates do not match (%lu != %lu)", x.size(), y.size());
    }

    // not enough coordinates
    else if (x.size() < 3) {
        ROS_ERROR("Cannot add ROI, not enough coordinates: %lu", x.size());
    }

    else {
        // add roi
        roi roi(x, y);

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
            event.x = x;
            event.y = y;
            roi_publisher.publish(event);
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

void rois::from_file ()
{
    // directory with roi coodinate files
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

                // invalid plan file
                if (roi_json.contains("fileType") == false or roi_json["fileType"] != "Plan") {
                    ROS_ERROR("Skipping file %s, invalid syntax: not a qGroundControl plan file!", roi_file_name.c_str());
                    continue;
                }
                if (roi_json.contains("mission") == false or roi_json["mission"].contains("items") == false or roi_json["mission"]["items"].size() < 3) {
                    ROS_ERROR("Skipping file %s, invalid syntax: no mission specified!", roi_file_name.c_str());
                    continue;
                }

                // extract coordinates
                ROS_DEBUG("Extract ROI coordinates from %s...", roi_file_name.c_str());
                vector<double> x;
                vector<double> y;
                for (auto c : roi_json["mission"]["items"]) {
                    if (c["params"].size() < 7) {
                        ROS_ERROR("Skipping file %s, invalid syntax: number of params must be 7 for each mission item!", roi_file_name.c_str());
                        break;
                    }
                    if (c["command"] != 16) {
                        ROS_DEBUG("%s: Skipping item (%f,%f), not a waypoint!", roi_file_name.c_str(), double(c["params"][4]), double(c["params"][5]));
                        continue;
                    }
                    x.push_back(c["params"][4]);
                    y.push_back(c["params"][5]);
                }

                // create roi object
                ROS_INFO("Add ROI from file %s...", roi_file_name.c_str());
                add_roi(x, y);
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

void rois::roi_callback (const cpswarm_msgs::PointArrayEvent::ConstPtr& event)
{
    // create roi object
    ROS_INFO("Received ROI from swarm member %s", event->swarmio.node.c_str());
    add_roi(event->x, event->y);
}
