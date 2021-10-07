#include "lib/rois.h"

rois::rois ()
{
    // directory with roi coodinate files
    string roi_dir_str = "";
    nh.param(this_node::getName() + "/roi_dir", roi_dir_str, roi_dir_str);

    // no roi files
    if (roi_dir_str.empty()) {
        ROS_ERROR("No directory for ROI files given!");
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
                vector<geometry_msgs::Point> coords;
                for (auto c : roi_json["mission"]["items"]) {
                    if (c["params"].size() < 7) {
                        ROS_ERROR("Skipping file %s, invalid syntax: number of params must be 7!", roi_file_name.c_str());
                        break;
                    }
                    geometry_msgs::Point coord;
                    coord.x = c["params"][4];
                    coord.y = c["params"][5];
                    coord.z = c["params"][6];
                    coords.push_back(coord);
                }

                // create roi object
                if (coords.size()) {
                    roi roi(coords);
                    ROS_INFO("Added ROI #%lu from %s: %lu coordinates", regions.size(), roi_file_name.c_str(), coords.size());
                    regions.emplace(regions.size(), roi);
                }
            }

            catch (json::exception &e) {
                ROS_DEBUG("Skipping file %s, invalid syntax: %s", roi_file_name.c_str(), e.what());
            }
        }
    }

    catch (filesystem::filesystem_error &e) {
        ROS_ERROR("Cannot read ROI files from %s: %s", roi_dir_str.c_str(), e.what());
    }

    if (regions.size() < 1) {
        ROS_ERROR("No ROIs specified!");
    }
}

map<int,roi> rois::get_rois ()
{
    return regions;
}
