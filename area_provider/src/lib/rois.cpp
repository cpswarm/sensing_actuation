#include "lib/rois.h"

rois::rois ()
{
    // directory with roi coodinate files
    string rois_dir_str;
    nh.param(this_node::getName() + "/rois", rois_dir_str, "");

    if (rois_dir_str.empty() == false) {
        // iterate all files in given directory
        const filesystem::path rois_dir{rois_dir_str};
        for(auto const& roi_file : filesystem::directory_iterator{rois_dir}) {
            // convert file contents to json object
            ifstream roi_stream(roi_file);
            json roi_json;
            roi_stream >> roi_json;

            // invalid plan file
            if (roi_json.contains("fileType") == false or roi_json["fileType"] != "Plan")
                continue;
            if (roi_json.contains("mission") == false or roi_json["mission"].contains("items") == false or roi_json["mission"]["items"].size < 3)
                continue;

            // TODO
            void coords;
            regions.append(roi(coords));

            // extract coordinates

            // create roi object
        }
    }
}
