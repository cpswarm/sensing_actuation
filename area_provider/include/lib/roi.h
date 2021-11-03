#ifndef ROI_H
#define ROI_H

#include "lib/area.h"

/**
 * @brief A class that holds a region of interest (ROI).
 */
class roi : public area
{
public:
    /**
     * @brief Constructor that initializes ROI coordinates.
     * @param raw_coords The coordinates of the ROI, can be global (GPS) or local coordinates.
     */
    roi (vector<geometry_msgs::Point> raw_coords);
};

#endif // ROI_H
