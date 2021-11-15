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
     * @param x The x-coordinates of the ROI, can be global (GPS) or local coordinates.
     * @param y The y-coordinates of the ROI, can be global (GPS) or local coordinates.
     */
    roi (vector<double> x, vector<double> y);
};

#endif // ROI_H
