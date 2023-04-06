#ifndef ROI_H
#define ROI_H

#include "lib/area.h"

/**
 * @brief A class that holds a region of interest (ROI).
 */
class roi : public area
{
    /**
     * @brief Allow the ROI manager to access private members.
     */
    friend class rois;

public:
    /**
     * @brief Constructor that initializes ROI coordinates.
     * @param x The x-coordinates of the ROI, can be global (GPS) or local coordinates.
     * @param y The y-coordinates of the ROI, can be global (GPS) or local coordinates.
     */
    roi (vector<double> x, vector<double> y);

    /**
     * @brief Get the global coordiantes.
     *
     * @return A pair of vectors. First the longitudes, second the latitudes.
     */
    pair<vector<double>, vector<double>> get_global ();

    /**
     * @brief Compare two ROIs.
     * @param other The other ROI to compare to this one.
     * @returns True, if both ROIs have the same coordinates, false otherwise.
     */
    bool operator== (const roi other);

    /**
     * @brief Compare two ROIs.
     * @param other The other ROI to compare to this one.
     * @return True, if this ROIs coordinates are less than the other ROI coordinates, false otherwise.
     */
    bool operator< (const roi other) const;

};

#endif // ROI_H
