#ifndef MA_H
#define MA_H

#include "lib/area.h"

/**
 * @brief A class that holds the mission area, i.e., the area in which CPSs are allowed to move.
 */
class ma : public area
{
public:
    /**
     * @brief Constructor that initializes mission area coordinates.
     */
    ma ();

private:
    /**
     * @brief Extract coordinates from existing map.
     */
    void map_to_coords ();

    /**
     * @brief Read mission area coordinates from ROS parameters. If global (GPS) coordinates are given they are converted to a local coordinate system.
     */
    void read_coords ();

    /**
     * @brief Callback function to receive map updates.
     */
    void map_callback (const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
     * @brief Subscriber to receive an existing map.
     */
    Subscriber map_subscriber;
};

#endif // MA_H
