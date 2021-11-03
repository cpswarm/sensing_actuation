#ifndef SWARM_POSITION_H
#define SWARM_POSITION_H

#include <ros/ros.h>
#include <cpswarm_msgs/ArrayOfVectors.h>
#include <cpswarm_msgs/VectorStamped.h>
#include "sector.h"

using namespace std;
using namespace ros;

/**
 * @brief A class that holds position information of the other CPSs in the swarm.
 */
class swarm_position
{
public:
    /**
     * @brief Constructor that initializes the private member variables.
     */
    swarm_position ();

    /**
     * @brief Check if the CPS is far enough from other CPSs ahead.
     * @param heading The heading of the CPS.
     * @return True if all other CPSs are either further away than avoidance distance or their bearing is outside of the heading angle of the CPS. False otherwise.
     */
    bool clear_ahead (double heading) const;

    /**
     * @brief Check if the CPS is dangerously close to other CPSs.
     * @return The distance that the current CPS has to back off from the other CPS in order to reach a safe distance again. Returns 0.0 in case no CPS is closer than the critical distance.
     */
    double danger () const;

    /**
     * @brief Get the absolute sector occupied by other CPSs with added safety bearing.
     * @return The inflated occupied sector.
     */
    sector inflated_region () const;

    /**
     * @brief Get the absolute sector occupied by other CPSs.
     * @return The occupied sector.
     */
    sector occupied_region () const;

private:
    /**
     * Caclulate the bearing error.
     * @param pose The pose (bearing, distance) to check.
     * @return The bearing error.
     */
    double bearing_tolerance (cpswarm_msgs::VectorStamped pose) const;

    /**
     * @brief Callback function to receive the relative positions of the other CPSs.
     * @param msg Distance and bearing of the other CPSs.
     */
    void swarm_pose_callback (const cpswarm_msgs::ArrayOfVectors::ConstPtr& msg);

    /**
     * @brief A node handle for the main ROS node.
     */
    NodeHandle nh;

    /**
     * @brief Subscriber for the relative positions of the other CPSs.
     */
    Subscriber swarm_pose_sub;

    /**
     * @brief The relative positions of the other swarm members (distance and bearing).
     */
    vector<vector<cpswarm_msgs::VectorStamped>> poses;

    /**
     * @brief The distance in meters which describes the uncertainty radius of the pose.
     */
    double pose_variation;

    /**
     * @brief The distance in meters to another CPSs below which they start their collision avoidance procedure.
     */
    double avoidance_dist;

    /**
     * @brief The distance in meters to another CPS below which they have to move apart first before the standard avoidance procedure.
     */
    double critical_dist;

    /**
     * @brief The angle in radian that must be clear on each side around the CPS's movement direction.
     */
    double clear_ang;

    /**
     * @brief The index of the latest position in the relative poses vectors.
     */
    unsigned int t;
};

#endif // SWARM_POSITION_H
