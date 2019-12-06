#include <ros/ros.h>
#include <tf2/utils.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;

/**
 * @brief Definition of the action client for moving the robot.
 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @brief The goal pose.
 */
geometry_msgs::Pose goal;

/**
 * @brief The position of the CPS.
 */
geometry_msgs::Pose pose;

/**
 * @brief Frame id of the position
 */
string frame_id;

/*
 * @brief Whether a valid position has been received.
 */
bool pose_valid;

/**
 * @brief Whether the goal is valid and should be pursued.
 */
bool goal_valid;

/**
 * @brief Whether the goal is invalid and should be canceled.
 */
bool cancel_goal;

/**
 * @brief Compute the yaw angle from a given pose.
 * @param pose The pose to compute the angle from.
 * @return An angle that represents the orientation of the pose.
 */
double get_yaw (geometry_msgs::Pose pose)
{
    tf2::Quaternion orientation;
    tf2::fromMsg(pose.orientation, orientation);
    return tf2::getYaw(orientation);
}

/**
 * Callback function to receive current pose.
 * @param msg The received CPS pose in local coordinates.
 */
void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose = msg->pose;
    
    ROS_DEBUG_THROTTLE(10, "POS_CTRL - Local pose (%.2f,%.2f,%2.f)", pose.position.x, pose.position.y, get_yaw(pose));
    pose_valid = true;
}

/**
 * Callback function for the CPS goal.
 * @param msg The CPS goal in local coordinates.
 */
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal = msg->pose;
    frame_id = msg->header.frame_id;
    ROS_DEBUG("POS_CTRL - Local goal (%.2f,%.2f,%.2f)", goal.position.x, goal.position.y, get_yaw(goal));

    goal_valid = true;
    cancel_goal = false;
    
}

/**
 * Callback function to stop the CPS movement.
 * @param msg An empty message.
 */
void stop_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_DEBUG("POS_CTRL - Stop moving");
    goal_valid = false;
    cancel_goal = true;
}

/**
 * @brief Main function to be executed by ROS.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Success.
 */
int main(int argc, char **argv)
{
    // init ros node
    init(argc, argv, "pos_controller");
    NodeHandle nh;

    // read parameters
    double x,y;
    nh.param(this_node::getName() + "/x", x, 0.0);
    nh.param(this_node::getName() + "/y", y, 0.0);
    geometry_msgs::Pose origin;
    origin.position.x = x;
    origin.position.y = y;
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 1);
    double goal_tolerance;
    nh.param(this_node::getName() + "/goal_tolerance", goal_tolerance, 0.1);
    double goal_tolerance_a;
    nh.param(this_node::getName() + "/goal_tolerance_a", goal_tolerance_a, 0.1);
    double goal_timeout;
    nh.param(this_node::getName() + "/goal_timeout", goal_timeout, 30.0);
    
    //To clear costmap before navigation
	ServiceClient clear_costmaps_client;
	std_srvs::Empty clear_costmaps;
	clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");


    // wait for valid pose and goal
    pose_valid = false;
    goal_valid = false;
    cancel_goal = false;

    // position subscriber
    Subscriber pose_sub = nh.subscribe("pos_provider/pose", queue_size, pose_callback);

    // move base action client in separate thread
    MoveBaseClient ac("move_base", true);

    // wait for valid position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("POS_CTRL - Waiting for valid pose");
        spinOnce();
        rate.sleep();
    }

    // wait for move base action server
    while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

    // goal subscribers
    Subscriber goal_sub = nh.subscribe("pos_controller/goal_position", queue_size, goal_callback);
    Subscriber stop_sub = nh.subscribe("pos_controller/stop", queue_size, stop_callback);

    ROS_DEBUG("POS_CTRL - Ready");

    // provide position controller
    while (ok()) {
        // get position and goal update
        spinOnce();

		//if there is a new goal
		if(goal_valid){
			
			// goal action message
			move_base_msgs::MoveBaseGoal goal_msg;
			goal_msg.target_pose.header.frame_id = frame_id;
			
			// create goal message
			goal_msg.target_pose.header.seq++;
			goal_msg.target_pose.header.stamp = Time::now();
			goal_msg.target_pose.pose = goal;
			
			// shift goal according to origin
			goal_msg.target_pose.pose.position.x -= origin.position.x;
			goal_msg.target_pose.pose.position.y -= origin.position.y;
			
			//clear costmaps
			if(clear_costmaps_client.call(clear_costmaps)){
				ROS_INFO("clearing costmaps");
			}else{
				ROS_ERROR("error clearing costmaps");
			}
			
			// send goal to ugv
			ac.sendGoalAndWait(goal_msg, Duration(goal_timeout));
			
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				ROS_INFO("Goal reached!");
			else
				ROS_INFO("The base failed to move for some reason");
				
			goal_valid = false;
		}       

        // sleep rest of cycle
        rate.sleep();
    }

    return 0;
}
