# include "mavros_battery_monitor.h"

battery_monitor::battery_monitor ()
{
    // read parameters
    double loop_rate;
    nh.param(this_node::getName() + "/loop_rate", loop_rate, 5.0);
    Rate rate(loop_rate);
    int queue_size;
    nh.param(this_node::getName() + "/queue_size", queue_size, 10);
    bool simulation;
    nh.param(this_node::getName() + "/simulation", simulation, false);
    nh.getParam(this_node::getName() + "/run_time", run_time);
    nh.param(this_node::getName() + "/spare_time", spare_time, 60);

    // initialize map of charging points
    init_cps();

    // initialize state-of-charge look-up-table
    init_lut();

    // get velocity
    ServiceClient pull_client = nh.serviceClient<mavros_msgs::ParamPull>("mavros/param/pull");
    ROS_DEBUG("Waiting MAVROS parameter pull service");
    pull_client.waitForExistence();
    mavros_msgs::ParamPull pull;
    if (pull_client.call(pull)) {
        if (pull.response.success) {
            ROS_DEBUG("Pulled %d parameters from FCU", pull.response.param_received);
            nh.getParam("mavros/param/MPC_XY_VEL_MAX", vel_xy);
            nh.getParam("mavros/param/MPC_Z_VEL_MAX_DN", vel_dn);
        }
        else {
            ROS_ERROR("Failed to pull parameters from FCU, cannot get velocity!");
        }
    }
    else {
        ROS_ERROR("Failed to pull parameters from FCU, cannot get velocity!");
    }

    // ros communication
    pose_sub = nh.subscribe("pos_provider/pose", queue_size, &battery_monitor::pose_callback, this);
	if (simulation)
    	battery_sub = nh.subscribe("battery_simulation", queue_size, &battery_monitor::battery_callback, this);
    else
    	battery_sub = nh.subscribe("mavros/battery", queue_size, &battery_monitor::battery_callback, this);
    battery_pub = nh.advertise<cpswarm_msgs::BatteryState>("battery", queue_size, true);

    // init position
    while (ok() && pose_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid pose...");
        rate.sleep();
        spinOnce();
    }

    // init battery state
    while (ok() && battery_valid == false) {
        ROS_DEBUG_ONCE("Waiting for valid battery state...");
        rate.sleep();
        spinOnce();
    }
}

void battery_monitor::calculate ()
{
/* TODO: remove
header:
  seq: 171
  stamp:
    secs: 1612517155
    nsecs: 218704222
  frame_id: ''
voltage: 17.134000778198242
temperature: 0.0
current: 0.009999999776482582
charge: nan
capacity: nan
design_capacity: nan
percentage: -0.009999999776482582
power_supply_status: 2
power_supply_health: 0
power_supply_technology: 3
present: True
cell_voltage: []
cell_temperature: []
location: "id1"
serial_number: ''
*/

    // estimate remaining run time
    time_remaining_total = int(round(double(run_time) * soc[battery.voltage] - spare_time));

    if (time_remaining_total < 0) {
        ROS_ERROR("Battery low, need to land immediately!");
    }

    // find closest charging point
    double cp_dist = 0;
    int cp_id;
    geometry_msgs::Point cp;
    for (auto cp : cps) {
        double dx = cp.second.x - pose.position.x;
        double dy = cp.second.y - pose.position.y;
        double dist_square = dx*dx + dy*dy;
        if (cp_dist <= 0 || dist_square < cp_dist) {
            cp_dist = dist_square;
            cp_id = cp.first;
        }
    }
    cp_dist = sqrt(cp_dist);

    // calculate required time to reach charging point
    time_return = int(round(cp_dist / vel_xy + pose.position.z / vel_dn));

    // remaining time available for work
    time_remaining_work = time_remaining_total - time_return;

    if (time_remaining_work < 0) {
        ROS_WARN("Battery low, need to return for recharging immediately!");
    }
}

void battery_monitor::publish ()
{
    cpswarm_msgs::BatteryState msg;
    msg.soc = int(round(soc[battery.voltage] * 100));
    msg.time_left = time_remaining_total;
    msg.time_work = time_remaining_work;

    battery_pub.publish(msg);
}

void battery_monitor::init_cps ()
{
    // read all charging points from parameter file
    vector<double> cps_x;
    vector<double> cps_y;
    nh.getParam(this_node::getName() + "/cps_x", cps_x);
    nh.getParam(this_node::getName() + "/cps_y", cps_y);
    if (cps_x.size() != cps_y.size()) {
        ROS_FATAL("Invalid charging points specified! Exiting...");
        shutdown();
    }
    else if (cps_x.size() < 1)
        ROS_INFO("There are no charging points!");

    // place charging points in map
    for (int i = 0; i < cps_x.size(); ++i) {
        ROS_DEBUG("Charging point %d at [%.2f, %.2f]", i, cps_x[i], cps_y[i]);
        geometry_msgs::Point new_cps_pos;
        new_cps_pos.x = cps_x[i];
        new_cps_pos.y = cps_y[i];
        cps.emplace(i, new_cps_pos);
    }
}

void battery_monitor::init_lut ()
{
    // TODO: replace by measurements
    // read all charging points from parameter file
    vector<double> volt;
    vector<double> soc;
    nh.getParam(this_node::getName() + "/voltage", volt);
    nh.getParam(this_node::getName() + "/soc", soc);
    if (volt.size() != soc.size() || volt.size() < 10) {
        ROS_FATAL("Invalid SOC-voltage look-up-table specified! Exiting...");
        shutdown();
    }

    // create look-up-table
    transform(volt.begin(), volt.end(), soc.begin(), inserter(this->soc, this->soc.end()), [](double a, double b){return make_pair(a, b);});
}

void battery_monitor::pose_callback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // valid pose received
    if (msg->header.stamp.isValid())
        pose_valid = true;

    // store new position and orientation in class variables
    pose = msg->pose;

    ROS_DEBUG_THROTTLE(1, "Pose [%.2f, %.2f, %.2f]", pose.position.x, pose.position.y, pose.position.z);
}

void battery_monitor::battery_callback (const sensor_msgs::BatteryState::ConstPtr& msg)
{
    // valid battery state received
    if (msg->header.stamp.isValid() && msg->voltage > 0 && msg->power_supply_status > 0 && msg->present)
        battery_valid = true;

    // store new battery state in class variable
    battery = *msg;
}