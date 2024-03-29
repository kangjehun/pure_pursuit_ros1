/*
 * Pure Pursuit Controller
 * Author: KangJehun 20170016 wpgnssla34@kaist.ac.kr
 * Last Edit : 2024.03.23
 * Implementation of pure-pursuit steering controller with bicycle model
 */

// Basic Libraries
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <tf/transform_datatypes.h>
#include <chrono>
// Messages
#include <nav_msgs/Path.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "velocity_profile_msgs/VelocityProfileStamped.h"
#include "waypoint_srvs/GetWaypoints.h"
#include "velocity_profile_srvs/GetVelocityProfile.h"
// Visualization
#include <visualization_msgs/Marker.h>
// TF
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Other Libraries
#include "utils_basic/utils_basic.h"



using namespace std;

// MACROS
// Queue Size
#define SUBQUEUESIZE 1
#define PUBQUEUESIZE 1

// GLOBAL VARIABLES
float max_float = numeric_limits<float>::max();

class PIDController {
public:
    double kp;  
    double ki;  
    double kd;  
    double dt;
    double prev_error;  
    double integral;

public:
    PIDController(double p, double i, double d, double t) : 
        kp(p), ki(i), kd(d), dt(t), prev_error(0.), integral(0.) {}

    double compute(double setpoint, double measured_value) {
        double error = setpoint - measured_value;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class VehicleController
{
private:
    // Handler
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    // Subscriber
    ros::Subscriber vehicle_state_sub_;
    ros::Subscriber waypoints_sub_;
    ros::Subscriber velocity_sub_;
    // Service Client
    ros::ServiceClient waypoints_client_;
    ros::ServiceClient velocity_profile_client_;
    // Publisher
    ros::Publisher vehicle_cmd_vel_pub_;
    ros::Publisher marker_nearest_waypoint_pub_;
    ros::Publisher marker_control_signal_pub_;
    // Service Response 
    nav_msgs::Path waypoints_; // global path
    velocity_profile_msgs::VelocityProfileStamped velocity_profile_;
    // Msgs to publish
    geometry_msgs::Twist cmd_vel_msg_;
    visualization_msgs::Marker waypoint_marker_;
    visualization_msgs::Marker control_signal_marker_;
    // TF
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    // Parameters 
    bool verbose_;                              // Show debugging messages
    bool realtime_waypoint_;                    // Update waypoints in real time (if true)
    float lookahead_;                           // Lookahead distance
    float wheelbase_length_;                    // Wheel-base length of vehicle
    float min_velocity_;                        // Minimum velocity of vehicle [mps]
    float max_velocity_;                        // Maximum velocity of vehicle [mps]
    float min_steering_;                        // Minimum steering angle of vehicle [rad]
    float max_steering_;                        // Maximum steering angle of vehicle [rad]
    float min_velocity_clipping_;                // Minimum velocity of vehicle [mps]
    float max_velocity_clipping_;                // Maximum velocity of vehicle [mps]
    float min_steering_clipping_;                // Minimum steering angle of vehicle [rad]
    float max_steering_clipping_;                // Maximum steering angle of vehicle [rad]
    // Member variables
    float theta_rear2goal_;                     // global heading angle of rear to goal vector
    float heading_;                             // global heading angle of the vehicle
    float alpha_;                               // theta_rear2goal - heading
    bool vehicle_state_is_updated_;
    int vehicle_state_update_count_;
    bool tf_is_updated_;
    int tf_update_count_;
    bool realtime_waypoint_is_updated_;
    bool realtime_velocityprofile_is_updated_;
    float x_ego_;
    float y_ego_;
    float z_ego_;
    float yaw_ego_;
    float vx_ego_;
    float wz_ego_;
    int nearest_idx_;
    int lookahead_idx_;
    size_t wp_arr_size_;
    string odometry_topic_;
    float control_signal_d_;
    float control_signal_theta_;
public:
    // Const
    const float frequency_;
    // PID Controller
    PIDController speed_pid_controller_;

private:
    void init_params();
    float find_distance(float x, float y);
    float find_distance_index_based(int idx);
    void find_nearest_waypoint();
    void find_idx_close_to_lookahead();
    void update_alpha();
    void visualize_waypoint_with_idx(int idx, int id, float r, float g, float b);
    void visualize_control_signal(int id, float r, float g, float b);
    //callback
    void callback_vehiclestate(const nav_msgs::Odometry::ConstPtr &msg);
    void callback_waypoints(const nav_msgs::Path::ConstPtr &msg);
    void callback_velocity_profile(const velocity_profile_msgs::VelocityProfileStamped::ConstPtr &msg);
public:
    VehicleController(ros::NodeHandle &nh, ros::NodeHandle &pnh, float frequency);
    ~VehicleController();
    void update_vehicle_state();
    void purepursuit();
};

// Constructer
VehicleController::VehicleController(ros::NodeHandle &nh, ros::NodeHandle &pnh, float frequency):
    nh_(nh), pnh_(pnh), frequency_(frequency), speed_pid_controller_(1.0, 0.0, 0.0, 1.0 / frequency),
    tfListener_(tfBuffer_)
{
    cout << "Pure Pursuit Node is Launched" << endl;
    // Set Parameters
    init_params();
    // Set Member Varibles
    theta_rear2goal_ = 0;
    heading_ = 0;
    alpha_ = 0;
    // waypoint_is_updated = false;
    vehicle_state_is_updated_ = false;
    vehicle_state_update_count_ = 0;
    tf_is_updated_ = false;
    tf_update_count_ = 0;
    realtime_waypoint_is_updated_ = false;
    realtime_velocityprofile_is_updated_ = false;
    x_ego_ = 0;
    y_ego_ = 0;
    z_ego_ = 0;
    yaw_ego_ = 0;
    vx_ego_ = 0;
    wz_ego_ = 0;
    nearest_idx_ = -1;
    wp_arr_size_ = 0;
    // Set Publishers
    vehicle_cmd_vel_pub_ 
        = nh_.advertise<geometry_msgs::Twist>("/scout/cmd_vel", PUBQUEUESIZE);
    marker_nearest_waypoint_pub_ 
        = nh_.advertise<visualization_msgs::Marker>("visualization_marker_nearest_waypoint", 1);
    marker_control_signal_pub_ 
        = nh_.advertise<visualization_msgs::Marker>("visualization_marker_control_signal", 1);
    // Set Subscribers
    vehicle_state_sub_  = nh_.subscribe(odometry_topic_, SUBQUEUESIZE, 
                                        &VehicleController::callback_vehiclestate, this);
    if (!realtime_waypoint_)
    {
        // Set Service Client
        waypoints_client_ = nh_.serviceClient<waypoint_srvs::GetWaypoints>("waypoints");
        velocity_profile_client_ = nh_.serviceClient<velocity_profile_srvs::GetVelocityProfile>("velocity_profile");
        // Call the services to retrieve data
        waypoint_srvs::GetWaypoints waypoints_srv;
        if (waypoints_client_.call(waypoints_srv))
        {
            waypoints_ = waypoints_srv.response.waypoints;
            wp_arr_size_ = waypoints_.poses.size();
        }
        else
        {
            ROS_ERROR("Failed to call waypoints service. Launch waypoint loader first");
            ros::shutdown();
        }
        velocity_profile_srvs::GetVelocityProfile velocity_profile_srv;
        if (velocity_profile_client_.call(velocity_profile_srv))
        {
            velocity_profile_ = velocity_profile_srv.response.velocity_profile;
        }
        else
        {
            ROS_ERROR("Failed to call velocity service. Launch waypoint loader first");
            ros::shutdown();
        }
    }
    else
    {
        waypoints_sub_  = nh_.subscribe("waypoints", SUBQUEUESIZE, 
                                        &VehicleController::callback_waypoints, this);
        velocity_sub_   = nh_.subscribe("velocity_profile", SUBQUEUESIZE, 
                                        &VehicleController::callback_velocity_profile, this);
    }
}

// Destructor
VehicleController::~VehicleController(){}

/*
 * init_parameters
 * func    : Initialize tuning parameters with ROS params
 * return  : None
 */
void VehicleController::init_params()
{
    pnh_.param("verbose", verbose_, true);
    pnh_.param("realtime_waypoint", realtime_waypoint_, true);
    pnh_.param("lookahead_distance", lookahead_, float(0));
    pnh_.param("wheelbase_length", wheelbase_length_, float(0.475));
    pnh_.param("kp", speed_pid_controller_.kp, 1.);
    pnh_.param("ki", speed_pid_controller_.ki, 0.);
    pnh_.param("kd", speed_pid_controller_.kd, 0.);
    pnh_.param("velocity_min", min_velocity_, float(-60));
    pnh_.param("velocity_max", max_velocity_, float(60));
    pnh_.param("steering_min", min_steering_, float(-25));
    pnh_.param("steering_max", max_steering_, float(25));
    pnh_.param("velocity_min_clipping", min_velocity_clipping_, float(-12));
    pnh_.param("velocity_max_clipping", max_velocity_clipping_, float(12));
    pnh_.param("steering_min_clipping", min_steering_clipping_, float(-25));
    pnh_.param("steering_max_clipping", max_steering_clipping_, float(25));
    pnh_.param("odometry_topic", odometry_topic_, string("/Odometry/base"));

    max_velocity_ = utils_basic::kph2mps(max_velocity_);
    min_velocity_ = utils_basic::kph2mps(min_velocity_);
    max_steering_ = utils_basic::deg2rad(max_steering_);
    min_steering_ = utils_basic::deg2rad(min_steering_);
    max_velocity_clipping_ = utils_basic::kph2mps(max_velocity_clipping_);
    min_velocity_clipping_ = utils_basic::kph2mps(min_velocity_clipping_);
    max_steering_clipping_ = utils_basic::deg2rad(max_steering_clipping_);
    min_steering_clipping_ = utils_basic::deg2rad(min_steering_clipping_);
    
    // [DEBUG]
    if(verbose_)
    {
        ROS_INFO("Parameter Setting");
        ROS_INFO(realtime_waypoint_ ? "realtime_waypoint : true" : "realtime_waypoint : false");
        ROS_INFO("lookahead : %f", lookahead_);
        ROS_INFO("wheelbase_length : %f", wheelbase_length_);
        ROS_INFO("max_velocity [m/s]: %f", max_velocity_); // [mps]
        ROS_INFO("min_velocity [m/s]: %f", min_velocity_); // [mps]
        ROS_INFO("max_steering [rad]: %f", max_steering_); // [rad]
        ROS_INFO("min_steering [rad]: %f", min_steering_); // [rad]
        ROS_INFO("max_velocity_clipping [m/s]: %f", max_velocity_clipping_); // [mps]
        ROS_INFO("min_velocity_clipping [m/s]: %f", min_velocity_clipping_); // [mps]
        ROS_INFO("max_steering_clipping [rad]: %f", max_steering_clipping_); // [rad]
        ROS_INFO("min_steering_clipping [rad]: %f", min_steering_clipping_); // [rad]
        ROS_INFO("speed PID controller kp : %f", speed_pid_controller_.kp);
        ROS_INFO("speed PID controller ki : %f", speed_pid_controller_.ki);
        ROS_INFO("speed PID controller kd : %f", speed_pid_controller_.kd);
        ROS_INFO("odometry_topic : %s", odometry_topic_.c_str());   
    }
}

float VehicleController::find_distance(float x, float y)
{
    float distance = sqrt(pow(x - x_ego_, 2.) + pow(y - y_ego_, 2.));
    return distance;
}

float VehicleController::find_distance_index_based(int idx)
{
    float x = waypoints_.poses[idx].pose.position.x;
    float y = waypoints_.poses[idx].pose.position.y;
    return find_distance(x, y);
}

void VehicleController::find_nearest_waypoint()
{
    float idx_dist;
    float smallest_dist;

    for (int i = 0; i < wp_arr_size_ ; i++)
    {
        idx_dist = find_distance_index_based(i);
        if (i == 0) 
        {
            smallest_dist = idx_dist;
            nearest_idx_ = i;
        }
        if (idx_dist < smallest_dist)
        {
            smallest_dist = idx_dist;
            nearest_idx_ = i;
        }
    }
}

void VehicleController::find_idx_close_to_lookahead()
{
    int idx = nearest_idx_;
    if(find_distance_index_based(idx) > lookahead_)
    {
        lookahead_idx_ = nearest_idx_;
    }
    if (vx_ego_ >= 0)
    {
        while (find_distance_index_based(idx) <= lookahead_)
        {
            lookahead_idx_ = idx;
            idx += 1;
            if (idx >= wp_arr_size_) { break; }
        }
    }
    else
    {
        while (find_distance_index_based(idx) < lookahead_)
        {
            lookahead_idx_ = idx;
            idx -= 1;
            if (idx <= 0) { break; }
        }
    }
} 

void VehicleController::update_alpha()
{
    float target_x = waypoints_.poses[lookahead_idx_].pose.position.x;
    float target_y = waypoints_.poses[lookahead_idx_].pose.position.y;
    float x_delta = target_x - x_ego_;
    float y_delta = target_y - y_ego_;
    heading_ = utils_basic::normalizeAngle(yaw_ego_);
    theta_rear2goal_ = utils_basic::normalizeAngle(atan2(y_delta, x_delta));
    alpha_ = utils_basic::normalizeAngle(theta_rear2goal_ - heading_);
    //[DEBUG]
    // ROS_INFO_THROTTLE(1.0, "heading angle    [deg]: %f", utils_basic::rad2deg(heading_));
    // ROS_INFO_THROTTLE(1.0, "theta_rear2goal  [deg]: %f", utils_basic::rad2deg(theta_rear2goal_));
    // ROS_INFO_THROTTLE(1.0, "alpha            [deg]: %f", utils_basic::rad2deg(alpha_));
}


/*
    Update vehicle state with tf data
*/
void VehicleController::update_vehicle_state()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0));
        // Position
        x_ego_ = transformStamped.transform.translation.x;
        y_ego_ = transformStamped.transform.translation.y;
        z_ego_ = transformStamped.transform.translation.z;

        // Orientation
        tf2::Quaternion q(
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z,
            transformStamped.transform.rotation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        yaw_ego_ = yaw;

        if (!tf_is_updated_)
        {
            tf_is_updated_ = true;
        }
        tf_update_count_++;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}

/*
    Implementation of vehicle state callback
*/
void VehicleController::callback_vehiclestate(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!vehicle_state_is_updated_)
    {
        vehicle_state_is_updated_ = true;
    }
    vehicle_state_update_count_++;

    // Position
    // float x = msg->pose.pose.position.x;
    // float y = msg->pose.pose.position.y;
    // float z = msg->pose.pose.position.z;

    // Orientation
    // geometry_msgs::Quaternion quat = msg->pose.pose.orientation;
    // float qx = msg->pose.pose.orientation.x;
    // float qy = msg->pose.pose.orientation.y;
    // float qz = msg->pose.pose.orientation.z;
    // float qw = msg->pose.pose.orientation.w;

    // Linear velocity
    float vx = msg->twist.twist.linear.x;
    // float vy = msg->twist.twist.linear.y;
    // float vz = msg->twist.twist.linear.z;
    
    // Vehicle Speed
    // vector<float> vect_v = {vx, vy, vz};
    // float v = utils_basic::normalizeVect(vect_v);

    // Angular velocity
    // float wx = msg->twist.twist.linear.x;
    // float wy = msg->twist.twist.linear.y;
    // float wz = msg->twist.twist.linear.z;

    // Euler from Quaternion
    // double roll, pitch, yaw;
    // utils_basic::quaternionToEuler(quat, roll, pitch, yaw);

    // [DEBUG]
    // ROS_INFO_THROTTLE(1.0, "Vehicle State is updated %d times", vehicle_state_update_count_);
    // ROS_INFO_THROTTLE(1.0, "Global Position [%f, %f, %f], vel [%f], yaw [%f]", x, y, z, v, yaw);
    // ROS_INFO_THROTTLE(1.0, "Roll, Pitch, Yaw [%f, %f, %f]", roll, pitch, yaw);
    // ROS_INFO_THROTTLE(1.0, "Seq [%d], Vel (linear x, norm, angular) [%f, %f, %f]", 
    //             msg->header.seq, vx, v, wz);
    // ROS_INFO_THROTTLE(1.0, "Vel (linear x, norm) [%f, %f]", vx, v);
    // ROS_INFO_THROTTLE(1.0, "Position (xc, yc, zc) [%f, %f, %f]", x, y, z);
    // ROS_INFO_THROTTLE(1.0, "Orientation (qx, qy, qz, qw) [%f, %f, %f, %f]", qx, qy, qz, qw);
    // ROS_INFO_THROTTLE(1.0, "\n");

    // Update member variable
    // x_ego_ = x;
    // y_ego_ = y;
    // z_ego_ = z;
    // yaw_ego_ = yaw;
    vx_ego_ = vx;
    // wz_ego_ = wz;
}

void VehicleController::callback_waypoints(const nav_msgs::Path::ConstPtr &msg)
{
    waypoints_ = *msg;
    wp_arr_size_ = waypoints_.poses.size();
    realtime_waypoint_is_updated_ = true;
}

void VehicleController::callback_velocity_profile(const velocity_profile_msgs::VelocityProfileStamped::ConstPtr &msg)
{
    velocity_profile_ = *msg;
    realtime_velocityprofile_is_updated_ = true;
}


/*
    Implementation of pure pursuit controller
*/ 
void VehicleController::purepursuit() 
{
    if(!vehicle_state_is_updated_)
    {
        // [DEBUG]
        ROS_INFO_THROTTLE(1.0, "vehicle state is not updated yet");
        return;
    }
    if(!tf_is_updated_)
    {
        // [DEBUG]
        ROS_INFO_THROTTLE(1.0, "tf is not updated yet");
        return;
    }
    if(realtime_waypoint_)
    {
        if(!realtime_waypoint_is_updated_)
        {
            // [DEBUG]
            ROS_INFO_THROTTLE(1.0, "realtime waypoint is not updated yet");
            return;
        }
        if(!realtime_velocityprofile_is_updated_)
        {
            // [DEBUG]
            ROS_INFO_THROTTLE(1.0, "realtime velocity profile is not updated yet");
            return;
        }
    }
    // Get the closest waypoint
    find_nearest_waypoint();
    find_idx_close_to_lookahead();
    

    // Velocity PID controller
    float v_desired = velocity_profile_.velocityprofile.velocities[nearest_idx_];
    control_signal_d_ = speed_pid_controller_.compute(v_desired, vx_ego_);

    // Pure Pursuit Controller
    update_alpha();
    control_signal_theta_ = 
        utils_basic::normalizeAngle(atan2((2. * wheelbase_length_ * sin(alpha_)), lookahead_));

    // [DEBUG]
    ROS_INFO_THROTTLE(1.0, "v_desired                     : %f", v_desired);
    ROS_INFO_THROTTLE(1.0, "nearest index                 : %d", nearest_idx_);
    ROS_INFO_THROTTLE(1.0, "lookahead index               : %d", lookahead_idx_);
    ROS_INFO_THROTTLE(1.0, "Before clipping");
    ROS_INFO_THROTTLE(1.0, 
        "control signal (d)     [kph]  : %f", utils_basic::mps2kph(control_signal_d_));
    ROS_INFO_THROTTLE(1.0, 
        "control signal (theta) [deg]  : %f", utils_basic::rad2deg(control_signal_theta_));

    visualize_waypoint_with_idx(nearest_idx_, 0, 1.0, 0.0, 0.0);
    visualize_waypoint_with_idx(lookahead_idx_, 1, 0.0, 0.0, 1.0);
    visualize_control_signal(0, 1.0, 1.0, 0.0);

    // Clipping
    control_signal_d_ = max(min_velocity_clipping_, min(control_signal_d_, max_velocity_clipping_));
    control_signal_theta_ = max(min_steering_clipping_, min(control_signal_theta_, max_steering_clipping_));

    // [DEBUG]
    ROS_INFO_THROTTLE(1.0, "After clipping");
    ROS_INFO_THROTTLE(1.0, 
        "control signal (d)     [kph]  : %f", utils_basic::mps2kph(control_signal_d_));
    ROS_INFO_THROTTLE(1.0, 
        "control signal (theta) [deg]  : %f", utils_basic::rad2deg(control_signal_theta_));

    // Remapping
    control_signal_d_ = 
        utils_basic::normalizeControlSignal(control_signal_d_, min_velocity_, max_velocity_, -1, 1);
    control_signal_theta_ = 
        utils_basic::normalizeControlSignal(control_signal_theta_, min_steering_, max_steering_, -1, 1);

    // [DEBUG]
    ROS_INFO_THROTTLE(1.0, "After remapping");
    ROS_INFO_THROTTLE(1.0, "control signal (d)     [-1, 1]  : %f", control_signal_d_);
    ROS_INFO_THROTTLE(1.0, "control signal (theta) [-1, 1]  : %f", control_signal_theta_);

    // Publish the message
    cmd_vel_msg_.linear.x = control_signal_d_;
    cmd_vel_msg_.angular.z = control_signal_theta_;
    vehicle_cmd_vel_pub_.publish(cmd_vel_msg_);
}

void VehicleController::visualize_waypoint_with_idx(int idx, int id, float r, float g, float b)
{
    waypoint_marker_.header.frame_id = "odom";
    waypoint_marker_.header.stamp = ros::Time::now();
    waypoint_marker_.ns = "nearest_waypoint";
    waypoint_marker_.id = id;
    waypoint_marker_.action = visualization_msgs::Marker::ADD;
    waypoint_marker_.type = visualization_msgs::Marker::SPHERE;
    waypoint_marker_.pose.position.x = waypoints_.poses[idx].pose.position.x;
    waypoint_marker_.pose.position.y = waypoints_.poses[idx].pose.position.y;
    waypoint_marker_.pose.position.z = waypoints_.poses[idx].pose.position.z;
    waypoint_marker_.scale.x = 0.5;
    waypoint_marker_.scale.y = 0.5;
    waypoint_marker_.scale.z = 0.5;
    waypoint_marker_.color.a = 1.0;
    waypoint_marker_.color.r = r;
    waypoint_marker_.color.g = g;
    waypoint_marker_.color.b = b;
    marker_nearest_waypoint_pub_.publish(waypoint_marker_);
}

void VehicleController::visualize_control_signal(int id, float r, float g, float b)
{
    control_signal_marker_.header.frame_id = "odom";
    control_signal_marker_.header.stamp = ros::Time::now();
    control_signal_marker_.ns = "control_signal";
    control_signal_marker_.id = id;
    control_signal_marker_.action = visualization_msgs::Marker::ADD;
    control_signal_marker_.type = visualization_msgs::Marker::ARROW;
    control_signal_marker_.pose.position.x = x_ego_;
    control_signal_marker_.pose.position.y = y_ego_;
    control_signal_marker_.pose.position.z = z_ego_;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = utils_basic::normalizeAngle(heading_ + control_signal_theta_);
    geometry_msgs::Quaternion quat = utils_basic::eulerToQuaternion(roll, pitch, yaw);
    control_signal_marker_.pose.orientation.x = quat.x;
    control_signal_marker_.pose.orientation.y = quat.y;
    control_signal_marker_.pose.orientation.z = quat.z;
    control_signal_marker_.pose.orientation.w = quat.w;
    control_signal_marker_.scale.x = control_signal_d_;
    control_signal_marker_.scale.y = 0.1;
    control_signal_marker_.scale.z = 0.1;
    control_signal_marker_.color.a = 1.0;
    control_signal_marker_.color.r = r;
    control_signal_marker_.color.g = g;
    control_signal_marker_.color.b = b;
    marker_control_signal_pub_.publish(control_signal_marker_);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    VehicleController vehicle_controller(nh, pnh, 100);

    chrono::steady_clock::time_point start_time, end_time;
    double loop_duration_ms;

    ros::Rate loop_rate(vehicle_controller.frequency_); // Default : 100Hz
    while(ros::ok())
    {
        /////////////////////////////////////////////////////////////////////////////////////////
        start_time = chrono::steady_clock::now();
        /////////////////////////////////////////////////////////////////////////////////////////

        vehicle_controller.update_vehicle_state();
        vehicle_controller.purepursuit();
        ros::spinOnce();
        loop_rate.sleep();
        
        /////////////////////////////////////////////////////////////////////////////////////////
        end_time = chrono::steady_clock::now();
        loop_duration_ms = 
            chrono::duration_cast<chrono::duration<double, milli>>(end_time - start_time).count();
        // [DEBUG]
        // ROS_INFO("Loop duration: %.5f s", loop_duration_ms);
        /////////////////////////////////////////////////////////////////////////////////////////

        vehicle_controller.speed_pid_controller_.dt = loop_duration_ms;
    }

    return 0;
}