#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <skyways/WGS84toCartesian.hpp>

using namespace std;

class Autopilot
{   
    public:
    void setArm(ros::ServiceClient arming_client) //Arming Function
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        if(arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle armed");
        }
        else{
            ROS_ERROR("Failed to arm vehicle");
        }
    }

    void setDisarm(ros::ServiceClient arming_client) //Disarming Function
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = false;
        if(arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
            ROS_INFO("Vehicle disarmed");
        }
        else{
            ROS_ERROR("Failed to disarm vehicle");
        }
    }

    void offboard_set_mode(ros::ServiceClient set_mode_client) //Offboard Mode Setting Function
    {
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";
        if(set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        else{
            ROS_ERROR("Failed to set Offboard mode");
        }
    }

    void land_set_mode(ros::ServiceClient set_mode_client) //Land Mode Setting Function
    {
        mavros_msgs::SetMode land_set_mode;
        land_set_mode.request.custom_mode = "AUTO.LAND";
        if(set_mode_client.call(land_set_mode) &&
            land_set_mode.response.mode_sent){
            ROS_INFO("Landing enabled");
        }
        else{
            ROS_ERROR("Failed to set Landing mode");
        }
    }
};

class StateMonitor //For state feedbacks
{
    public:
    mavros_msgs::State state;
    geometry_msgs::PoseArray waypoints;
    geometry_msgs::TwistStamped velocity;
    geometry_msgs::Vector3Stamped force, fd_bck;
    geometry_msgs::Vector3 r_q, v_q;
    double phi_q, theta_q, psi_q;
    std::array<double, 2> result, WGS84Position, WGS84Reference;


    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        state = *msg; //state feedback
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        force.header = msg->header;
        fd_bck.header = msg->header;
        v_q.z = msg->pose.pose.position.z;
        tf::Quaternion quatq;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quatq);
        tf::Matrix3x3(quatq).getRPY(phi_q, theta_q, psi_q);
    }

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        WGS84Position = {msg->latitude, msg->longitude};
        result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
        ROS_INFO("%f %f", result[0], result[1]);
        r_q.x = result[0]; 
        r_q.y = result[1];
    }

    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        v_q.x = msg->twist.linear.x; 
        v_q.y = msg->twist.linear.y; 
        v_q.z = msg->twist.linear.z;
    }

    void waypoints_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        waypoints = *msg; //getting setpoints
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "setpoint_mission");
    if(argc != 2) //madatory condition to set Vehicle ID
    {
        ROS_ERROR("Usage: setpoint_mission <vehicle_id>");
        return 1;
    }
    string id = argv[1]; //Get the vehicle ID
    ros::NodeHandle nh; //Create object for node
    StateMonitor stateMt; //Create object for state feedbacks
    Autopilot offb_ctl; //Create object for offboard control

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (id + "/mavros/state", 10, &StateMonitor::state_cb, &stateMt); //Subscribe to state feedbacks
    ros::Subscriber setpoint_sub = nh.subscribe<geometry_msgs::PoseArray>
            (id + "/waypoints_publisher", 10, &StateMonitor::waypoints_cb, &stateMt); //Subscribe to setpoints
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (id + "/mavros/setpoint_position/local", 10); //Publish setpoints
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (id + "/mavros/cmd/arming"); //Create object for arming service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (id + "/mavros/set_mode"); //Create object for set mode service

    ros::Rate rate(20.0);

    // Get the setpoints from ROS 2 topic
    geometry_msgs::PoseArray waypoints;
    while(ros::ok() && waypoints.poses.size()==0)
    {
        waypoints = stateMt.waypoints;
        ROS_INFO("Waiting for setpoints");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("%d Setpoints received", (int)waypoints.poses.size());
    
    // wait for FCU connection
    while(ros::ok() && !stateMt.state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Generate blank message for publishing blank setpoints
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    //send a few setpoints before starting as required for offboard mode
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set offboard mode
    while(stateMt.state.mode != "OFFBOARD" && ros::ok()){
        offb_ctl.offboard_set_mode(set_mode_client);
        ros::spinOnce();
        rate.sleep();
    }

    // Arm the drone
    while(stateMt.state.armed!=true){
        offb_ctl.setArm(arming_client);
        ros::spinOnce();
        rate.sleep();
    }
    
    // Loop to send the setpoints to the vehicle via MAVROS
    int n = (int)waypoints.poses.size();
    while(ros::ok()){}
    return 0;
}