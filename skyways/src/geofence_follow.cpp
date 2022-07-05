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
    bool switched = false;
    mavros_msgs::State state;
    geometry_msgs::PoseArray waypoints;
    geometry_msgs::Vector3 position, velocity;
    double phi_q, theta_q, psi_q;
    std::array<double, 2> result, WGS84Position, WGS84Reference;


    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
        state = *msg; //state feedback
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        position.z = msg->pose.pose.position.z;
        tf::Quaternion quatq;
        tf::quaternionMsgToTF(msg->pose.pose.orientation, quatq);
        tf::Matrix3x3(quatq).getRPY(phi_q, theta_q, psi_q);
    }

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        WGS84Position = {msg->latitude, msg->longitude};
        result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
        ROS_INFO("%f %f", result[0], result[1]);
        position.x = result[0]; 
        position.y = result[1];
    }

    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        //Get velocity feedback
        velocity.x = msg->twist.linear.x; 
        velocity.y = msg->twist.linear.y; 
        velocity.z = msg->twist.linear.z;
    }

    void waypoints_cb(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        waypoints = *msg; //getting setpoints
    }

    void switch_cb(const std_msgs::Bool::ConstPtr& msg)
    {
        switched = msg->data; //getting switch status
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
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
            (id + "/mavros/global_position/local", 10, &StateMonitor::odom_cb, &stateMt); //Subscribe to odometry feedbacks
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
            (id + "/mavros/global_position/global", 10, &StateMonitor::gps_cb, &stateMt); //Subscribe to GPS feedbacks
    ros::Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
            (id + "/mavros/local_position/velocity_local", 10, &StateMonitor::velocity_cb, &stateMt); //Subscribe to velocity feedbacks
    ros::Subscriber switch_sub = nh.subscribe<std_msgs::Bool>
            ("/gfswitch", 100, &StateMonitor::switch_cb, &stateMt); //Subscribe to gfswitch
    ros::Publisher attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            (id + "/mavros/setpoint_raw/attitude", 10); //Publish to attitude setpoints
    ros::Publisher force_pub = nh.advertise<geometry_msgs::Vector3>
            (id + "/control_force", 10); //Publish to control force
    ros::Publisher feedback_pub = nh.advertise<geometry_msgs::Vector3>
            (id + "/feedback", 10); //Publish to feedback
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (id + "/mavros/cmd/arming"); //Create object for arming service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (id + "/mavros/set_mode"); //Create object for set mode service
    ros::Rate rate(20.0); //Set the rate of the loop

    //Declare all necessary variables
    geometry_msgs::PoseArray waypoints_global, waypoints;
    geometry_msgs::Vector3 net_F, e3, force, feedback, acceleration, acceleration_d, position_d, position, Position, velocity, velocity_d;
    int i_wp = 0; //waypoint index
    double F_t, phi_d, theta_d, psi_d = 0, theta = 0, d1, d2, curr_wp[2], nxt_wp[2], altitude_d;
    double g = 9.81, m_q = 2.9, force_scaling_factor = 0.0261, s_d = 3, R = 1, apf_y1 = -1, apf_y2 = 1, kp_apf = 1, kd_apf = 1, kp_tan = 3, kd_tan = 3, k_p = 1.5, k_d = 1.5;//Constants
    position_d.x = 0;
    altitude_d = 5;
    velocity_d.x = 0;

    // Get the setpoints from ROS 2 topic
    while(ros::ok() && waypoints_global.poses.size()==0)
    {
        waypoints_global = stateMt.waypoints;
        ROS_INFO("Waiting for waypoints");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("%d Waypoints received", (int)waypoints_global.poses.size());
    
    stateMt.WGS84Reference[0] = waypoints_global.poses[0].position.x;
    stateMt.WGS84Reference[1] = waypoints_global.poses[0].position.y;
    
    waypoints.poses.resize(waypoints_global.poses.size());
    for(int i=1; i<waypoints_global.poses.size(); i++){
      stateMt.WGS84Position = {waypoints_global.poses[i].position.x, waypoints_global.poses[i].position.y};
      stateMt.result = {wgs84::toCartesian(stateMt.WGS84Reference, stateMt.WGS84Position)};
      waypoints.poses[i].position.x = stateMt.result[0]; 
      waypoints.poses[i].position.y = stateMt.result[1];
      ROS_INFO("%f %f", waypoints.poses[i].position.x, waypoints.poses[i].position.y);
    }
    
    curr_wp[0] = waypoints.poses[0].position.x; 
    curr_wp[1] = waypoints.poses[0].position.y;
    nxt_wp[0] = waypoints.poses[1].position.x;
    nxt_wp[1] = waypoints.poses[1].position.y;
    theta = atan2((nxt_wp[1]-curr_wp[1]),(nxt_wp[0]-curr_wp[0]));
    
    // wait for FCU connection
    while(ros::ok() && !stateMt.state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // Generate blank message for publishing blank setpoints
    mavros_msgs::AttitudeTarget attitude;
    attitude.orientation.x = 0;  
    attitude.orientation.y = 0; 
    attitude.orientation.z = 0; 
    attitude.orientation.w = 1;
    attitude.thrust = 0.73; 
    attitude.type_mask = 7;

    //send a few setpoints before starting as required for offboard mode
    for(int i = 100; ros::ok() && i > 0; --i){
        attitude_pub.publish(attitude);
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
    
    // Loop to calculate and send attitude target to the vehicle via MAVROS
    int n = (int)waypoints.poses.size();
    while(ros::ok()){
        if(position_d.x < sqrt((curr_wp[0]-nxt_wp[0])*(curr_wp[0]-nxt_wp[0])+(curr_wp[1]-nxt_wp[1])*(curr_wp[1]-nxt_wp[1]))){
        if(sqrt((curr_wp[0]-nxt_wp[0])*(curr_wp[0]-nxt_wp[0])+(curr_wp[1]-nxt_wp[1])*(curr_wp[1]-nxt_wp[1])) - position_d.x < 5){
          if(velocity_d.x > 0.5){
            velocity_d.x = velocity_d.x - 0.0015;
          }
        }
        else if(velocity_d.x < s_d && stateMt.switched == true){
          velocity_d.x = velocity_d.x + 0.0015;
        }
        position_d.x = position_d.x + velocity_d.x*0.001;
      }
      else
      {
        if(abs(position.x-position_d.x)<0.3){
          i_wp++;
          if(i_wp<6){
            curr_wp[0] = waypoints.poses[i_wp].position.x;
            curr_wp[1] = waypoints.poses[i_wp].position.y;
            nxt_wp[0] = waypoints.poses[i_wp+1].position.x;; 
            nxt_wp[1] = waypoints.poses[i_wp+1].position.y;
            theta = atan2((nxt_wp[1]-curr_wp[1]),(nxt_wp[0]-curr_wp[0]));
            position_d.x = 0;
          }
          else{
            break;
          }
        }
      }
      Position.x = stateMt.position.x-curr_wp[0]; 
      Position.y = stateMt.position.y-curr_wp[1];
      position.x = Position.x*cos(theta) + Position.y*sin(theta);
      position.y = -Position.x*sin(theta) + Position.y*cos(theta);
      velocity.x = stateMt.velocity.x*cos(theta) + stateMt.velocity.y*sin(theta);
      velocity.y = -stateMt.velocity.x*sin(theta) + stateMt.velocity.y*cos(theta);

      d1 = abs(apf_y1-position.y);
      d2 = abs(apf_y2-position.y);

      double s = sqrt(stateMt.velocity.x*stateMt.velocity.x + stateMt.velocity.y*stateMt.velocity.y);  
      acceleration.y = kp_apf*((1/d1-1/R)*(1/d1-1/R))*(1-(d1-R)/abs(d1-R))-kp_apf*((1/d2-1/R)*(1/d2-1/R))*(1-(d2-R)/abs(d2-R)) - kd_apf*velocity.y;
      acceleration.x = kp_tan*(position_d.x-position.x)+kd_tan*(velocity_d.x-s);

      acceleration_d.x = acceleration.x*cos(theta) - acceleration.y*sin(theta);
      acceleration_d.y = acceleration.x*sin(theta) + acceleration.y*cos(theta);
      acceleration_d.z = k_p*(altitude_d-stateMt.position.z)-k_d*stateMt.velocity.z;

      if(stateMt.switched == false){
        acceleration_d.x = -k_p*stateMt.position.x-k_d*stateMt.velocity.x;
        acceleration_d.y = -k_p*stateMt.position.y-k_d*stateMt.velocity.y;
      }

      if(abs(acceleration_d.x)>10){
        acceleration_d.x = 10*(acceleration_d.x)/(abs(acceleration_d.x));
      }
      if(abs(acceleration_d.y)>10){
        acceleration_d.y = 10*(acceleration_d.y)/(abs(acceleration_d.y));
      }

      feedback.x = d1;
      feedback.y = d2;
      feedback.z = 0;
      force.x = acceleration.x;
      force.y = acceleration.y;
      force.z = acceleration_d.z;

      if(stateMt.position.z < 2){
        acceleration_d.x = 0;  
        acceleration_d.y = 0;
      }

      e3.x = 0; e3.y = 0; e3.z = 1;
      net_F.x = m_q*acceleration_d.x + m_q*g*e3.x;  
      net_F.y = m_q*acceleration_d.y + m_q*g*e3.y;  
      net_F.z = m_q*acceleration_d.z + m_q*g*e3.z;  
      F_t = sqrt(pow(net_F.x,2)+pow(net_F.y,2)+pow(net_F.z,2));
      phi_d = asin((net_F.x*sin(psi_d)-net_F.y*cos(psi_d))/F_t);
      theta_d = atan((net_F.x*cos(psi_d)-net_F.y*sin(psi_d))/(net_F.z));

      attitude.orientation = tf::createQuaternionMsgFromRollPitchYaw(phi_d,theta_d,psi_d);

      if(F_t>(1/force_scaling_factor)){
        attitude.thrust = 1;
      } else{
        attitude.thrust = force_scaling_factor*F_t;
      }

      force_pub.publish(force);
      feedback_pub.publish(feedback);
      attitude_pub.publish(attitude);
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}