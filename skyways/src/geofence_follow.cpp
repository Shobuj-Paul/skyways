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
#include <skyways/DataPacketResponse.h>

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

class StateMonitor
{
    public:
    mavros_msgs::State state;
    geometry_msgs::Vector3 position, velocity;
    skyways::DataPacketResponse packet;
    bool switched = false;
    double phi, theta, psi;
    std::array<double, 2> result, WGS84Position, WGS84Reference;

    void state_cb(const mavros_msgs::State::ConstPtr& msg)
    {
      state = *msg;
    }

    void pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
      position.z = msg->pose.pose.position.z;
      tf::Quaternion quaternion;
      tf::quaternionMsgToTF(msg->pose.pose.orientation, quaternion);
      tf::Matrix3x3(quaternion).getRPY(phi, theta, psi);
    }

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
      WGS84Position = {msg->latitude, msg->longitude};
      result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
      position.x = result[0]; 
      position.y = result[1];
      ROS_INFO("%f %f", position.x, position.y);
    }

    void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
      velocity.x = msg->twist.linear.x; 
      velocity.x = msg->twist.linear.y; 
      velocity.z = msg->twist.linear.z;
    }

    void switch_cb(const std_msgs::Bool::ConstPtr& msg)
    {
      if(msg->data == true)
      {
        switched = true;
      }
    }

    void packet_cb(const skyways::DataPacketResponse::ConstPtr& msg){
        packet = *msg; //getting setpoints
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ofb_apf1");
    ros::NodeHandle n;
    StateMonitor stateMt;
    Autopilot offb_ctl;

    int i_wp = 0;
    double phi_d, theta_d, psi_d;
    double p_vxq_d = 0, theta = 0, p_xq_d = 0;
    double F_t, d1, d2, speed, speed_d, altitude_d, R;
    const double k_p = 1.5, k_d = 1.5, g = 9.81, m_q = 2.9, force_scaling_factor = 0.0261, apf_y1 = -1, apf_y2 = 1, kp_apf = 1, kd_apf = 1, kp_tan = 3, kd_tan = 3;
    std::array<double, 2> result, WGS84Position, WGS84Reference{0, 0};
    geometry_msgs::Vector3 e3, acceleration_d, net_F, force, feedback, p_a, p_q, P_q, p_vq;
    geometry_msgs::Vector3 curr_wp, next_wp;
    geometry_msgs::PoseArray waypoints_global, waypoints;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    if(argc!=2)
    {
      ROS_ERROR("Usage: rosrun skyways geofence_rectangle <vehicle_id>");
    }
    std::string id = argv[1];

    ros::Subscriber position_sub = n.subscribe<nav_msgs::Odometry>
          (id + "/mavros/global_position/local", 1, &StateMonitor::pos_cb, &stateMt);
    ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>
          (id + "/mavros/global_position/global", 1, &StateMonitor::gps_cb, &stateMt);
    ros::Subscriber velocity_sub = n.subscribe<geometry_msgs::TwistStamped>
          (id + "/mavros/local_position/velocity_local", 1, &StateMonitor::velocity_cb, &stateMt);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
          (id + "/mavros/state", 100, &StateMonitor::state_cb, &stateMt);
    ros::Subscriber switch_sub = n.subscribe<std_msgs::Bool>
          ("/gfswitch", 100, &StateMonitor::switch_cb, &stateMt);
    ros::Subscriber packet_sub = n.subscribe<skyways::DataPacketResponse>
          (id + "/data_packet", 10, &StateMonitor::packet_cb, &stateMt);
    ros::Publisher attitude_pub = n.advertise<mavros_msgs::AttitudeTarget>
          (id + "/mavros/setpoint_raw/attitude", 10);
    ros::Publisher force_pub = n.advertise<geometry_msgs::Vector3Stamped>
        (id + "/control_force", 10);
    ros::Publisher feedback_pub = n.advertise<geometry_msgs::Vector3Stamped>
          (id + "/feedback", 10);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
          (id + "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
          (id + "/mavros/set_mode");
    ros::Rate rate(1000);

    // Get the data packet parameters and waypoints
    while(stateMt.packet.waypoints.poses.size()==0)
    {
      speed_d = stateMt.packet.vel_mag;
      altitude_d = stateMt.packet.altitude;
      R = stateMt.packet.geofence_radius;
      ROS_INFO("Waiting for waypoints");
      ros::spinOnce();
      rate.sleep();
    }

    int size = stateMt.packet.waypoints.poses.size();
    waypoints_global.poses.resize(size);
    for (int i=0; i<size; i++)
    {
      waypoints_global.poses[i].position.x = stateMt.packet.waypoints.poses[i].position.x;
      waypoints_global.poses[i].position.y = stateMt.packet.waypoints.poses[i].position.y;
    }
    
    WGS84Reference[0] = waypoints_global.poses[0].position.x;
    WGS84Reference[1] = waypoints_global.poses[0].position.y;
    waypoints.poses.resize(size);
    for(int i=1; i<size; i++)
    {
      WGS84Position = {waypoints_global.poses[i].position.x, waypoints_global.poses[i].position.y};
      result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
      waypoints.poses[i].position.x = result[0]; 
      waypoints.poses[i].position.y = result[1];
      ROS_INFO("%f %f", waypoints.poses[i].position.x, waypoints.poses[i].position.y);
    }

    curr_wp.x = waypoints.poses[0].position.x;
    curr_wp.y = waypoints.poses[0].position.y;
    next_wp.x = waypoints.poses[1].position.x; 
    next_wp.y = waypoints.poses[1].position.y;
    theta = atan2((next_wp.x-curr_wp.y),(next_wp.x-curr_wp.y));
    
    while(ros::ok() && !stateMt.state.connected)
    {
      ros::spinOnce();
      rate.sleep();
    }
    
    // Declare attitude message
    mavros_msgs::AttitudeTarget attitude;
    attitude.orientation.x = 0;  
    attitude.orientation.y = 0; 
    attitude.orientation.z = 0; 
    attitude.orientation.w = 1;
    attitude.thrust = 0.73; 
    attitude.type_mask = 7;

    // Send setpoints before starting
    for(int i=100; ros::ok() && i>0; --i){
      attitude_pub.publish(attitude);
      ros::spinOnce();
      rate.sleep();
    }
    
    // Set offboard mode
    ros::Time last_request = ros::Time::now();
    while(stateMt.state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0))
    {
        offb_ctl.offboard_set_mode(set_mode_client);
        ros::spinOnce();
        rate.sleep();
    }

    // Arm the drone
    while(stateMt.state.armed!=true && ros::Time::now() - last_request > ros::Duration(5.0))
    {
        offb_ctl.setArm(arming_client);
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok())
    {
        float dist = sqrt(pow((curr_wp.x-next_wp.x),2)+pow((curr_wp.y-next_wp.y),2));
        if(p_xq_d < dist)
        {
            if(dist - p_xq_d < 5 && p_vxq_d > 0.5)
            {
                p_vxq_d = p_vxq_d - 0.0015;
            }
            else if(p_vxq_d < speed_d && stateMt.switched == true)
            {
                p_vxq_d = p_vxq_d + 0.0015;
            }
            p_xq_d = p_xq_d + p_vxq_d*0.001;
        }
        else
        {
            if(abs(p_q.x-p_xq_d)<0.3)
            {
                i_wp++;
                if(i_wp<size)
                {
                    curr_wp.x = waypoints.poses[i_wp].position.x; 
                    curr_wp.y = waypoints.poses[i_wp].position.y;
                    next_wp.x = waypoints.poses[i_wp+1].position.x;
                    next_wp.y = waypoints.poses[i_wp+1].position.y;
                    theta = atan2((next_wp.y-curr_wp.y),(next_wp.x-curr_wp.x));
                    p_xq_d = 0;
                }
                else
                {
                  break;
                }
            } 
        }

        P_q.x = stateMt.position.x-curr_wp.x; 
        P_q.y = stateMt.position.y-curr_wp.y;
        p_q.x = P_q.x*cos(theta)+P_q.y*sin(theta);
        p_q.y = -P_q.x*sin(theta)+P_q.y*cos(theta);
        p_vq.x = stateMt.velocity.x*cos(theta)+stateMt.velocity.y*sin(theta);
        p_vq.y = -stateMt.velocity.x*sin(theta)+stateMt.velocity.y*cos(theta);

        d1 = abs(apf_y1-p_q.y);
        d2 = abs(apf_y2-p_q.y);

        p_a.y = kp_apf*((1/d1-1/R)*(1/d1-1/R))*(1-(d1-R)/abs(d1-R))-kp_apf*((1/d2-1/R)*(1/d2-1/R))*(1-(d2-R)/abs(d2-R)) - kd_apf*p_vq.y;
        speed = sqrt(stateMt.velocity.x*stateMt.velocity.x+stateMt.velocity.y*stateMt.velocity.y);
        p_a.x = kp_tan*(p_xq_d-p_q.x)+kd_tan*(p_vxq_d-speed);

        acceleration_d.x = p_a.x*cos(theta)-p_a.y*sin(theta);
        acceleration_d.y = p_a.x*sin(theta)+p_a.y*cos(theta);
        acceleration_d.z = k_p*(altitude_d-stateMt.position.z)-k_d*stateMt.velocity.z;

        if(stateMt.switched == false)
        {
          acceleration_d.x = -k_p*stateMt.position.x-k_d*stateMt.velocity.x;
          acceleration_d.y = -k_p*stateMt.position.y-k_d*stateMt.velocity.y;
        }

        if(abs(acceleration_d.x)>10)
        {
          acceleration_d.x = 10*(acceleration_d.x)/(abs(acceleration_d.x));
        }
        if(abs(acceleration_d.y)>10)
        {
          acceleration_d.y = 10*(acceleration_d.y)/(abs(acceleration_d.y));
        }

        feedback.x = d1;
        feedback.y = d2;
        feedback.z = 0;
        force.x = p_a.x;
        force.y = p_a.y;
        force.z = acceleration_d.z;

        if(stateMt.position.z < 2){
          acceleration_d.x = 0;  acceleration_d.y = 0;
        }

        e3.x = 0; //Unit Vector
        e3.y = 0; 
        e3.z = 1;
        net_F.x = m_q*acceleration_d.x + m_q*g*e3.x;  
        net_F.y = m_q*acceleration_d.y + m_q*g*e3.y;  
        net_F.z = m_q*acceleration_d.z + m_q*g*e3.z;  
        F_t = sqrt(pow(net_F.x,2)+pow(net_F.y,2)+pow(net_F.z,2));
        psi_d = 0;
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