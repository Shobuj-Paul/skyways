#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;

class OffboardControl
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

class stateMonitor //For state feedbacks
{
    public:
    mavros_msgs::State state;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseArray setpoints;

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
    state = *msg; //state feedback
    }

    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose = *msg; //position feedback
    }

    void setpoint_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
    setpoints = *msg; //getting setpoints
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
    stateMonitor stateMt; //Create object for state feedbacks
    OffboardControl offb_ctl; //Create object for offboard control

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            (id + "/mavros/state", 10, &stateMonitor::state_cb, &stateMt); //Subscribe to state feedbacks
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            (id + "/mavros/local_position/pose", 10, &stateMonitor::pos_cb, &stateMt); //Subscribe to position feedbacks
    ros::Subscriber setpoint_sub = nh.subscribe<geometry_msgs::PoseArray>
            (id + "/waypoints_publisher", 10, &stateMonitor::setpoint_cb, &stateMt); //Subscribe to setpoints
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            (id + "/mavros/setpoint_position/local", 10); //Publish setpoints
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            (id + "/mavros/cmd/arming"); //Create object for arming service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            (id + "/mavros/set_mode"); //Create object for set mode service

    ros::Rate rate(20.0);

    // Get the setpoints from ROS 2 topic
    geometry_msgs::PoseArray setpoints;
    while(ros::ok() && setpoints.poses.size()==0)
    {
        setpoints = stateMt.setpoints;
        ROS_INFO("Waiting for setpoints");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("%d Setpoints received", (int)setpoints.poses.size());
    
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
    int n = (int)setpoints.poses.size();
    while(ros::ok()){
        for(int i=0; i<n; i++){
            pose.pose.position.x = setpoints.poses[i].position.x;
            pose.pose.position.y = setpoints.poses[i].position.y;
            pose.pose.position.z = setpoints.poses[i].position.z;
            while(!((std::abs(stateMt.pose.pose.position.x - pose.pose.position.x) < 0.5) && (std::abs(stateMt.pose.pose.position.y - pose.pose.position.y) < 0.5) && (std::abs(stateMt.pose.pose.position.z - pose.pose.position.z) < 0.5))) //Keep publishing setpoints while comparing the current position with the setpoint within a given tolerance
            {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Setpoint reached.");
        }
        ROS_INFO("All setpoints reached.");
        while(stateMt.state.mode != "AUTO.LAND" && ros::ok()) //Land the drone once setpoints are reached
        {
        offb_ctl.land_set_mode(set_mode_client);
        ros::spinOnce();
        rate.sleep();
        }
    }
    return 0;
}