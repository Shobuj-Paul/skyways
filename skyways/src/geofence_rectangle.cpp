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

int i_wp = 0, GFswitch = 0, ofb_switch;
double xq, yq, zq, vxq, vyq, vzq, phi_q, theta_q, psi_q;
double p_xq_d = 0, yq_d = 0, zq_d = 5, phi_d, theta_d, psi_d = 0, s_d = 3, vyq_d = 0, vzq_d = 0;
double p_xq, p_yq, P_xq, P_yq, p_vxq, p_vyq, p_vxq_d = 0, s, p_a_x, p_a_y, theta = 0, R = 1, d1, d2, apf_y1 = -1, apf_y2 = 1, kp_apf = 1, kd_apf = 1, kp_tan = 3, kd_tan = 3;
double F_t, g = 9.81, m_q = 2.9, force_scaling_factor = 0.0261;
double k_p = 1.5, k_d = 1.5;
double wp[5][2], curr_wp[2], nxt_wp[2];
//double Gwp[5][2] = {{13.0271702, 77.5636031}, {13.0273507, 77.5636269}, {13.0272683, 77.5642817}, {13.0270861, 77.5642474},{13.0271702, 77.5636031}};
double Gwp[100][2];
std::array<double, 2> result, WGS84Position, WGS84Reference{0, 0};
geometry_msgs::Vector3 e3, aq_d, net_F;
geometry_msgs::Vector3Stamped force;
geometry_msgs::Vector3Stamped fd_bck;
geometry_msgs::PoseArray waypoints;

mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget acc;
mavros_msgs::SetMode offb_set_mode;
mavros_msgs::CommandBool arm_cmd;

ros::Time last_request;

void state_cb(const mavros_msgs::State::ConstPtr& msg1){
  current_state = *msg1;
}

void PosCallback(nav_msgs::Odometry quad_pos){
  force.header = quad_pos.header;
  fd_bck.header = quad_pos.header;
  //xq = quad_pos.pose.pose.position.x;  
  //yq = quad_pos.pose.pose.position.y;
  zq = quad_pos.pose.pose.position.z;
  tf::Quaternion quatq;
  tf::quaternionMsgToTF(quad_pos.pose.pose.orientation, quatq);
  tf::Matrix3x3(quatq).getRPY(phi_q, theta_q, psi_q);
}

void GposCallback(sensor_msgs::NavSatFix quad_Gpos){
  WGS84Position = {quad_Gpos.latitude, quad_Gpos.longitude};
  result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
  ROS_INFO("%f %f", result[0], result[1]);
  xq = result[0]; yq = result[1];
}

void TwistCallback(geometry_msgs::TwistStamped quad_twist){
  vxq = quad_twist.twist.linear.x; vyq = quad_twist.twist.linear.y; vzq = quad_twist.twist.linear.z;
}

void GFswitch_Callback(std_msgs::Bool Data){
  if(Data.data == true){
    GFswitch = 1;
    //p_vxq_d = s_d;
  }
}

void waypoint_cb(const geometry_msgs::PoseArray::ConstPtr& msg){
    waypoints = *msg; //getting setpoints
}

int main(int argc, char **argv){
  ros::init(argc, argv, "ofb_apf1");
  ros::NodeHandle n;
  ros::Subscriber quad_pos, quad_twist, state_sub, quad_Gpos, GFswitch_sub, waypoint_sub;
  ros::Publisher mav_pub, control_pub, feedback_pub;
  ros::ServiceClient arming_client, set_mode_client;
  if(argc!=2)
  {
    ROS_ERROR("Usage: rosrun skyways geofence_rectangle <vehicle_id>");
  }
  std::string vehicle_id = argv[1];

  quad_pos = n.subscribe("/mavros/global_position/local", 1, PosCallback);
  quad_Gpos = n.subscribe("/mavros/global_position/global", 1, GposCallback);
  quad_twist = n.subscribe("/mavros/local_position/velocity_local", 1, TwistCallback);
  state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
  GFswitch_sub = n.subscribe<std_msgs::Bool>("gfswitch", 100, GFswitch_Callback);
  mav_pub = n.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  control_pub = n.advertise<geometry_msgs::Vector3Stamped>("control_force", 10);
  feedback_pub = n.advertise<geometry_msgs::Vector3Stamped>("feedback", 10);
  arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  waypoint_sub = n.subscribe<geometry_msgs::PoseArray>(vehicle_id + "/waypoints_publisher", 10, waypoint_cb);

  ros::Rate loop_rate(1000);
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    loop_rate.sleep();
  }

  while(waypoints.poses.size()==0)
  {
    ROS_INFO("Waiting for waypoints");
    ros::spinOnce();
    loop_rate.sleep();
  }

  int size = waypoints.poses.size();
  for (int i=0; i<size; i++)
  {
    Gwp[i][0] = waypoints.poses[i].position.x;
    Gwp[i][1] = waypoints.poses[i].position.y;
  }
  WGS84Reference[0] = Gwp[0][0];
  WGS84Reference[1] = Gwp[0][1];

  for(int i=1; i<5; i++){
    WGS84Position = {Gwp[i][0], Gwp[i][1]};
    result = {wgs84::toCartesian(WGS84Reference, WGS84Position)};
    wp[i][0] = result[0]; 
    wp[i][1] = result[1];
    ROS_INFO("%f %f", wp[i][0], wp[i][1]);
  }

  curr_wp[0] = wp[0][0]; curr_wp[1] = wp[0][1]; nxt_wp[0] = wp[1][0]; nxt_wp[1] = wp[1][1];
  theta = atan2((nxt_wp[1]-curr_wp[1]),(nxt_wp[0]-curr_wp[0]));
  //ROS_INFO("%f", theta);
  acc.orientation.x = 0;  acc.orientation.y = 0; acc.orientation.z = 0; acc.orientation.w = 1;
  acc.thrust = 0.73; acc.type_mask = 7;

  for(int i=100; ros::ok() && i>0; --i){
    mav_pub.publish(acc);
    ros::spinOnce();
    loop_rate.sleep();
  }

  offb_set_mode.request.custom_mode = "OFFBOARD";
  arm_cmd.request.value = true;
  last_request = ros::Time::now();

  while(ros::ok()){
    if( current_state.mode != "OFFBOARD" &&
    (ros::Time::now() - last_request > ros::Duration(5.0)) && ofb_switch == 0){
      if( set_mode_client.call(offb_set_mode) &&
      offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
        ofb_switch = 1;
      }
      last_request = ros::Time::now();
    } 
    else
    {
      if(!current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( arming_client.call(arm_cmd) &&
          arm_cmd.response.success){
            ROS_INFO("UAV armed");
          }
          last_request = ros::Time::now();
        }
    }
      if(p_xq_d < sqrt((curr_wp[0]-nxt_wp[0])*(curr_wp[0]-nxt_wp[0])+(curr_wp[1]-nxt_wp[1])*(curr_wp[1]-nxt_wp[1]))){
        if(sqrt((curr_wp[0]-nxt_wp[0])*(curr_wp[0]-nxt_wp[0])+(curr_wp[1]-nxt_wp[1])*(curr_wp[1]-nxt_wp[1])) - p_xq_d < 5){
          if(p_vxq_d > 0.5){
            p_vxq_d = p_vxq_d - 0.0015;
          }
        }
        else if(p_vxq_d < s_d && GFswitch == 1){
          p_vxq_d = p_vxq_d + 0.0015;
        }
        //ROS_INFO("%f %f %f",p_xq_d, p_vxq_d, sqrt((curr_wp[0]-nxt_wp[0])*(curr_wp[0]-nxt_wp[0])+(curr_wp[1]-nxt_wp[1])*(curr_wp[1]-nxt_wp[1])));
        p_xq_d = p_xq_d + p_vxq_d*0.001;
      }
      else
      {
        if(abs(p_xq-p_xq_d)<0.3){
          i_wp++;
          if(i_wp<6){
            curr_wp[0] = wp[i_wp][0]; curr_wp[1] = wp[i_wp][1];
            nxt_wp[0] = wp[i_wp+1][0]; nxt_wp[1] = wp[i_wp+1][1];
            theta = atan2((nxt_wp[1]-curr_wp[1]),(nxt_wp[0]-curr_wp[0]));
            p_xq_d = 0;
          }else{
            break;
          }
        }
      }
      //ROS_INFO("%f %f %f", p_xq, p_xq_d, P_xq);
      P_xq = xq-curr_wp[0]; P_yq = yq-curr_wp[1];
      p_xq = P_xq*cos(theta)+P_yq*sin(theta);
      p_yq = -P_xq*sin(theta)+P_yq*cos(theta);
      p_vxq = vxq*cos(theta)+vyq*sin(theta);
      p_vyq = -vxq*sin(theta)+vyq*cos(theta);

      d1 = abs(apf_y1-p_yq);
      d2 = abs(apf_y2-p_yq);

      p_a_y = kp_apf*((1/d1-1/R)*(1/d1-1/R))*(1-(d1-R)/abs(d1-R))-kp_apf*((1/d2-1/R)*(1/d2-1/R))*(1-(d2-R)/abs(d2-R)) - kd_apf*p_vyq;
      s = sqrt(vxq*vxq+vyq*vyq);
      p_a_x = kp_tan*(p_xq_d-p_xq)+kd_tan*(p_vxq_d-s);

      //ROS_INFO("%f %f", p_xq_d, p_vxq_d);

      aq_d.x = p_a_x*cos(theta)-p_a_y*sin(theta);
      aq_d.y = p_a_x*sin(theta)+p_a_y*cos(theta);
      aq_d.z = k_p*(zq_d-zq)-k_d*vzq;

      if(GFswitch == 0){
        aq_d.x = -k_p*xq-k_d*vxq;
        aq_d.y = -k_p*yq-k_d*vyq;
        //ROS_INFO("%f %f", aq_d.X(), aq_d.Y());
      }

      if(abs(aq_d.x)>10){
        aq_d.x = 10*(aq_d.x)/(abs(aq_d.x));
      }
      if(abs(aq_d.y)>10){
        aq_d.y = 10*(aq_d.y)/(abs(aq_d.y));
      }

      fd_bck.vector.x = d1;
      fd_bck.vector.y = d2;
      fd_bck.vector.z = 0;
      force.vector.x = p_a_x;
      force.vector.y = p_a_y;
      force.vector.z = aq_d.z;

      if(zq < 2){
        aq_d.x = 0;  aq_d.y = 0;
      }

      e3.x = 0; e3.y = 0; e3.z = 1;
      net_F.x = m_q*aq_d.x + m_q*g*e3.x;  
      net_F.y = m_q*aq_d.y + m_q*g*e3.y;  
      net_F.z = m_q*aq_d.z + m_q*g*e3.z;  
      F_t = sqrt(pow(net_F.x,2)+pow(net_F.y,2)+pow(net_F.z,2));
      phi_d = asin((net_F.x*sin(psi_d)-net_F.y*cos(psi_d))/F_t);
      theta_d = atan((net_F.x*cos(psi_d)-net_F.y*sin(psi_d))/(net_F.z));

      acc.orientation = tf::createQuaternionMsgFromRollPitchYaw(phi_d,theta_d,psi_d);

      if(F_t>(1/force_scaling_factor)){
        acc.thrust = 1;
      } else{
        acc.thrust = force_scaling_factor*F_t;
      }

      control_pub.publish(force);
      feedback_pub.publish(fd_bck);
      mav_pub.publish(acc);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
