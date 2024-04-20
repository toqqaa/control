
#include "ros.h"
#include <rospy_tutorials/Floats.h>

double RM_input = 0, RM_output = 0, RM_setpoint=000  , RM_pose_cahnge;
double LM_input = 0, LM_output = 0, LM_setpoint =000 , LM_pose_cahnge;


rospy_tutorials::Floats joint_state;




void set_angle_cb( const rospy_tutorials::Floats& vel_msg){
  
 LM_setpoint = vel_msg.data[0]; 
 RM_setpoint = vel_msg.data[1];
}
ros::Publisher pub("joint_states_from_arduino", &joint_state);
ros::Subscriber<rospy_tutorials::Floats> sub("joints_to_aurdino", set_angle_cb);
//ros::Publisher raw_imu_pub("imu_raw", &raw_imu_msg);

void ros_setup(){
  nh.initNode();
  

  nh.subscribe(sub);
  nh.advertise(pub);
//  nh.advertise(raw_imu_pub);
 
}

void ros_loop(){


pub.publish(&joint_state);
 nh.spinOnce();  
}
