#include "hi_bridge_hw/BridgeHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <cmath>
#include "std_msgs/Float32.h"

namespace legged
{
  
void writedata2file(float pos,float vel,float tau,std::string path)
{
  std::ofstream file(path, std::ios::app);
  if (!file.is_open()) {
        std::cerr << "无法打开文件用于写入\n";
        return;
    }
  file << pos << " "<<vel<<" "<<tau<<"\n";
  file.close();
}
bool BridgeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{

  root_nh.setParam("gsmp_controller_switch", "null");

  odom_sub_ = root_nh.subscribe("/imu/data", 1, &BridgeHW::OdomCallBack, this);
  if (!piHW::init(root_nh, robot_hw_nh))
    return false;

  robot_hw_nh.getParam("power_limit", powerLimit_);
   
  setupJoints();
  setupImu();
  cmd_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_pos", 10);
  cmd_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_vel", 10);
  cmd_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("cmd_ff", 10);

  read_pos_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_pos", 10);
  read_vel_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_vel", 10);
  read_ff_pub_ = robot_hw_nh.advertise<std_msgs::Float64MultiArray>("read_ff", 10);

  return true;
}


void BridgeHW::read(const ros::Time &time, const ros::Duration &period)
{
 
  float pos,vel,tau;
  for (int i=0; i<12; i++){
    motorsInterface->get_motor_state_dynamic_config(pos,vel,tau,map_index_23dof[i+1]);
    test_jointData_[i].pos_ = pos;
    test_jointData_[i].vel_ = vel;
    test_jointData_[i].tau_ = tau;  //电机真实数据
  }
  
  
  for(int i=0;i<12;i++)
  {
    jointData_[i].pos_ = test_directionMotor_[i] * test_jointData_[i].pos_;
    jointData_[i].vel_ = test_directionMotor_[i] * test_jointData_[i].vel_;
    jointData_[i].tau_ = test_directionMotor_[i] * test_jointData_[i].tau_;
  }

  float q4 = jointData_[4].pos_;
  float q5 = jointData_[5].pos_;
  float dq4 = jointData_[4].vel_;
  float dq5 = jointData_[5].vel_;
  float t4 = jointData_[4].tau_;
  float t5 = jointData_[5].tau_;

  float q10 = jointData_[10].pos_;
  float q11 = jointData_[11].pos_;
  float dq10 = jointData_[10].vel_;
  float dq11 = jointData_[11].vel_;
  float t10 = jointData_[10].tau_;
  float t11 = jointData_[11].tau_;
  
  jointData_[4].pos_ = (q4 + q5) / 2;
  jointData_[5].pos_ = (q4 - q5) / 2;
  jointData_[4].vel_ = (dq4 + dq5) / 2;
  jointData_[5].vel_ = (dq4 - dq5) / 2;
  jointData_[4].tau_ = t4 + t5;
  jointData_[5].tau_ = t4 - t5;

  jointData_[10].pos_ = (q10 + q11) / 2;
  jointData_[11].pos_ = (q10 - q11) / 2;
  jointData_[10].vel_ = (dq10 + dq11) / 2;
  jointData_[11].vel_ = (dq10 - dq11) / 2;
  jointData_[10].tau_ = q10 + q11;
  jointData_[11].tau_ = q10 - q11;

  
  imuData_.ori[0] = yesenceIMU_.orientation.x;       
  imuData_.ori[1] = yesenceIMU_.orientation.y; 
  imuData_.ori[2] = yesenceIMU_.orientation.z; 
  imuData_.ori[3] = yesenceIMU_.orientation.w; 
  imuData_.angular_vel[0] = yesenceIMU_.angular_velocity.x;  
  imuData_.angular_vel[1] = yesenceIMU_.angular_velocity.y;
  imuData_.angular_vel[2] = yesenceIMU_.angular_velocity.z;
  imuData_.linear_acc[0] = yesenceIMU_.linear_acceleration.x;   
  imuData_.linear_acc[1] = yesenceIMU_.linear_acceleration.y;
  imuData_.linear_acc[2] = yesenceIMU_.linear_acceleration.z;

}

void BridgeHW::write(const ros::Time& time, const ros::Duration& period)
{
 
  
  float q4 = jointData_[4].pos_des_;
  float q5 = jointData_[5].pos_des_;
  float dq4 = jointData_[4].vel_des_;
  float dq5 = jointData_[5].vel_des_;
  float t4 = jointData_[4].ff_;
  float t5 = jointData_[5].ff_;

  float q10 = jointData_[10].pos_des_;
  float q11 = jointData_[11].pos_des_;
  float dq10 = jointData_[10].vel_des_;
  float dq11 = jointData_[11].vel_des_;
  float t10 = jointData_[10].ff_;
  float t11 = jointData_[11].ff_;

  jointData_[4].pos_des_ = (q4 + q5);
  jointData_[5].pos_des_ = (q4 - q5);
  jointData_[4].vel_des_ = (dq4 + dq5);
  jointData_[5].vel_des_ = (dq4 - dq5);
  jointData_[4].ff_ = (t4 + t5) / 2;
  jointData_[5].ff_ = (t4 - t5) / 2;
  
  jointData_[10].pos_des_ = (q10 + q11);
  jointData_[11].pos_des_ = (q10 - q11);
  jointData_[10].vel_des_ = (dq10 + dq11);
  jointData_[11].vel_des_ = (dq10 - dq11);
  jointData_[10].ff_ = (t10 + t11) / 2;
  jointData_[11].ff_ = (t10 - t11) / 2;

  for (int i = 0; i < 12; ++i)//as the urdf rank
  {

    yksSendcmd_[i].kp_ = jointData_[i].kp_;
    yksSendcmd_[i].kd_ = jointData_[i].kd_;
    yksSendcmd_[i].pos_des_ = jointData_[i].pos_des_ * test_directionMotor_[i];//+ write_baseMotor_[i] ;
    yksSendcmd_[i].vel_des_ = jointData_[i].vel_des_ * test_directionMotor_[i];
    yksSendcmd_[i].ff_ = test_directionMotor_[i]*jointData_[i].ff_ ;

    // yksSendcmd_[i].kp_ = 0;
    // yksSendcmd_[i].kd_ = 0;
    // yksSendcmd_[i].pos_des_ = 0;
    // yksSendcmd_[i].vel_des_ = 0;
    // yksSendcmd_[i].ff_ = 0;
  }
  

  motorsInterface->fresh_cmd_dynamic_config(0, 0, 0, 30, 1,map_index_23dof[0]);  //loin_yaw
  for (int i = 0; i < 12; ++i){
    motorsInterface->fresh_cmd_dynamic_config(yksSendcmd_[i].pos_des_, yksSendcmd_[i].vel_des_, yksSendcmd_[i].ff_ * 0.4, yksSendcmd_[i].kp_, yksSendcmd_[i].kd_,map_index_23dof[i+1]);  // leg 
  }
  for (int i = 13; i < 23; ++i){
    motorsInterface->fresh_cmd_dynamic_config(0, 0, 0, 30, 1,map_index_23dof[i]);   // arm
  }

  motorsInterface->motor_send_2();
}

bool BridgeHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("leg_l") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("leg_r") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;
    if (joint.first.find("1_joint") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("2_joint") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("3_joint") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("4_joint") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("5_joint") != std::string::npos)
      joint_index = 4;
    else if (joint.first.find("6_joint") != std::string::npos)
      joint_index = 5;
    else
      continue;

    int index = leg_index * 6 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool BridgeHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

}  // namespace legged
