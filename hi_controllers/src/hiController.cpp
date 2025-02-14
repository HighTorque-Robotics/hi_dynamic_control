//
// Created by qiayuan on 2022/6/24.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, hightorque Robotics.Co.Ltd. All rights reserved.

For further information, contact: service@hightorque.cn or visit our website
at https://www.hightorque.cn/.
********************************************************************************/

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "hi_controllers/hiController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <angles/angles.h>
#include <hi_estimation/FromTopiceEstimate.h>
#include <hi_estimation/LinearKalmanFilter.h>
#include <hi_wbc/WeightedWbc.h>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/Float64MultiArray.h"
#include "hi_controllers/utilities.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <chrono>

#include "hi_interface/foot_planner/InverseKinematics.h"

namespace legged
{
bool hiController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh)
{
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(HiInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(
      HiInterface_->getPinocchioInterface(), pinocchioMapping, HiInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<hiRobotVisualizer>(
      HiInterface_->getPinocchioInterface(), HiInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new piSelfCollisionVisualization(
      HiInterface_->getPinocchioInterface(), HiInterface_->getGeometryInterface(), pinocchioMapping, nh));

  //////////////////////////////////////////////////////////////////////////////////

  pos_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_position", 10);
  vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_velocity", 10);
  torque_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("joint_torque", 10);

  body_pose_pub_ = controller_nh.advertise<geometry_msgs::PoseStamped>("pose_stamped", 10);
  imu_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("imu", 10);

  cmd_pos_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_position", 10);
  cmd_vel_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_velocity", 10);
  cmd_tau_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("cmd_joint_torque", 10);
  //////////////////////////////////////////////////////////////////////////////////


  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HiInterface_->getPinocchioInterface(),
                                                                    HiInterface_->getCentroidalModelInfo());

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();

  std::vector<std::string> joint_names{
    "leg_l1_joint", "leg_l2_joint", "leg_l3_joint", "leg_l4_joint", "leg_l5_joint","leg_l6_joint",
    "leg_r1_joint", "leg_r2_joint", "leg_r3_joint", "leg_r4_joint", "leg_r5_joint","leg_r6_joint"
  };
  for (const auto& joint_name : joint_names)
  {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("imu_link");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(HiInterface_->getPinocchioInterface(),
                                       HiInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controller_nh);
  wbc_->loadTasksSetting(taskFile, verbose);
  wbc_->setStanceMode(true);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(HiInterface_->getCentroidalModelInfo());

  // Configuring the hardware interface
  eeKinematicsPtr_->setPinocchioInterface(HiInterface_->getPinocchioInterface());

  // Reading relevant parameters
  RetrievingParameters();

  // loadEigenMatrix
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defalutJointPos_);

  // Configuring an inverse kinematics processing object
  inverseKinematics_.setParam(std::make_shared<PinocchioInterface>(HiInterface_->getPinocchioInterface()),
                              std::make_shared<CentroidalModelInfo>(HiInterface_->getCentroidalModelInfo()));

  info_ = HiInterface_->getCentroidalModelInfo();

  return true;
}

void hiController::starting(const ros::Time& time)
{
  startingTime_.fromSec(time.toSec() - 0.0001);
  const ros::Time shifted_time = time - startingTime_;
  // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);

  mpcRunning_ = false;

  // Mode Subscribe
  ModeSubscribe();

  // Dynamic server
  serverPtr_ =
      std::make_unique<dynamic_reconfigure::Server<hi_controllers::TutorialsConfig>>(ros::NodeHandle("controller"));
  dynamic_reconfigure::Server<hi_controllers::TutorialsConfig>::CallbackType f;
  f = boost::bind(&hiController::dynamicParamCallback, this, _1, _2);
  serverPtr_->setCallback(f);
}

void hiController::update(const ros::Time& time, const ros::Duration& period)
{
  const ros::Time shifted_time = time - startingTime_;
  // State Estimate
  updateStateEstimation(shifted_time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Evaluate the current policy
  vector_t optimizedState(stateDim_), optimizedInput(inputDim_);
  size_t plannedMode = 3;
  bool mpc_updated_ = false;

  if (firstStartMpc_)
  {
    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState,
                                     optimizedInput, plannedMode);
    currentObservation_.input = optimizedInput;
    mpc_updated_ = true;
  }

  if (setWalkFlag_)
  {
    // wbc_->setStanceMode(false);
    plannedMode = 0;
  }
  else
  {
    optimizedState.setZero();
    // optimizedInput.setZero();
    optimizedState.segment(6, 6) = currentObservation_.state.segment<6>(6);
    optimizedState.segment(6 + 6, jointDim_) = defalutJointPos_;
    // wbc_->setStanceMode(true);
  }
  const vector_t& mpc_planned_body_pos = optimizedState.segment(6, 6);
  const vector_t& mpc_planned_joint_pos = optimizedState.segment(6 + 6, jointDim_);
  const vector_t& mpc_planned_joint_vel = optimizedInput.segment(24, jointDim_);
  // std::cerr<<"足底力："<<optimizedInput.segment(0,3)<<std::endl;
  // WBC
  wbcTimer_.startTimer();
  // std::cout<<"plannedMode:"<<plannedMode<<std::endl;
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  const vector_t& wbc_planned_torque = x.tail(jointDim_);
  const vector_t& wbc_planned_joint_acc = x.segment(6, jointDim_);
  const vector_t& wbc_planned_body_acc = x.head(6);
  const vector_t& wbc_planned_contact_force = x.segment(6 + jointDim_, wbc_->getContactForceSize());
  wbcTimer_.endTimer();

  vector_t mpc_planned_joint_force = optimizedInput.head(24);  
  vector_t mpc_leg_force = vector_t(24);
  mpc_leg_force.segment(0,6) = mpc_planned_joint_force.head(6);
  mpc_leg_force.segment(6,6) = mpc_planned_joint_force.segment(12,6);
  mpc_leg_force.segment(12,6) = mpc_planned_joint_force.segment(6,6);  
  mpc_leg_force.segment(18,6) = mpc_planned_joint_force.segment(18,6);    

  vector_t mpc_planned_joint_tau;
  mpc_planned_joint_tau.setZero(12);
  matrix_t Jac_c_left = matrix_t(12, info_.generalizedCoordinatesNum);
  matrix_t Jac_c_right = matrix_t(12, info_.generalizedCoordinatesNum);

  Jac_c_left.block(0,0,6,info_.generalizedCoordinatesNum) = wbc_->getContactJacobi().block(0, 0, 6, info_.generalizedCoordinatesNum);
  Jac_c_left.block(6,0,6,info_.generalizedCoordinatesNum) = wbc_->getContactJacobi().block(12, 0, 6, info_.generalizedCoordinatesNum);
  Jac_c_right.block(0,0,6,info_.generalizedCoordinatesNum) = wbc_->getContactJacobi().block(6, 0, 6, info_.generalizedCoordinatesNum);
  Jac_c_right.block(6,0,6,info_.generalizedCoordinatesNum) = wbc_->getContactJacobi().block(18, 0, 6, info_.generalizedCoordinatesNum);

  matrix_t Jac_leg_left = matrix_t(12,6);
  matrix_t Jac_leg_right = matrix_t(12,6);

  Jac_leg_left = Jac_c_left.block(0, 6 , 12, 6);
  Jac_leg_right = Jac_c_right.block(0, 12 , 12, 6);

  mpc_planned_joint_tau.segment(0,6) = -Jac_leg_left.transpose() * mpc_leg_force.segment(0,12);
  mpc_planned_joint_tau.segment(6,6) = -Jac_leg_right.transpose() * mpc_leg_force.segment(12,12);


  posDes_ = centroidal_model::getJointAngles(optimizedState, HiInterface_->getCentroidalModelInfo());
  velDes_ = centroidal_model::getJointVelocities(optimizedInput, HiInterface_->getCentroidalModelInfo());

  scalar_t dt = period.toSec();

  velDes_ = velDes_ + wbc_planned_joint_acc * dt;

  posDes_ = posDes_ + 0.5 * wbc_planned_joint_acc * dt * dt + velDes_ * dt;

  vector_t output_torque(jointDim_);


  //*********************** Set Joint Command: Normal Tracking *****************************//
  std_msgs::Float64MultiArray ctrl_pos_msg, ctrl_vel_msg, ctrl_tau_msg;
  ctrl_pos_msg.data.resize(12);
  ctrl_vel_msg.data.resize(12);
  ctrl_tau_msg.data.resize(12);
  for (size_t j = 0; j < jointDim_; ++j)
  {
    //"Limit protection
    const auto& model = HiInterface_->getPinocchioInterface().getModel();
    double lower_bound = model.lowerPositionLimit(6 + j);
    double upper_bound = model.upperPositionLimit(6 + j);
    if (!emergencyStopFlag_ && loadControllerFlag_ &&
        (hybridJointHandles_[j].getPosition() > upper_bound + 0.02 ||
         hybridJointHandles_[j].getPosition() < lower_bound - 0.02))
    {
      emergencyStopFlag_ = true;
      std::cerr << "Reach Position Limit!!!!!!!!!!!!!!!!!!!!!!!! " << j << ":" << hybridJointHandles_[j].getPosition()
                << std::endl;
    }
    if (!loadControllerFlag_)
    {
      if (j == 4 || j == 5 || j == 10 || j == 11)
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_feet, 0);
      }
      else
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j], kp_position, kd_position,
                                          0);
      }
      ctrl_pos_msg.data[j] = mpc_planned_joint_pos[j];
      ctrl_vel_msg.data[j] = mpc_planned_joint_vel[j];
      ctrl_tau_msg.data[j] = 0;
    }
    else
    {
      contact_flag_t cmdContactFlag = modeNumber2StanceLeg(
          mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
      if (j == 0 || j == 1 || j == 6 || j == 7)
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                          cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_small,
                                          mpc_planned_joint_tau(j));
      }
      else if (j == 4 || j == 5 || j == 10 || j == 11)
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                          cmdContactFlag[int(j / 6)] ? kp_small_stance : kp_small_swing, kd_feet,
                                          mpc_planned_joint_tau(j));
      }
      else
      {
        hybridJointHandles_[j].setCommand(mpc_planned_joint_pos[j], mpc_planned_joint_vel[j],
                                          cmdContactFlag[int(j / 6)] ? kp_big_stance : kp_big_swing, kd_big,
                                          mpc_planned_joint_tau(j));
      }
      ctrl_pos_msg.data[j] = mpc_planned_joint_pos[j];
      ctrl_vel_msg.data[j] = mpc_planned_joint_vel[j];
      ctrl_tau_msg.data[j] = mpc_planned_joint_tau(j);
    }
    if (emergencyStopFlag_)
    {
      hybridJointHandles_[j].setCommand(0, 0, 0, 1, 0);
    }
    posDesOutput_(j) = hybridJointHandles_[j].getPositionDesired();
    velDesOutput_(j) = hybridJointHandles_[j].getVelocityDesired();

    output_torque(j) = hybridJointHandles_[j].getFeedforward() +
                       hybridJointHandles_[j].getKp() *
                           (hybridJointHandles_[j].getPositionDesired() - hybridJointHandles_[j].getPosition()) +
                       hybridJointHandles_[j].getKd() *
                           (hybridJointHandles_[j].getVelocityDesired() - hybridJointHandles_[j].getVelocity());
  
    }
  //*********************** Set Joint Command: Torque Tracking Test *****************************//

  CommandData command_data;
  vector_t planned_state = currentObservation_.state;
  vector_t planned_input = currentObservation_.input;
  planned_state.tail(jointDim_) = posDesOutput_;
  planned_input.tail(jointDim_) = velDesOutput_;
  command_data.mpcTargetTrajectories_.timeTrajectory.push_back(currentObservation_.time);
  command_data.mpcTargetTrajectories_.stateTrajectory.push_back(planned_state);
  command_data.mpcTargetTrajectories_.inputTrajectory.push_back(planned_input);
  command_data = mpc_updated_ ? mpcMrtInterface_->getCommand() : command_data;
  PrimalSolution primal_solution = mpc_updated_ ? mpcMrtInterface_->getPolicy() : PrimalSolution();

  // Visualization
  robotVisualizer_->update(currentObservation_, primal_solution, command_data,
                           HiInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
  cmd_pos_pub_.publish(ctrl_pos_msg);
  cmd_vel_pub_.publish(ctrl_vel_msg);
  cmd_tau_pub_.publish(ctrl_tau_msg);
}

void hiController::updateStateEstimation(const ros::Time& time, const ros::Duration& period)
{
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size()),
      jointTor(hybridJointHandles_.size());
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t cmdContactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////

  std_msgs::Float64MultiArray torque_msg, pos_msg, vel_msg;
  torque_msg.data.resize(12);
  pos_msg.data.resize(12);
  vel_msg.data.resize(12);

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i)
  {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
    jointTor(i) = hybridJointHandles_[i].getEffort();

    pos_msg.data[i] = jointPos(i);
    vel_msg.data[i] = jointVel(i);    
    torque_msg.data[i] = jointTor(i);
  }

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "odom";
  pose_stamped.pose.position.x = currentObservation_.state(6);
  pose_stamped.pose.position.y = currentObservation_.state(7);
  pose_stamped.pose.position.z = currentObservation_.state(8);
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;
  pose_stamped.header.stamp = ros::Time::now(); 

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  cmdContactFlag = modeNumber2StanceLeg(
      mpcMrtInterface_->getReferenceManager().getModeSchedule().modeAtTime(currentObservation_.time));
  if (!firstStartMpc_)
  {
    for (size_t i = 0; i < 4; ++i)
    {
      cmdContactFlag[i] = true;
    }
  }
  stateEstimate_->updateCmdContact(cmdContactFlag);
  stateEstimate_->setStartStopTime4Legs(
      HiInterface_->getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner()->threadSaftyGetStartStopTime(
          currentObservation_.time));


  std_msgs::Float64MultiArray array;

  for (size_t i = 0; i < 4; ++i)
  {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
    array.data.push_back(quat.coeffs()(i));
  }

  for (size_t i = 0; i < 3; ++i)
  {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    array.data.push_back(angularVel(i));
  }

    for (size_t i = 0; i < 3; ++i)
  {
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
    array.data.push_back(linearAccel(i));
  }

  for (size_t i = 0; i < 9; ++i)
  {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  imu_pub_.publish(array);

  //////////////////////////////////////////////////////////////////////////////////////////////////
  static std::chrono::steady_clock::time_point last_publish_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  if (current_time - last_publish_time > std::chrono::milliseconds(100)) {
    pos_pub_.publish(pos_msg);
    vel_pub_.publish(vel_msg);
    torque_pub_.publish(torque_msg);
    body_pose_pub_.publish(pose_stamped);
    
    ROS_INFO_STREAM("current Height: " << currentObservation_.state(8));



    last_publish_time = current_time;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(cmdContactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance,
                            linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);

  currentObservation_.time = time.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state.head(stateDim_) = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();

  const auto& reference_manager = HiInterface_->getSwitchedModelReferenceManagerPtr();
  reference_manager->getSwingTrajectoryPlanner()->setBodyVelWorld(stateEstimate_->getBodyVelWorld());
  reference_manager->setEstContactFlag(cmdContactFlag);

  stateEstimate_->setCmdTorque(jointTor);
  stateEstimate_->estContactForce(period);

  auto remove_gravity = linearAccel;
  remove_gravity(2) -= 9.81;
}

hiController::~hiController()
{
  controllerRunning_ = false;
  mpcRunning_ = false;
  if (mpcThread_.joinable())
  {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void hiController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile,
                                            const std::string& referenceFile, bool verbose)
{
  HiInterface_ = std::make_shared<hiInterface>(taskFile, urdfFile, referenceFile);
  HiInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void hiController::setupMpc()
{
  mpc_ = std::make_shared<SqpMpc>(HiInterface_->mpcSettings(), HiInterface_->sqpSettings(),
                                  HiInterface_->getOptimalControlProblem(), HiInterface_->getInitializer());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  auto rosReferenceManagerPtr =
      std::make_shared<RosReferenceManager>(robotName, HiInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);
}

void hiController::setupMrt()
{
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&HiInterface_->getRollout());
  mpcTimer_.reset();
  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_)
    {
      try
      {
        executeAndSleep(
            [&]() {
              if (mpcRunning_)
              {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
                firstStartMpc_ = true;
              }
            },
            HiInterface_->mpcSettings().mpcDesiredFrequency_);
      }
      catch (const std::exception& e)
      {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(HiInterface_->sqpSettings().threadPriority, mpcThread_);
}

void hiController::setupStateEstimate(const std::string& taskFile, bool verbose)
{
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(
      HiInterface_->getPinocchioInterface(), HiInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  stateEstimate_->loadSettings(taskFile, verbose);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;
}

void hiController::dynamicParamCallback(hi_controllers::TutorialsConfig& config, uint32_t level)
{
  kp_position = config.kp_position;
  kd_position = config.kd_position;

  kp_big_stance = config.kp_big_stance;
  kp_big_swing = config.kp_big_swing;

  kp_small_stance = config.kp_small_stance;
  kp_small_swing = config.kp_small_swing;
  kd_small = config.kd_small;
  kd_big = config.kd_big;

  kd_feet = config.kd_feet;
}

void hiController::RetrievingParameters()
{
  stateDim_ = HiInterface_->getCentroidalModelInfo().stateDim;
  inputDim_ = HiInterface_->getCentroidalModelInfo().inputDim;
  jointDim_ = HiInterface_->getCentroidalModelInfo().actuatedDofNum;
  footDim_ = HiInterface_->getCentroidalModelInfo().numThreeDofContacts;
  gencoordDim_ = HiInterface_->getCentroidalModelInfo().generalizedCoordinatesNum;
  dofPerLeg_ = jointDim_ / 2;
  defalutJointPos_.resize(jointDim_);
}

void hiController::resetMPC()
{
  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });
  mpcMrtInterface_->resetMpcNode(target_trajectories);
}
void hiController::ModeSubscribe()
{
  subSetWalk_ =
      ros::NodeHandle().subscribe<std_msgs::Float32>("/set_walk", 1, &hiController::setWalkCallback, this);
  subLoadcontroller_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/load_controller", 1,
                                                                      &hiController::loadControllerCallback, this);
  subEmgstop_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/emergency_stop", 1,
                                                               &hiController::EmergencyStopCallback, this);
  subResetTarget_ = ros::NodeHandle().subscribe<std_msgs::Float32>("/reset_estimation", 1,
                                                                  &hiController::ResetTargetCallback, this);                                                               
}

void hiController::EmergencyStopCallback(const std_msgs::Float32::ConstPtr& msg)
{
  emergencyStopFlag_ = true;
  ROS_INFO("unload the controller");
}

void hiController::setWalkCallback(const std_msgs::Float32::ConstPtr& msg)
{
  setWalkFlag_ = true;
  ROS_INFO("Set WALK Mode");
}

void hiController::loadControllerCallback(const std_msgs::Float32::ConstPtr& msg)
{
  loadControllerFlag_ = true;
  mpcRunning_ = true;
  ROS_INFO("Successfully load the controller");
}

void hiController::ResetTargetCallback(const std_msgs::Float32::ConstPtr& msg)
{
    // Initial state
  currentObservation_.state.setZero(stateDim_);
  currentObservation_.input.setZero(inputDim_);
  currentObservation_.state.segment(6 + 6, jointDim_) = defalutJointPos_;
  currentObservation_.mode = ModeNumber::STANCE;

  TargetTrajectories target_trajectories({ currentObservation_.time }, { currentObservation_.state },
                                         { currentObservation_.input });

  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO("Reset the target");

} 

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::hiController, controller_interface::ControllerBase)