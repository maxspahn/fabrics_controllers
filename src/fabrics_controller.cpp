// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <fabrics_controllers/fabrics_controller.h>

#include <cmath>
#include <memory>
#include <chrono>

#include <fabrics_controllers/pure_controller.c>
#include <fabrics_controllers/franka_model.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace fabrics_controllers {

bool FabricsController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FabricsController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "FabricsController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("FabricsController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "FabricsController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "FabricsController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "FabricsController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "FabricsController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "FabricsController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("FabricsController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("FabricsController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FabricsController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FabricsController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (cartesian_pose_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FabricsController: Error getting cartesian pose interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        cartesian_pose_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "FabricsController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "FabricsController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "FabricsController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  fabrics_goal_subscriber_ = node_handle.subscribe(
    "fabrics_goal", 20, &FabricsController::goalCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());
  error_publisher_= node_handle.advertise<std_msgs::Float64>("error",1);
  error_per_joint_publisher_= node_handle.advertise<std_msgs::Float64MultiArray>("error_per_joint",1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  return true;
}

void FabricsController::goalCallback(
    const std_msgs::Float64MultiArray& msg) {
  for (int i = 0; i < 7; ++i) {
    goal_position_0_[i] = msg.data[i];
  }
}



void FabricsController::starting(const ros::Time& /*time*/) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  goal_position_0_[0] = 0;
  goal_position_0_[1] = 0;
  goal_position_0_[2] = 0;
  goal_position_0_[3] = -1.57089;
  goal_position_0_[4] = 0;
  goal_position_0_[5] = 1.57079;
  goal_position_0_[6] = -0.7853;
  tau_vector_[0] = 0;
  tau_vector_[1] = 0;
  tau_vector_[2] = 0;
  tau_vector_[3] = 0;
  tau_vector_[4] = 0;
  tau_vector_[5] = 0;
  tau_vector_[6] = 0;
  weight_goal_0_[0] = 1;
}

void FabricsController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();
  std::array<double, 49> mass = model_handle_->getMass();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  fabrics_input_[0] = robot_state.q.data();
  fabrics_input_[1] = dq_filtered_.data();
  fabrics_input_[2] = weight_goal_0_.data();
  fabrics_input_[3] = goal_position_0_.data();
  fabrics_output_[0] = new double[7];
  casadi_f0(fabrics_input_, fabrics_output_, setting_0_, setting_1_, setting_2_);

  Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass.data());
  Eigen::Matrix<double, 7, 1> acc_vector;
  Eigen::VectorXd tau_vector(7);
  //std::cout << "acc : ";
  
  for (int i = 0; i < 7; ++i) {
    tau_vector[i] = k_gains_[i] * fabrics_output_[0][i];
    //std::cout << acc_vector[i] << ",";
  }
  std::vector<double> joint_error;
  double fabrics_error(0.0);
  for (size_t i = 0; i < 7; ++i) {
    joint_error.push_back(goal_position_0_[i] - robot_state.q[i]);
    fabrics_error += std::sqrt(std::pow(joint_error[i], 2.0)) / 7.0;
  }
  std_msgs::Float64 error_msg;
  error_msg.data = fabrics_error;
  error_publisher_.publish(error_msg);
  std_msgs::Float64MultiArray error_per_joint_msg;
  error_per_joint_msg.data = joint_error;
  error_per_joint_publisher_.publish(error_per_joint_msg);



  //std::cout << std::endl;

  //tau_vector << mass_matrix * acc_vector;

  // friction
  /*
  Eigen::Matrix<double, 7, 1>  tau_f;
  tau_f(0) =  FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
  tau_f(1) =  FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
  tau_f(2) =  FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
  tau_f(3) =  FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
  tau_f(4) =  FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
  tau_f(5) =  FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
  tau_f(6) =  FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;
  */
  // end friction


  /*
  std::cout << "tau : ";
  for (int i = 0; i < 7; ++i) {
    std::cout << tau_vector[i] << ",";
  }
  std::cout << std::endl;

  std::cout << "mass matrix : " << std::endl;
  for (int i = 0; i < 7; ++i) {
    for (int j = 0; j < 7; ++j) {
      std::cout << mass_matrix(i,j) << ",";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  */




  for (size_t i = 0; i < 7; ++i) {
    /*
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
    */
    //tau_vector_[i] = (1-alpha) * tau_vector_[i] + alpha * tau_vector[i];
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] + tau_vector[i];
    //tau_d_calculated[i] = tau_vector[i];
  }


  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> FabricsController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace fabrics_controllers

PLUGINLIB_EXPORT_CLASS(fabrics_controllers::FabricsController,
                       controller_interface::ControllerBase)
