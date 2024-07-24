// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <Eigen/Dense>

//#include <fabrics_controllers/pure_controller.c>
#include <fabrics_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace fabrics_controllers {

class FabricsController: public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

  void goalCallback(const std_msgs::Float64MultiArray& msg);

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  // fabrics function call settings
  long long int* setting_0_ = new long long int[2];
  double* setting_1_ = new double[2];
  int setting_2_ = 0;
  std::array<double, 7> goal_position_0_;
  std::array<double, 1> weight_goal_0_;

  // Define inputs and outputs
  // Potential Segmentation Faults can be caused by the wrong number of arguments.
  unsigned int nb_inputs = 4;
  unsigned int nb_outputs = 1;


  const double** fabrics_input_ = new const double*[nb_inputs];
  double** fabrics_output_ = new double*[nb_outputs];


  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 16> initial_pose_;
  std::array<double, 7> tau_vector_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;

  ros::Publisher error_publisher_;
  ros::Publisher error_per_joint_publisher_;
  ros::Subscriber fabrics_goal_subscriber_;
};

}  // namespace fabrics_controllers
