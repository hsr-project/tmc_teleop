/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef TMC_PSEUDO_VELOCITY_CONTROLLER_PSEUDO_VELOCITY_CONTROLLER_HPP_
#define TMC_PSEUDO_VELOCITY_CONTROLLER_PSEUDO_VELOCITY_CONTROLLER_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <tmc_manipulation_util/joint_trajectory_publisher.hpp>
#include <tmc_msgs/msg/joint_velocity.hpp>

namespace tmc_pseudo_velocity_controller {

/// @class PseudoVelocityController
/// @brief Class that performs pseudo -speed control
class PseudoVelocityController : public rclcpp::Node {
 public:
  /// constructor
  PseudoVelocityController() : PseudoVelocityController(rclcpp::NodeOptions()) {}
  explicit PseudoVelocityController(const rclcpp::NodeOptions& options);

  virtual ~PseudoVelocityController() = default;

 private:
  /// JointVelocity callback
  /// @param [IN] joint_state joint information
  /// @param [IN] joint_velocity joint speed command
  void JointVelocityCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state,
                             const tmc_msgs::msg::JointVelocity::ConstSharedPtr& joint_velocity);

  /// Returns the joint location information required for orbit output
  /// @param [IN] joint_velocity joint speed command
  /// @param [IN] use_joint_names joints required for orbital output
  /// @param [OUT] dst_positions joint position information required for orbital output
  /// @return Whether you succeeded in acquiring the necessary joint location information
  bool InterpolateJointPositions_(const tmc_msgs::msg::JointVelocity::ConstSharedPtr& joint_velocity,
                                  const std::vector<std::string>& use_joint_names,
                                  std::vector<std::pair<std::string, double>>& dst_positions);

  /// Returns the saturated speed to the speed limit
  /// @param [IN] joint_name joint name
  /// @param [in] velocity speed
  /// @return Sperplied speed
  double SaturateToVelocityLimit_(const std::string& joint_name, double velocity);

  /// Check the joints to be controlled
  /// @param [IN] joint_name joint name
  /// @return If it is a control target, True
  bool IsControlJoint(const std::string& joint_name) const;

  /// Late time from the final command transmission to the current angle [S]
  double joint_state_update_delay_;

  /// Rail regeneration time [s]
  double time_from_start_;

  /// publishing period [s]
  std::optional<double> publish_period_;

  /// Group of joint name and maximum speed [RAD/S or M/S]
  std::map<std::string, double> max_velocity_;

  /// Group with joint name and maximum joint angle
  std::map<std::string, double> joint_limit_upper_;

  /// General name and minimum angle group
  std::map<std::string, double> joint_limit_lower_;

  /// Issuance of JointTrajectory
  std::vector<tmc_manipulation_util::JointTrajectoryPublisher::Ptr> trajectory_pubs_;

  /// Jointstate subscription
  message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_sub_;

  /// JointVelocity subscription
  message_filters::Subscriber<tmc_msgs::msg::JointVelocity> joint_velocity_sub_;

  using ApproximateSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::JointState, tmc_msgs::msg::JointVelocity>;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> syncronizer_;

  /// The previous command reception time
  rclcpp::Time last_command_published_time_;

  /// A joint angle for updating the position command by local
  sensor_msgs::msg::JointState local_joint_state_;

  /// Current joint angle
  sensor_msgs::msg::JointState current_joint_state_;

  /// Track collision judgment result
  bool command_result_;

  /// Information for updating local_joint_state_
  std::vector<std::pair<uint32_t, double>> index_and_position_diffs_;
};

}  // namespace tmc_pseudo_velocity_controller

#endif  // TMC_PSEUDO_VELOCITY_CONTROLLER_PSEUDO_VELOCITY_CONTROLLER_HPP_
