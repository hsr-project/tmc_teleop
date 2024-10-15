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
#include "pseudo_velocity_controller.hpp"

#include <algorithm>
#include <limits>

#include <urdf/model.h>

#include <tmc_utils/parameters.hpp>
#include <tmc_utils/qos.hpp>

namespace {
/// Delay time from the default final command to update the current angle [S]
constexpr double kDefaultJointStateUpdateDelayFromLastCommand = 0.5;
/// Default orbit play time [S]
constexpr double kDefaultTimeFromStart = 0.05;
/// Difference judgment parameter between local joint angle and present angle
constexpr double kLimitVelocityDuration = 0.5;

bool ExtractPosition(const rclcpp::Logger& logger,
                     const sensor_msgs::msg::JointState& joint_state,
                     const std::string& joint_name,
                     double& dst_position,
                     uint32_t& dst_index) {
  const auto name_it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
  if (name_it == joint_state.name.end()) {
    RCLCPP_ERROR(logger, "Joint id [%s] does not exist.", joint_name.c_str());
    return false;
  }
  dst_index = std::distance(joint_state.name.begin(), name_it);
  if (dst_index >= joint_state.position.size()) {
    RCLCPP_ERROR(logger, "Joint position [%d] does not exist.", dst_index);
    return false;
  }
  dst_position = joint_state.position[dst_index];
  return true;
}

bool ExtractPosition(const rclcpp::Logger& logger,
                     const sensor_msgs::msg::JointState& joint_state,
                     const std::string& joint_name,
                     double& dst_position) {
  uint32_t index;
  return ExtractPosition(logger, joint_state, joint_name, dst_position, index);
}
}  // unnamed namespace

namespace tmc_pseudo_velocity_controller {

PseudoVelocityController::PseudoVelocityController(const rclcpp::NodeOptions& options)
    : Node("pseudo_velocity_controller", options),
      command_result_(true) {
  joint_state_update_delay_ = tmc_utils::GetParameter(this, "joint_state_update_delay_from_last_command",
                                                      kDefaultJointStateUpdateDelayFromLastCommand);
  if (joint_state_update_delay_ <= 0.0) {
    joint_state_update_delay_ = kDefaultJointStateUpdateDelayFromLastCommand;
    RCLCPP_WARN(this->get_logger(), "Invalid joint_state_update_delay_from_last_command param. Using default %f [s]",
                joint_state_update_delay_);
  }

  time_from_start_ = tmc_utils::GetParameter(this, "time_from_start", kDefaultTimeFromStart);
  if (time_from_start_ <= 0.0) {
    time_from_start_ = kDefaultTimeFromStart;
    RCLCPP_WARN(this->get_logger(), "Invalid time_from_start param. Using default %f [s]", time_from_start_);
  }

  double publish_rate = tmc_utils::GetParameter(this, "publish_rate", 30.0);
  if (publish_rate > 0.0) {
    publish_period_ = 1.0 / publish_rate;
  }

  const auto joint_trajectory_controllers = tmc_utils::GetParameter<std::vector<std::string>>(
      this, "joint_trajectory_controllers", {});
  if (joint_trajectory_controllers.empty()) {
    throw std::runtime_error("Failed to load joint_trajectory_controllers");
  }
  for (const auto& joint_trajectory_controller : joint_trajectory_controllers) {
    trajectory_pubs_.push_back(
        std::make_shared<tmc_manipulation_util::JointTrajectoryPublisher>(this, joint_trajectory_controller));
  }

  // Read a group of joint name and maximum speed
  const auto max_velocity_joint_names = tmc_utils::GetParameter<std::vector<std::string>>(
      this, "max_velocity_joint_names", {});
  if (max_velocity_joint_names.empty()) {
    throw std::runtime_error("Failed to load 'max_velocity_joint_names'.");
  }
  const auto max_velocity = tmc_utils::GetParameter<std::vector<double>>(this, "max_velocity", {});
  if (max_velocity.empty()) {
    throw std::runtime_error("Failed to load 'max_velocity'.");
  }
  for (auto value : max_velocity) {
    if (value < 0.0) {
      throw std::runtime_error("max_velocity must be positive value.");
    }
  }
  if (max_velocity_joint_names.size() != max_velocity.size()) {
    throw std::runtime_error("The size of 'max_velocity' or 'max_velocity_joint_names' is invalid.");
  }
  for (auto i = 0; i < max_velocity.size(); ++i) {
    max_velocity_[max_velocity_joint_names.at(i)] = max_velocity.at(i);
  }

  // Read the joint angle limit
  const auto urdf_str = tmc_utils::GetParameter(this, "robot_description", "");
  if (urdf_str.empty()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load robot_description");
    throw std::runtime_error("Failed to load robot_description.");
  }
  const auto urdf = std::make_shared<urdf::Model>();
  if (!urdf->initString(urdf_str)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to parse URDF contained in robot_description.");
    throw std::runtime_error("Failed to parse URDF contained in robot_description.");
  }
  for (uint32_t i = 0; i < max_velocity.size(); ++i) {
    std::string joint_name = max_velocity_joint_names.at(i);
    if (urdf->getJoint(joint_name)) {
      const auto urdf_joint_limits = urdf->getJoint(joint_name)->limits;
      if (urdf->getJoint(joint_name)->type == urdf::Joint::CONTINUOUS) {
        joint_limit_upper_[joint_name] = std::numeric_limits<double>::max();
        joint_limit_lower_[joint_name] = std::numeric_limits<double>::lowest();
      } else {
        joint_limit_upper_[joint_name] = urdf_joint_limits->upper;
        joint_limit_lower_[joint_name] = urdf_joint_limits->lower;
      }
    }
  }

  last_command_published_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  auto rmw_qos_profile = tmc_utils::BestEffortQoS().get_rmw_qos_profile();
  joint_state_sub_.subscribe(this, "joint_states", rmw_qos_profile);
  joint_velocity_sub_.subscribe(this, "~/ref_joint_velocity", rmw_qos_profile);

  syncronizer_ =
      std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
          ApproximateSyncPolicy(10), joint_state_sub_, joint_velocity_sub_);
  syncronizer_->registerCallback(
      std::bind(&PseudoVelocityController::JointVelocityCallback, this, std::placeholders::_1, std::placeholders::_2));
}

bool PseudoVelocityController::InterpolateJointPositions_(
    const tmc_msgs::msg::JointVelocity::ConstSharedPtr& joint_velocity,
    const std::vector<std::string>& use_joint_names,
    std::vector<std::pair<std::string, double>>& dst_positions) {
  // Joint_velocity contains the necessary joints in the controller
  std::vector<std::pair<std::string, double>> name_and_velocities;
  for (auto i = 0; i < joint_velocity->name.size(); ++i) {
    for (const auto& use_joint_name : use_joint_names) {
      if (joint_velocity->name.at(i) == use_joint_name) {
        name_and_velocities.push_back(std::make_pair(joint_velocity->name.at(i), joint_velocity->velocity.at(i)));
        break;
      }
    }
  }

  // If the joint is not a Controller, do nothing
  if (name_and_velocities.empty()) {
    return false;
  }

  dst_positions.reserve(name_and_velocities.size());
  for (const auto& name_and_velocity : name_and_velocities) {
    const auto joint_name = name_and_velocity.first;

    if (max_velocity_.count(joint_name) == 0) {
      RCLCPP_ERROR(this->get_logger(), "Max velocity param is not set for joint id [%s].", joint_name.c_str());
      return false;
    }
    if (joint_limit_upper_.count(joint_name) == 0 || joint_limit_lower_.count(joint_name) == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed to load joint_limit_upper_[" << joint_name << "]"
                          << " or joint_limit_lower_[" << joint_name << "].");
      return false;
    }

    // There is also a limit to the difference between the angle and the joint angle of the local management.
    // Calculate the final angle limit with the joint angle limit.
    double current_position;
    if (!ExtractPosition(this->get_logger(), current_joint_state_, joint_name, current_position)) {
      return false;
    }
    const double joint_limit_upper = std::min(joint_limit_upper_[joint_name],
                                              current_position + max_velocity_[joint_name] * kLimitVelocityDuration);
    const double joint_limit_lower = std::max(joint_limit_lower_[joint_name],
                                              current_position - max_velocity_[joint_name] * kLimitVelocityDuration);

    double local_position;
    uint32_t local_index;
    if (!ExtractPosition(this->get_logger(), local_joint_state_, joint_name, local_position, local_index)) {
      return false;
    }
    const auto saturated_velocity = SaturateToVelocityLimit_(joint_name, name_and_velocity.second);
    double command = local_position + saturated_velocity * time_from_start_;

    if (command > joint_limit_upper) {
      command = joint_limit_upper;
    } else if (command < joint_limit_lower) {
      command = joint_limit_lower;
    }
    dst_positions.push_back(std::make_pair(joint_name, command));
    index_and_position_diffs_.push_back(std::make_pair(local_index, command - local_position));
  }
  return true;
}

void PseudoVelocityController::JointVelocityCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state,
    const tmc_msgs::msg::JointVelocity::ConstSharedPtr& joint_velocity) {
  const auto stamp = this->now();
  const auto duration = (stamp - last_command_published_time_).seconds();
  if (publish_period_.has_value() && duration < publish_period_.value()) {
    return;
  }

  // Currently acquired angle
  current_joint_state_ = *joint_state;

  // Currently updated the angle if there is no certain time speed command from the final trajectory command update
  // If you calculate the command angle for the current joint angle without using the local joint angle, it will be vibrated.
  if ((stamp - last_command_published_time_).seconds() >= joint_state_update_delay_ || !command_result_) {
    local_joint_state_ = *joint_state;
  } else {
    const auto rate = std::min(1.0, duration / time_from_start_);
    for (const auto& index_and_position_diff : index_and_position_diffs_) {
      local_joint_state_.position.at(index_and_position_diff.first) += rate * index_and_position_diff.second;
    }
  }

  // Confirm that the size of the joint name and the speed are equal and not empty
  if (joint_velocity->name.size() != 0 && joint_velocity->name.size() != joint_velocity->velocity.size()) {
    RCLCPP_WARN(this->get_logger(), "Name size [%lu] and velocity size [%lu] does not match.",
                joint_velocity->name.size(), joint_velocity->velocity.size());
    return;
  }

  for (const auto& input_name : joint_velocity->name) {
    if (!IsControlJoint(input_name)) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 1000,
                                  input_name << " is not in controlled joints.");
    }
  }

  const auto prev_index_and_position_diffs = index_and_position_diffs_;
  index_and_position_diffs_.clear();
  bool do_publish = false;
  for (const auto& pub : trajectory_pubs_) {
    std::vector<std::pair<std::string, double>> positions;
    if (!InterpolateJointPositions_(joint_velocity, pub->joint_names(), positions)) {
      continue;
    }
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = stamp;
    trajectory.points.resize(1);
    trajectory.points.back().time_from_start = rclcpp::Duration::from_seconds(time_from_start_);
    for (const auto& name_and_poisiton : positions) {
      trajectory.joint_names.push_back(name_and_poisiton.first);
      trajectory.points.back().positions.push_back(name_and_poisiton.second);
    }
    pub->Publish(trajectory, local_joint_state_);
    do_publish = true;
  }
  if (do_publish) {
    last_command_published_time_ = stamp;
  } else {
    index_and_position_diffs_ = prev_index_and_position_diffs;
  }
}

double PseudoVelocityController::SaturateToVelocityLimit_(const std::string& joint_name, double velocity) {
  double out_velocity = velocity;
  if (velocity > max_velocity_[joint_name]) {
    out_velocity = max_velocity_[joint_name];
  } else if (velocity < -max_velocity_[joint_name]) {
    out_velocity = -max_velocity_[joint_name];
  }
  return out_velocity;
}

bool PseudoVelocityController::IsControlJoint(const std::string& joint_name) const {
  for (const auto& pub : trajectory_pubs_) {
    for (const auto& controlled : pub->joint_names()) {
      if (joint_name == controlled) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace tmc_pseudo_velocity_controller
