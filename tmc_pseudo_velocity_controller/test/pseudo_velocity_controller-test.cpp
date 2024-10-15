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
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>

#include "../src/pseudo_velocity_controller.hpp"

namespace {
constexpr double kEpsilon = 1e-6;

const std::vector<std::string> kController1Joints = {"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R"};
const std::vector<std::string> kController2Joints = {"CARM/WRIST_R", "CARM/WRIST_Y"};

class CachingSubscriber {
 public:
  using Ptr = std::shared_ptr<CachingSubscriber>;

  CachingSubscriber(const rclcpp::Node::SharedPtr& node, const std::string& topic_name) {
    sub_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        topic_name, 1, std::bind(&CachingSubscriber::Callback, this, std::placeholders::_1));
  }
  virtual ~CachingSubscriber() = default;

  bool IsSubscribed() const { return !msgs_.empty(); }
  std::vector<trajectory_msgs::msg::JointTrajectory> GetTrajectories() const { return msgs_; }
  void Reset() { msgs_.clear(); }

 private:
  void Callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { msgs_.push_back(*msg); }
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  std::vector<trajectory_msgs::msg::JointTrajectory> msgs_;
};


sensor_msgs::msg::JointState CreateJointState(const rclcpp::Time& stamp, double shoulder_y_pos = 0.2) {
  sensor_msgs::msg::JointState state;
  state.header.stamp = stamp;
  state.name = {"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R", "CARM/WRIST_R", "CARM/WRIST_Y"};
  state.position = {0.1, shoulder_y_pos, 0.3, 0.4, 0.0};
  return state;
}

}  // unnamed namespace

namespace tmc_pseudo_velocity_controller {

class PseudoVelocityControllerTest : public ::testing::Test {
 protected:
  void SetUp() override;

  void PublishCommand(const std::vector<std::string>& joint_names, const std::vector<double>& velocities,
                      double shoulder_y_pos = 0.2);

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<tmc_msgs::msg::JointVelocity>::SharedPtr joint_velocity_pub_;

  CachingSubscriber::Ptr joint_trajectory_sub_;
  CachingSubscriber::Ptr joint_trajectory_2_sub_;
};

void PseudoVelocityControllerTest::SetUp() {
  client_node_ = rclcpp::Node::make_shared("client");
  joint_state_pub_ = client_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  joint_velocity_pub_ = client_node_->create_publisher<tmc_msgs::msg::JointVelocity>(
      "pseudo_velocity_controller/ref_joint_velocity", 1);

  joint_trajectory_sub_ = std::make_shared<CachingSubscriber>(client_node_, "controller_1/joint_trajectory");
  joint_trajectory_2_sub_ = std::make_shared<CachingSubscriber>(client_node_, "controller_2/joint_trajectory");

  boost::property_tree::ptree tree;
  auto istream = std::istringstream(tmc_manipulation_tests::hsra::GetUrdf());
  boost::property_tree::read_xml(istream, tree);
  for (auto& child : tree.get_child("robot")) {
    if (child.first != "joint") {
      continue;
    }
    if (child.second.get<std::string>("<xmlattr>.name") == "CARM/WRIST_Y") {
      child.second.put("<xmlattr>.type", "continuous");
    }
  }
  std::stringstream ostream;
  boost::property_tree::write_xml(ostream, tree);

  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", ostream.str()),
      rclcpp::Parameter("joint_state_update_delay_from_last_command", 0.2),
      rclcpp::Parameter("publish_rate", 250.0),
      rclcpp::Parameter("joint_trajectory_controllers", std::vector<std::string>({"controller_1", "controller_2"})),
      rclcpp::Parameter("controller_1.joints", kController1Joints),
      rclcpp::Parameter("controller_2.joints", kController2Joints),
      rclcpp::Parameter("max_velocity_joint_names",
                        std::vector<std::string>({"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R",
                                                  "CARM/SHOULDER_P", "CARM/ELBOW_P", "CARM/WRIST_Y",
                                                  "CARM/WRIST_R", "CARM/WRIST_P"})),
      rclcpp::Parameter("max_velocity", std::vector<double>({1.0, 0.2, 0.2, 0.2, 0.2, 1000.0, 0.2, 0.2}))};
  server_node_ = std::make_shared<PseudoVelocityController>(node_options);
}

void PseudoVelocityControllerTest::PublishCommand(
    const std::vector<std::string>& joint_names, const std::vector<double>& velocities, double shoulder_y_pos) {
  const auto stamp = client_node_->now();

  tmc_msgs::msg::JointVelocity command;
  command.header.stamp = stamp;
  command.name = joint_names;
  command.velocity = velocities;
  joint_velocity_pub_->publish(command);

  joint_state_pub_->publish(CreateJointState(stamp, shoulder_y_pos));
}

TEST_F(PseudoVelocityControllerTest, SingleJointCommand) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y"}, {0.1});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  auto trajectory = joint_trajectory_sub_->GetTrajectories().front();
  EXPECT_EQ(trajectory.joint_names, kController1Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 3);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.1, kEpsilon);
  // Now angle + instruction speed * time_from_start
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.1 * 0.05, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[2], 0.3, kEpsilon);

  // Stop sending commands for a while and reset
  joint_trajectory_sub_->Reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  while (!joint_trajectory_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y"}, {-0.1}, 0.3);
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  trajectory = joint_trajectory_sub_->GetTrajectories().front();
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.3 - 0.1 * 0.05, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, ContinuousCommand) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.5);
  const auto command_change_stamp = client_node_->now() + rclcpp::Duration::from_seconds(0.2);
  while (client_node_->now() < timeout) {
    if (client_node_->now() < command_change_stamp) {
      PublishCommand({"CARM/SHOULDER_Y"}, {0.1});
    } else {
      // Continuous command values ​​change the joint status to test not to use the current joint status.
      PublishCommand({"CARM/SHOULDER_Y"}, {0.1}, 0.3);
    }
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_GT(joint_trajectory_sub_->GetTrajectories().size(), 40);

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().back();
  EXPECT_EQ(trajectory.joint_names, kController1Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 3);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.1, kEpsilon);
  // The command value of +0.05 seconds that moved 0.5 seconds should appear
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.1 * 0.55, 0.02);
  EXPECT_NEAR(trajectory.points[0].positions[2], 0.3, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, PublishRate) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.5);
  while (client_node_->now() < timeout) {
    PublishCommand({"CARM/SHOULDER_Y"}, {0.1});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // Since 250Hz is the upper limit, in 0.5 seconds, it should not fly over 125 topics.
  EXPECT_LT(joint_trajectory_sub_->GetTrajectories().size(), 125);

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().back();
  EXPECT_EQ(trajectory.joint_names, kController1Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 3);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.1, kEpsilon);
  // The command value of +0.05 seconds that moved 0.5 seconds should appear
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.1 * 0.55, 0.02);
  EXPECT_NEAR(trajectory.points[0].positions[2], 0.3, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, TooBigVelocity) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y"}, {0.3});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().front();
  // The speed limit is 0.2
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.2 * 0.05, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, JointMaxPosition) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (client_node_->now() < timeout) {
    PublishCommand({"CARM/LINEAR"}, {1.0});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().back();
  // The command value is sparse in the position limit
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.49, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, JointMinPosition) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (client_node_->now() < timeout) {
    PublishCommand({"CARM/LINEAR"}, {-1.0});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().back();
  // The command value is sparse in the position limit
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.0, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, MultiJointSingleController) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y", "CARM/SHOULDER_R"}, {0.1, 0.15});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto trajectory = joint_trajectory_sub_->GetTrajectories().front();
  EXPECT_EQ(trajectory.joint_names, kController1Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 3);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.1 * 0.05, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[2], 0.3 + 0.15 * 0.05, kEpsilon);

  EXPECT_FALSE(joint_trajectory_2_sub_->IsSubscribed());
}

TEST_F(PseudoVelocityControllerTest, MultiJointMultiController) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_sub_->IsSubscribed() || !joint_trajectory_2_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y", "CARM/WRIST_R"}, {0.1, 0.15});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  auto trajectory = joint_trajectory_sub_->GetTrajectories().front();
  EXPECT_EQ(trajectory.joint_names, kController1Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 3);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.1, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.2 + 0.1 * 0.05, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[2], 0.3, kEpsilon);

  trajectory = joint_trajectory_2_sub_->GetTrajectories().front();
  EXPECT_EQ(trajectory.joint_names, kController2Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 2);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.4 + 0.15 * 0.05, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[1], 0.0, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, ContinuousJoint) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_2_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/WRIST_Y"}, {1000.0});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  const auto trajectory = joint_trajectory_2_sub_->GetTrajectories().front();
  EXPECT_EQ(trajectory.joint_names, kController2Joints);
  ASSERT_EQ(trajectory.points.size(), 1);
  ASSERT_EQ(trajectory.points[0].positions.size(), 2);
  EXPECT_NEAR(trajectory.points[0].positions[0], 0.4, kEpsilon);
  EXPECT_NEAR(trajectory.points[0].positions[1], 1000.0 * 0.05, kEpsilon);
}

TEST_F(PseudoVelocityControllerTest, InvalidSizeOfVelocityCommand) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(0.5);
  while (client_node_->now() < timeout) {
    if (joint_trajectory_sub_->IsSubscribed() || joint_trajectory_2_sub_->IsSubscribed()) {
      FAIL();
      return;
    }
    PublishCommand({"CARM/SHOULDER_Y", "CARM/SHOULDER_R"}, {0.1});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

TEST_F(PseudoVelocityControllerTest, InvalidJointName) {
  const auto timeout = client_node_->now() + rclcpp::Duration::from_seconds(1.0);
  while (!joint_trajectory_sub_->IsSubscribed()) {
    if (client_node_->now() > timeout) {
      FAIL();
      return;
    }
    // Even if an unauthorized joint is mixed, the normal part is issued
    PublishCommand({"CARM/SHOULDER_Y", "hoge"}, {0.1, 0.1});
    rclcpp::spin_some(server_node_);
    rclcpp::spin_some(client_node_);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

}  // namespace tmc_pseudo_velocity_controller

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
