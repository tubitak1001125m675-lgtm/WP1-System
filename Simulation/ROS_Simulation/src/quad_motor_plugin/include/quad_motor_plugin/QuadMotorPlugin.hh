#pragma once

#include <array>
#include <mutex>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/EntityComponentManager.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <chrono>

namespace quad_motor_plugin
{
class QuadMotorPlugin:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  QuadMotorPlugin();

  void Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr) override;

  void PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;

private:
  void ThrustCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  ignition::gazebo::Link baseLink;

  std::array<double, 4> latestThrusts{0.0, 0.0, 0.0, 0.0};

  std::mutex thrustMutex;

  std::chrono::steady_clock::time_point lastCommandWallTime{
    std::chrono::steady_clock::now()
  };
  double commandTimeoutSec{0.2};
  bool commandReceived{false};

  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr thrustSub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::string thrustTopic{"/quad/rotor_thrusts"};
};
}