#include "quad_motor_plugin/QuadMotorPlugin.hh"
#include <ignition/plugin/Register.hh>
#include <ignition/math/Vector3.hh>

namespace quad_motor_plugin
{

QuadMotorPlugin::QuadMotorPlugin()
{
}

// This method create a ROS 2 subscriber to receive motor thrust commands from a topic.
// The subscriber listens to a Float64MultiArray message containing the
// thrust values of the quadrotor motors and forwards received messages
// to the ThrustCallback() function for processing.
void QuadMotorPlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &)
{
  // create a Gazebo Model object from the entity ID _entity and stores it in the class member variable model.
  this->model = ignition::gazebo::Model(_entity);

  // Check whether this->model really refers to a valid Gazebo model inside the current simulation world.
  if (!this->model.Valid(_ecm))
  {
    ignerr << "QuadMotorPlugin must be attached to a model.\n";
    return;
  }
  // Initializ ROS 2 only if it has not already been initialized:
  if (!rclcpp::ok())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
  // Create a single-threaded ROS 2 executor (rclcpp::executors::SingleThreadedExecutor) and stores it in your plugin
  // So, this Gazebo plugin can process ROS callbacks
  this->executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Create a ROS 2 node named quad_motor_plugin_node and store it in the class member rosNode
  this->rosNode = rclcpp::Node::make_shared("quad_motor_plugin_node");

  // Check whether the SDF plugin block contains an element named thrust_topic.
  // If it exists, then read its value as a std::string and stores it in the class member variable thrustTopic.
  if (_sdf->HasElement("thrust_topic"))
  {
    this->thrustTopic = _sdf->Get<std::string>("thrust_topic");
  }

  // Find the entity ID of a link named base_link inside the model. 
  // _ecm is the Gazebo Entity Component Manager, which stores simulation entities and components.
  auto linkEntity = this->model.LinkByName(_ecm, "base_link");

  // Check whether Gazebo actually found the link
  if (linkEntity == ignition::gazebo::kNullEntity)
  {
    ignerr << "Could not find base_link\n";
    return;
  }

  // Wrap the found linkEntity as a Gazebo Link object:
  this->baseLink = ignition::gazebo::Link(linkEntity);

  // Tell Gazebo to compute/store velocity and acceleration information for baseLink
  this->baseLink.EnableVelocityChecks(_ecm, true);
  this->baseLink.EnableAccelerationChecks(_ecm, true);


  ignmsg << "Found base_link entity = "
         << linkEntity << "\n";
  // Create a ROS 2 subscriber inside the Gazebo plugin
  this->thrustSub =
    this->rosNode->create_subscription<std_msgs::msg::Float64MultiArray>(
      this->thrustTopic,
      rclcpp::QoS(1),
      std::bind(&QuadMotorPlugin::ThrustCallback, this, std::placeholders::_1));

  this->executor->add_node(this->rosNode);

  ignmsg << "QuadMotorPlugin loaded. Listening to ROS topic: "
         << this->thrustTopic << "\n";
}

// Callback function executed whenever a new motor thrust command
// is received from the ROS 2 thrust topic. The function reads the
// thrust values from the incoming Float64MultiArray message,
// updates the internal motor thrust commands, records the reception
// time, and marks that a valid command has been received.
void QuadMotorPlugin::ThrustCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 4)
    return;

  // Prevent other threads from accessing the thrust data while this code section is using or modifying it
  std::lock_guard<std::mutex> lock(this->thrustMutex);

  // Read the four motor thrust commands from the incoming message and save them for later use in the Gazebo update loop.
  for (int i = 0; i < 4; ++i)
  {
    this->latestThrusts[i] = msg->data[i];
  }
  // Record the exact time at which the latest thrust command was received.
  this->lastCommandWallTime = std::chrono::steady_clock::now();
  // Mark that the quadrotor has received at least one motor thrust command.
  this->commandReceived = true;

  if (false)    // For debugging
    ignmsg << "Received thrusts: "
           << msg->data[0] << ", "
           << msg->data[1] << ", "
           << msg->data[2] << ", "
           << msg->data[3] << "\n";
}

void QuadMotorPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  // Process ROS callbacks
  this->executor->spin_some();
  bool commandTimedOut = false;

  // This block checks whether the thrust command has timed out
  // It is inside a block mainly to limit the lifetime of the mutex lock
  {
    // locks thrustMutex to safely access shared thrust data.
    std::lock_guard<std::mutex> lock(this->thrustMutex);
    if (!this->commandReceived)
    {
      // If no command has ever been received, consider the command timed out
      commandTimedOut = true;
    }
    else
    {
      // Otherwise, it checks how much real time has passed since the last received command
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - this->lastCommandWallTime).count();
      if (elapsed > this->commandTimeoutSec)
        commandTimedOut = true;
    }

    if (commandTimedOut)
    {
      this->latestThrusts = {0.0, 0.0, 0.0, 0.0};
    }
  }

  std::array<double, 4> thrusts;

  // Safely copy the latest thrust commands
  // First lock the mutex so latestThrusts cannot be changed by another thread during copying
  {
    std::lock_guard<std::mutex> lock(this->thrustMutex);
    thrusts = this->latestThrusts;
  }

  double totalThrust = thrusts[0] + thrusts[1] + thrusts[2] + thrusts[3];

  if (!this->baseLink.Valid(_ecm))
  {
    ignerr << "Base link invalid\n";
    return;
  }

  ignition::math::Vector3d forceWorld(0.0, 0.0,totalThrust);
  ignition::math::Vector3d torqueWorld(0.0, 0.0, 0.0);

  this->baseLink.AddWorldWrench(_ecm, forceWorld, torqueWorld);

  if (false)    // For debugging
    ignmsg << "Applying wrench force = "
           << totalThrust << "\n";
}
}

IGNITION_ADD_PLUGIN(
  quad_motor_plugin::QuadMotorPlugin,
  ignition::gazebo::System,
  quad_motor_plugin::QuadMotorPlugin::ISystemConfigure,
  quad_motor_plugin::QuadMotorPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  quad_motor_plugin::QuadMotorPlugin,
  "quad_motor_plugin::QuadMotorPlugin")