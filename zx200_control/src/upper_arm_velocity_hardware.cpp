#include <cstdio>

#include "zx200_control/upper_arm_velocity_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

//
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
//
namespace zx200_control
{
hardware_interface::CallbackReturn Zx200UpperArmVelocityHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("uac_velocity_hw");

  imu_js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/zx200/front/joint_states", 100, [this](sensor_msgs::msg::JointState msg) { imu_js_callback(msg); });

  joint_cmd_pub_ = node_->create_publisher<com3_msgs::msg::JointCmd>("/zx200/front_cmd", 100);

  node_thread_ = std::thread([this]() { rclcpp::spin(node_); });

  joint_cmd_msg_.joint_name.resize(info_.joints.size());
  latest_joint_states_.name.resize(info_.joints.size());
  for (int i = 0; i < (int)info_.joints.size(); i++)
  {
    joint_cmd_msg_.joint_name[i] = info_.joints[i].name;
    latest_joint_states_.name[i] = info_.joints[i].name;
  }
  joint_cmd_msg_.position.resize(info_.joints.size(), 0);
  joint_cmd_msg_.velocity.resize(info_.joints.size(), 0);
  joint_cmd_msg_.effort.resize(info_.joints.size(), 0);

  latest_joint_states_.position.resize(info_.joints.size(), 0);
  latest_joint_states_.velocity.resize(info_.joints.size(), 0);
  latest_joint_states_.effort.resize(info_.joints.size(), 0);

  position_states_.resize(info_.joints.size(), 0);
  velocity_states_.resize(info_.joints.size(), 0);
  // old_position_states_.resize(info_.joints.size(), 0);
  // predicted_positions_.resize(info_.joints.size(), 0);

  // position_commands_.resize(info_.joints.size(), 0);
  // velocity_commands_.resize(info_.joints.size(), 0);
  velocity_commands_.resize(info_.joints.size(), 0);

  // imu_joint_values_.resize(info_.joints.size(), 0);

  /* input initial pose here not for real robot should get from init file */
  // position_states_[2] = 1;
  // old_position_states_[2] = 1;
  // predicted_positions_[2] = 1;
  // imu_joint_values_[2] = 1;
  // hw_commands_[0] = 0;
  // hw_commands_[1] = 0;
  // hw_commands_[2] = 1;
  // hw_commands_[3] = 0;
  // hw_commands_[4] = 0;

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    // Exactly one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmVelocityHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmVelocityHardware"),
                   "Joint '%s' have %s command interface. '%s' expected.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Exactly two state interface on each joint
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmVelocityHardware"),
                   "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                   joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmVelocityHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zx200UpperArmVelocityHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Zx200UpperArmVelocityHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Zx200UpperArmVelocityHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
Zx200UpperArmVelocityHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zx200UpperArmVelocityHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmVelocityHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Zx200UpperArmVelocityHardware::read(const rclcpp::Time& /*time*/,
                                                                    const rclcpp::Duration& /*period*/)
{
  for (int i = 0; i < latest_joint_states_.name.size(); i++)
  {
    position_states_[i] = latest_joint_states_.position[i];
    velocity_states_[i] = latest_joint_states_.velocity[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zx200UpperArmVelocityHardware::write(const rclcpp::Time& /*time*/,
                                                                     const rclcpp::Duration& /*period*/)
{
  // Send command
  // joint_cmd_msg_.position = position_commands_;
  // joint_cmd_msg_.velocity = velocity_commands_;
  joint_cmd_msg_.velocity = velocity_commands_;
  joint_cmd_pub_->publish(joint_cmd_msg_);

  return hardware_interface::return_type::OK;
}

void Zx200UpperArmVelocityHardware::imu_js_callback(const sensor_msgs::msg::JointState& msg)
{
  latest_joint_states_ = msg;
}
}  // namespace zx200_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zx200_control::Zx200UpperArmVelocityHardware, hardware_interface::SystemInterface)
