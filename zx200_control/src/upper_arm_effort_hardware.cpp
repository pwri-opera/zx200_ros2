#include <cstdio>
#include <limits>

#include "zx200_control/upper_arm_effort_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

//
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
//
namespace zx200_control
{
hardware_interface::CallbackReturn Zx200UpperArmEffortHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  node_ = rclcpp::Node::make_shared("uac_effort_hw");

  // TODO: Fix topic name
  imu_js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/zx200/joint_states", 100, [this](sensor_msgs::msg::JointState msg) { imu_js_callback(msg); });

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

  // position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  // velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  position_states_.resize(info_.joints.size(), 0);
  velocity_states_.resize(info_.joints.size(), 0);
  // old_position_states_.resize(info_.joints.size(), 0);
  // predicted_positions_.resize(info_.joints.size(), 0);

  // get initial joint position and velocity from the hardware interface
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    std::string joint_name = info_.joints[i].name;

    auto get_initial_value =
    [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
      double initial_value{0.0};
      if (!interface_info.initial_value.empty()) {
        try {
          initial_value = std::stod(interface_info.initial_value);
          RCLCPP_INFO(node_->get_logger(), "\t\t\t found initial value: %f", initial_value);
        } catch (std::invalid_argument &) {
          RCLCPP_ERROR_STREAM(
            node_->get_logger(),
            "Failed converting initial_value string to real number for the joint "
              << joint_name
              << " and state interface " << interface_info.name
              << ". Actual value of parameter: " << interface_info.initial_value
              << ". Initial value will be set to 0.0");
          throw std::invalid_argument("Failed converting initial_value string");
        }
      }
      return initial_value;
    };
  
    for (const hardware_interface::InterfaceInfo & interface_info : info_.joints[i].state_interfaces)
    {
      if (interface_info.name == hardware_interface::HW_IF_POSITION)
      {
        position_states_[i] = latest_joint_states_.position[i] = get_initial_value(interface_info);
      }
      else if (interface_info.name == hardware_interface::HW_IF_VELOCITY)
      {
        velocity_states_[i] = latest_joint_states_.velocity[i] = get_initial_value(interface_info);
      }
    }
  }

  // position_commands_.resize(info_.joints.size(), 0);
  // velocity_commands_.resize(info_.joints.size(), 0);
  effort_commands_.resize(info_.joints.size(), 0);

  // imu_joint_values_.resize(info_.joints.size(), 0);

  /* input initial pose here not for real robot should get from init file */
  latest_joint_states_.position[2] = 1;
  // position_states_[2] = 1;
  // old_position_states_[2]=1;
  // predicted_positions_[2]=1;
  // imu_joint_values_[2]=1;
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
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmEffortHardware"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmEffortHardware"),
                   "Joint '%s' have %s command interface. '%s' expected.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Exactly two state interface on each joint
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("Zx200UpperArmEffortHardware"), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmEffortHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Zx200UpperArmEffortHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Zx200UpperArmEffortHardware::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> Zx200UpperArmEffortHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    // command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
Zx200UpperArmEffortHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Set current jointstates.
  // RCLCPP_INFO(rclcpp::get_logger("Zx200UpperArmEffortHardware"), "Waiting to activate...");
  // bool state_is_nan = true;

  // while (state_is_nan)
  // {
  //   state_is_nan = false;

  //   for (int i = 0; i < info_.joints.size(); i++)
  //   {
  //     if (std::isnan(position_states_[i]) || std::isnan(velocity_states_[i]))
  //       state_is_nan = true;
  //   }
  // }

  // for(int i = 0; i < info_.joints.size(); i++){
  //   joint_cmd_msg_.position[i] = position_states_[i];
  //   joint_cmd_msg_.velocity[i] = velocity_states_[i];
  // }

  RCLCPP_INFO(rclcpp::get_logger("Zx200UpperArmEffortHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmEffortHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Zx200UpperArmEffortHardware::read(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  for (size_t i = 0; i < latest_joint_states_.name.size(); i++)
  {
    position_states_[i] = latest_joint_states_.position[i];
    velocity_states_[i] = latest_joint_states_.velocity[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Zx200UpperArmEffortHardware::write(const rclcpp::Time& /*time*/,
                                                                   const rclcpp::Duration& /*period*/)
{
  // Send command
  // joint_cmd_msg_.position = position_commands_;
  // joint_cmd_msg_.velocity = velocity_commands_;
  joint_cmd_msg_.effort = effort_commands_;
  joint_cmd_pub_->publish(joint_cmd_msg_);

  // TODO: Fix to use feedback via CAN
  // for(int i = 0; i < info_.joints.size(); i++){
  //   velocity_states_[i] = (position_states_[i] - old_position_states_[i]) / 1e2; // update rate is 1e2 hz
  // }

  return hardware_interface::return_type::OK;
}

void Zx200UpperArmEffortHardware::imu_js_callback(const sensor_msgs::msg::JointState& msg)
{
  latest_joint_states_ = msg;
}

}  // namespace zx200_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(zx200_control::Zx200UpperArmEffortHardware, hardware_interface::SystemInterface)
