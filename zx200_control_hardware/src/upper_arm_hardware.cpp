#include <cstdio>

#include "zx200_control_hardware/upper_arm_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
// 
namespace ZX200_control_hardware
{
  hardware_interface::CallbackReturn ZX200UpperArmPositionHardware::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("uac_hw");
    // joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/test/joint_states", 100);
    // swing_setpoint_pub_ = node_->create_publisher<std_msgs::msg::Float64>("swing/setpoint_common", 100);
    // boom_setpoint_pub_ = node_->create_publisher<std_msgs::msg::Float64>("boom/setpoint_common", 100);
    // arm_setpoint_pub_ = node_->create_publisher<std_msgs::msg::Float64>("arm/setpoint_common", 100);
    // bucket_setpoint_pub_ = node_->create_publisher<std_msgs::msg::Float64>("bucket/setpoint_common", 100);

    // swing_state_pub_ = node_->create_publisher<std_msgs::msg::Float64>("swing/state", 100);
    // boom_state_pub_ = node_->create_publisher<std_msgs::msg::Float64>("boom/state", 100);
    // arm_state_pub_ = node_->create_publisher<std_msgs::msg::Float64>("arm/state", 100);
    // bucket_state_pub_ = node_->create_publisher<std_msgs::msg::Float64>("bucket/state", 100);

    // TODO: Fix topic name
    imu_js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/zx200/com3_ros/joint_states", 100, [this](sensor_msgs::msg::JointState msg){ imu_js_callback(msg);});

    joint_cmd_pub_ = node_->create_publisher<com3_msgs::msg::JointCmd>("/zx200/joint_cmd", 100);

    node_thread_ = std::thread([this]()
                               { rclcpp::spin(node_); });

    // joint_state_msg_.header.frame_id = "joint-state data";
    // joint_state_msg_.name.resize(info_.joints.size());
    // joint_state_msg_.position.resize(info_.joints.size(), 0);
    // joint_state_msg_.velocity.resize(info_.joints.size(), 0);
    // joint_state_msg_.effort.resize(info_.joints.size(), 0);
    // for (int i = 0; i < (int)info_.joints.size(); i++)
    // {
    //   joint_state_msg_.name[i] = info_.joints[i].name;
    // }

    joint_cmd_msg_.joint_name.resize(info_.joints.size());
    for (int i = 0; i < (int)info_.joints.size(); i++)
    {
      joint_cmd_msg_.joint_name[i] = info_.joints[i].name;
    }
    joint_cmd_msg_.position.resize(info_.joints.size(), 0);
    joint_cmd_msg_.velocity.resize(info_.joints.size(), 0);
    joint_cmd_msg_.effort.resize(info_.joints.size(), 0);

    // hw_states_.resize(info_.joints.size(), 0);
    // hw_commands_.resize(info_.joints.size(), 0);
    // ac58_joint_states_.resize(info_.joints.size(), 0.7);
    position_states_.resize(info_.joints.size(), 0);
    position_commands_.resize(info_.joints.size(), 0);
    velocity_commands_.resize(info_.joints.size(), 0);
    effort_commands_.resize(info_.joints.size(), 0);

    /* input initial pose here not for real robot should get from init file */
    // hw_commands_[0] = 0;
    // hw_commands_[1] = 0;
    // hw_commands_[2] = 1;
    // hw_commands_[3] = 0;
    // hw_commands_[4] = 0;

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // Exactly three command interface on each joint
      if (joint.command_interfaces.size() != 3)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ZX200UpperArmPositionHardware"),
            "Joint '%s' has %zu command interfaces found. 3 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Exactly one state interface on each joint
      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ZX200UpperArmPositionHardware"),
            "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("ZX200UpperArmPositionHardware"),
            "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ZX200UpperArmPositionHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ZX200UpperArmPositionHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  ZX200UpperArmPositionHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  ZX200UpperArmPositionHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(
      //     info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn ZX200UpperArmPositionHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("ZX200UpperArmPositionHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn ZX200UpperArmPositionHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type ZX200UpperArmPositionHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    predicted_positions = imu_joint_values;    // TODO: Compensate deadtime
    position_states_ = predicted_positions_;

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type ZX200UpperArmPositionHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Send command
    joint_cmd_msg_.position = position_commands_;
    joint_cmd_msg_.velocity = velocity_commands_;
    joint_cmd_msg_.effort = effort_commands_;
    joint_cmd_pub_->publish(joint_cmd_msg_);

    // angle_cmd_.data = hw_commands_[0];
    // swing_setpoint_pub_->publish(angle_cmd_);
    // angle_cmd_.data = hw_commands_[1];
    // boom_setpoint_pub_->publish(angle_cmd_);
    // angle_cmd_.data = hw_commands_[2];
    // arm_setpoint_pub_->publish(angle_cmd_);
    // angle_cmd_.data = hw_commands_[3];
    // bucket_setpoint_pub_->publish(angle_cmd_);

    // std_msgs::msg::Float64 tmp_state;
    // // /joint_statesに/ac58_joint_publisher/joint_states を常に反映するため
    // hw_states_[0]=ac58_joint_states_[0];
    // hw_states_[1]=ac58_joint_states_[1];
    // hw_states_[2]=ac58_joint_states_[2];
    // hw_states_[3]=ac58_joint_states_[3]; 

    // tmp_state.data=hw_states_[0];
    // swing_state_pub_->publish(tmp_state);
    // tmp_state.data=hw_states_[1];
    // boom_state_pub_->publish(tmp_state);
    // tmp_state.data=hw_states_[2];
    // arm_state_pub_->publish(tmp_state);
    // tmp_state.data=hw_states_[3];
    // bucket_state_pub_->publish(tmp_state);
    
    return hardware_interface::return_type::OK;
  }

  void ZX200UpperArmPositionHardware::imu_js_callback(const sensor_msgs::msg::JointState &msg)
  {
    imu_joint_values_ = msg.position;
    // for (int i = 0; i < msg.position.size(); i++)
    // {
    //     if(msg.name[i] == "swing_joint"){
    //         ac58_joint_states_[0] = msg.position[i];
    //     }else if (msg.name[i] == "boom_joint"){
    //         ac58_joint_states_[1] = msg.position[i];
    //     }else if (msg.name[i] == "arm_joint"){
    //         ac58_joint_states_[2] = msg.position[i];
    //     }else if (msg.name[i] == "bucket_joint"){
    //         ac58_joint_states_[3] = msg.position[i];
    //     }
    // }
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ZX200_control_hardware::ZX200UpperArmPositionHardware, hardware_interface::SystemInterface)
