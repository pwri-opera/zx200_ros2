#include <cstdio>
#include <limits>
#include <algorithm>

#include "zx200_control/upper_arm_unity_hardware.hpp"

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace zx200_control
{

hardware_interface::CallbackReturn
Zx200UpperArmPositionUnityHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // ベースクラス初期化
  if (hardware_interface::SystemInterface::on_init(info)
      != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // rclcppノード生成
  node_ = rclcpp::Node::make_shared("upper_arm_unity_hw");

  // Unity からの JointState を購読
  imu_js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/zx200/joint_states", 100,
      [this](const sensor_msgs::msg::JointState & msg)
      {
        imu_js_callback(msg);
      });

  // com3_msgs/JointCmd を Unity 側へ送信
  joint_cmd_pub_ =
      node_->create_publisher<com3_msgs::msg::JointCmd>("/zx200/front_cmd", 100);

  // ノードを別スレッドで spin
  node_thread_ = std::thread([this]()
  {
    rclcpp::spin(node_);
  });

  // JointCmd / JointState のサイズを info_.joints に合わせる
  joint_cmd_msg_.joint_name.resize(info_.joints.size());
  latest_joint_states_.name.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_cmd_msg_.joint_name[i] = info_.joints[i].name;
    latest_joint_states_.name[i] = info_.joints[i].name;
  }

  joint_cmd_msg_.position.resize(info_.joints.size(), 0.0);
  joint_cmd_msg_.velocity.resize(info_.joints.size(), 0.0);
  joint_cmd_msg_.effort.resize(info_.joints.size(), 0.0);

  latest_joint_states_.position.resize(info_.joints.size(), 0.0);
  latest_joint_states_.velocity.resize(info_.joints.size(), 0.0);
  latest_joint_states_.effort.resize(info_.joints.size(), 0.0);

  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  position_commands_.resize(info_.joints.size(), 0.0);

  // インタフェースのチェック
    for (const auto & joint : info_.joints)
    {
    // command_interface は 1つだけ
    if (joint.command_interfaces.size() != 1)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
            "Joint '%s' has %zu command interfaces. 1 expected.",
            joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // command_interface は position 固定
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
            "Joint '%s' command interface '%s' is not '%s'.",
            joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // ★ state_interface は「少なくとも position を一個は持っていればOK」
    if (joint.state_interfaces.empty())
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
            "Joint '%s' has no state interfaces. At least 'position' is required.",
            joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position = false;
    for (const auto & si : joint.state_interfaces)
    {
        if (si.name == hardware_interface::HW_IF_POSITION)
        {
        has_position = true;
        break;
        }
    }

    if (!has_position)
    {
        RCLCPP_FATAL(
            rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
            "Joint '%s' does not have a 'position' state interface.",
            joint.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // velocity があれば後で export_state_interfaces で拾う、無くても OK
    }





  RCLCPP_INFO(
      rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
      "on_init done.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmPositionUnityHardware::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
      "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Zx200UpperArmPositionUnityHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];

    for (const auto & si : joint.state_interfaces)
    {
      if (si.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.name,
                hardware_interface::HW_IF_POSITION,
                &position_states_[i]));
      }
      else if (si.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                joint.name,
                hardware_interface::HW_IF_VELOCITY,
                &velocity_states_[i]));
      }
      else
      {
        // 今回は position/velocity 以外は想定していないので警告を出す程度でOK
        RCLCPP_WARN(
            rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
            "Joint '%s' has unsupported state interface '%s' (ignored).",
            joint.name.c_str(), si.name.c_str());
      }
    }
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
Zx200UpperArmPositionUnityHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
Zx200UpperArmPositionUnityHardware::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
      "Successfully activated!");

  // 必要なら、ここで latest_joint_states_ を state にコピーしておくことも可能
  // for (size_t i = 0; i < info_.joints.size(); ++i) {
  //   position_states_[i] = latest_joint_states_.position[i];
  //   velocity_states_[i] = latest_joint_states_.velocity[i];
  // }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
Zx200UpperArmPositionUnityHardware::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
      rclcpp::get_logger("Zx200UpperArmPositionUnityHardware"),
      "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
Zx200UpperArmPositionUnityHardware::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
  // ★ Unity から来た JointState をそのまま state に反映
  //   （名前順が一致している前提）
  const size_t n = std::min(info_.joints.size(), latest_joint_states_.position.size());

  for (size_t i = 0; i < n; ++i)
  {
    position_states_[i] = latest_joint_states_.position[i];
    velocity_states_[i] = latest_joint_states_.velocity[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
Zx200UpperArmPositionUnityHardware::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
  // ★ controller からもらった position_commands_ をそのまま publish
  joint_cmd_msg_.position = position_commands_;

  // velocity / effort はここでは 0 にしておく
  std::fill(joint_cmd_msg_.velocity.begin(),
            joint_cmd_msg_.velocity.end(), 0.0);
  std::fill(joint_cmd_msg_.effort.begin(),
            joint_cmd_msg_.effort.end(), 0.0);

  joint_cmd_pub_->publish(joint_cmd_msg_);

  return hardware_interface::return_type::OK;
}

void
Zx200UpperArmPositionUnityHardware::imu_js_callback(
    const sensor_msgs::msg::JointState & msg)
{
  // ★ Unity 側からの最新状態を丸ごと保存
  latest_joint_states_ = msg;
}

}  // namespace zx200_control

// pluginlib 登録
PLUGINLIB_EXPORT_CLASS(
  zx200_control::Zx200UpperArmPositionUnityHardware,
  hardware_interface::SystemInterface)
