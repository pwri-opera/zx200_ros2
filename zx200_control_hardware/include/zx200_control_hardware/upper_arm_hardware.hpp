#ifndef UPPER_ARM_CONTROLLER_HPP_
#define UPPER_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "zx200_control_hardware/visibility_control.h"
#include "com3_msgs/msg/joint_cmd.hpp"

namespace ZX200_control_hardware
{
    class ZX200UpperArmPositionHardware : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(ZX200UpperArmPositionHardware)

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo &info) override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_configure(
                const rclcpp_lifecycle::State &previous_state) override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State &previous_state) override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State &previous_state) override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

            ZX200_CONTROL_HARDWARE_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

        private:
            //   void ac58_js_callback(const sensor_msgs::msg::JointState &msg);
            // std::vector<double> hw_commands_;
            // std::vector<double> hw_states_;
            std::vector<double> position_commands_; // for unity
            std::vector<double> velocity_commands_;
            std::vector<double> effort_commands_;
            std::vector<double> position_states_;

            std::vector<double> predicted_positions_; // predicted position
            // std::vector<double> velocity_states_;
            // std::vector<double> effort_states_;
            // std::vector<double> ac58_joint_states_;           
            std::vector<double> imu_joint_values_;

            std::shared_ptr<rclcpp::Node> node_;
            std::thread node_thread_;
            // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swing_setpoint_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr boom_setpoint_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_setpoint_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bucket_setpoint_pub_;

            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr swing_state_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr boom_state_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr arm_state_pub_;
            // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bucket_state_pub_;
            // sensor_msgs::msg::JointState joint_state_msg_;
            // std_msgs::msg::Float64 angle_cmd_;
            // rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ac58_js_sub_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr imu_js_sub_;
            com3_msgs::msg::JointCmd joint_cmd_msg_;
    };
}
#endif //UPPER_ARM_CONTROLLER_HPP_