#include "rebel_arm_hardware_interface/rebel_arm_hardware_interface.hpp"

namespace rebel_arm_hardware_interface
{
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        try {
            hardwareConfig.position1JointName = info.hardware_parameters.at("position1_joint_name");
            hardwareConfig.position2JointName = info.hardware_parameters.at("position2_joint_name");
            hardwareConfig.position3JointName = info.hardware_parameters.at("position3_joint_name");
            hardwareConfig.position4JointName = info.hardware_parameters.at("position4_joint_name");
            hardwareConfig.position5JointName = info.hardware_parameters.at("position5_joint_name");
            hardwareConfig.position6JointName = info.hardware_parameters.at("position6_joint_name");

            serialPortConfig.device = info.hardware_parameters.at("device");
            serialPortConfig.baudRate = std::stoi(info.hardware_parameters.at("baud_rate"));
            serialPortConfig.timeout = std::stoi(info.hardware_parameters.at("timeout"));
        } catch (const std::out_of_range & e) {
            RCLCPP_FATAL(rclcpp::get_logger("RebelArmHardwareInterface"), "Missing hardware parameters: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        } catch (const std::exception & e) {
            RCLCPP_FATAL(rclcpp::get_logger("RebelArmHardwareInterface"), "Error parsing hardware parameters: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }

        rebel_arm_joint1 = RebelArmJoint(hardwareConfig.position1JointName);
        rebel_arm_joint2 = RebelArmJoint(hardwareConfig.position2JointName);
        rebel_arm_joint3 = RebelArmJoint(hardwareConfig.position3JointName);
        rebel_arm_joint4 = RebelArmJoint(hardwareConfig.position4JointName);
        rebel_arm_joint5 = RebelArmJoint(hardwareConfig.position5JointName);
        rebel_arm_joint6 = RebelArmJoint(hardwareConfig.position6JointName);

        for (const auto & joint : info.joints) {
            if (joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has %zu command interfaces. Expected 1.", joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has command interface '%s'. Expected '%s'.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has %zu state interfaces. Expected 1.", joint.name.c_str(), joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RebelArmHardwareInterface"),
                    "Joint '%s' has state interface '%s'. Expected '%s'.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // No changes in these functions
    std::vector<hardware_interface::StateInterface> RebelArmHardwareInterface::export_state_interfaces() { /* ... */ }
    std::vector<hardware_interface::CommandInterface> RebelArmHardwareInterface::export_command_interfaces() { /* ... */ }
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &) { /* ... */ }
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State &) { /* ... */ }
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &) { /* ... */ }
    hardware_interface::CallbackReturn RebelArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &) { /* ... */ }
    hardware_interface::return_type RebelArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &) { /* ... */ }
    hardware_interface::return_type RebelArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &) { /* ... */ }
    void RebelArmHardwareInterface::rebelArmFeedbackCallback(RebelArmFeedback) { /* ... */ }
}

PLUGINLIB_EXPORT_CLASS(rebel_arm_hardware_interface::RebelArmHardwareInterface, hardware_interface::SystemInterface)

