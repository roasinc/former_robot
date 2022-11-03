#include "former_hardware_interface/former_hardware_interface.hpp"

#include <math.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace former_hardware_interface
{
hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    auto port_name = info_.hardware_parameters["port_name"];
    auto baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Get port_name [%s] and baudrate [%d]", port_name.c_str(), baudrate);

    // open serial port, initialize


    // Joint variables Initialize
    hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(info_.joints.size(), 0.0);//std::numeric_limits<double>::quiet_NaN());
    hw_states_velocities_.resize(info_.joints.size(), 0.0);//std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "on_init: success");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FormerSystemHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FormerSystemHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        if(info_.joints[i].command_interfaces[0].name == "position")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]
                )
            );
        }
        else if(info_.joints[i].command_interfaces[0].name == "velocity")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]
                )
            );
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "on_activate");
    // reset motor driver and initialize. And then, start motor driver


    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "on_deactivate");
    // stop motor driver

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FormerSystemHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "read");
    // read current state from motor driver

    hw_states_positions_[0] = 0.0;
    hw_states_positions_[1] = 0.0;
    hw_states_velocities_[0] = 0.0;
    hw_states_velocities_[1] = 0.0;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FormerSystemHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration & /*period*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "write");
    // write command to motor driver

    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(former_hardware_interface::FormerSystemHardwareInterface, hardware_interface::SystemInterface)