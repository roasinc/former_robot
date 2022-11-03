#ifndef FORMER_SYSTEM_HARDWARE_INTERFACE_HPP
#define FORMER_SYSTEM_HARDWARE_INTERFACE_HPP

#include <string>
#include <vector>
#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "former_hardware_interface/visibility_control.h"

namespace former_hardware_interface
{
class FormerSystemHardwareInterface: public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(FormerSystemHardwareInterface)

        FORMER_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        FORMER_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        //
    private:
        LibSerial::SerialPort ser_;

        std::vector<int32_t> last_encoder_value_;
        std::vector<double> hw_commands_positions_;
        std::vector<double> hw_commands_velocities_;
        std::vector<double> hw_states_positions_;
        std::vector<double> hw_states_velocities_;
};
} // namespace former_hardware_interface

#endif //FORMER_SYSTEM_HARDWARE_INTERFACE_HPP

