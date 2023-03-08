#ifndef FORMER_CONTROLLERS__GPIO_CONTROLLER_HPP_
#define FORMER_CONTROLLERS__GPIO_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"
#include "former_interfaces/msg/robot_feedback.hpp"


namespace former_controllers
{
    enum CommandInterfaces
    {
        ENABLE_MOTOR_CMD = 0u,
    };

    enum StateInterfaces
    {
        ENABLE_MOTOR_STATE = 0u,
        ESTOP_BUTTON_STATE = 1,
        SYSTEM_VOLTAGE = 2,
        CHARGING_VOLTAGE = 3,
        USER_POWER_CURRENT1 = 4,
        USER_POWER_CURRENT2 = 5,
        CURRENT_TEMPERATURE = 6,
        FAULT_FLAG = 7,
    };

    class GPIOController : public controller_interface::ControllerInterface
    {
        public:
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_init() override;

        private:

        private:
            bool enable_motor_state_;
            bool estop_button_state_;
            double system_voltage_;
            double charging_voltage_;
            double user_power_current1_;
            double user_power_current2_;
            double current_temperature_;
            uint8_t fault_flag_;

            std::shared_ptr<rclcpp::Publisher<former_interfaces::msg::RobotFeedback>> pub_robot_feedback_;
            // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_motor_;
    };
}

#endif // FORMER_CONTROLLERS__GPIO_CONTROLLER_HPP_