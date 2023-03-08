#include "former_controllers/gpio_controller.hpp"
#include <string>

namespace former_controllers
{
    controller_interface::CallbackReturn GPIOController::on_init()
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_init...");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("gpio/set_enable_motor");

        RCLCPP_INFO(get_node()->get_logger(), "command_interface_configuration...");
        return config;
    }

    controller_interface::InterfaceConfiguration GPIOController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        config.names.emplace_back("gpio/motor_enabled");
        config.names.emplace_back("gpio/estop_button_state");
        config.names.emplace_back("gpio/system_voltage");
        config.names.emplace_back("gpio/charging_voltage");
        config.names.emplace_back("gpio/user_power_current1");
        config.names.emplace_back("gpio/user_power_current2");
        config.names.emplace_back("gpio/curent_temperature");
        config.names.emplace_back("gpio/fault_flags");

        RCLCPP_INFO(get_node()->get_logger(), "state_interface_configuration...");
        return config;
    }

    controller_interface::return_type GPIOController::update(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        enable_motor_state_ = static_cast<bool>(state_interfaces_[StateInterfaces::ENABLE_MOTOR_STATE].get_value());
        estop_button_state_ = static_cast<bool>(state_interfaces_[StateInterfaces::ESTOP_BUTTON_STATE].get_value());
        system_voltage_ = static_cast<double>(state_interfaces_[StateInterfaces::SYSTEM_VOLTAGE].get_value());
        charging_voltage_ = static_cast<double>(state_interfaces_[StateInterfaces::CHARGING_VOLTAGE].get_value());
        user_power_current1_ = static_cast<double>(state_interfaces_[StateInterfaces::USER_POWER_CURRENT1].get_value());
        user_power_current2_ = static_cast<double>(state_interfaces_[StateInterfaces::USER_POWER_CURRENT2].get_value());
        current_temperature_ = static_cast<double>(state_interfaces_[StateInterfaces::CURRENT_TEMPERATURE].get_value());
        fault_flag_ = static_cast<uint8_t>(state_interfaces_[StateInterfaces::FAULT_FLAG].get_value());

        auto feedback_msg = former_interfaces::msg::RobotFeedback();

        feedback_msg.header.stamp = get_node()->now();

        feedback_msg.motor_enabled = enable_motor_state_;
        feedback_msg.estop_button = estop_button_state_;
        feedback_msg.system_voltage = system_voltage_;
        feedback_msg.charging_voltage = charging_voltage_;
        feedback_msg.user_power_current1 = user_power_current1_;
        feedback_msg.user_power_current2 = user_power_current2_;
        feedback_msg.current_temperature = current_temperature_;
        feedback_msg.fault_flag = fault_flag_;

        pub_robot_feedback_->publish(feedback_msg);
        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_node()->get_logger(), "on_configure...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        try
        {
            pub_robot_feedback_ = get_node()->create_publisher<former_interfaces::msg::RobotFeedback>("~/robot_feedback", rclcpp::SystemDefaultsQoS());
            // sub_enable_motor_ = get_node()->create_subscription<std_msgs::msg::Bool>(
            //     "~/enable_motor", 10, std::bind(&GPIOController::callback_enable_motor_command, this, std::placeholders::_1));
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_node()->get_logger(), "on_activate...");
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        try
        {
            pub_robot_feedback_.reset();
            // sub_enable_motor_.reset();
        }
        catch (...)
        {
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // void GPIOController::callback_enable_motor_command(const std_msgs::msg::Bool & msg)
    // {
    //     command_interfaces_[CommandInterfaces::ENABLE_MOTOR_CMD].set_value(msg.data ? 1.0 : 0.0);
    // }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(former_controllers::GPIOController, controller_interface::ControllerInterface)