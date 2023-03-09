#include "former_hardware_interface/former_hardware_interface.hpp"

#include <math.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace former_hardware_interface
{
hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    // Get info parameters from URDF
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Name: %s", info_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Number of Joints %zu", info_.joints.size());

    // Initialize hardware_interface
    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Check the URDF Data
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),  joint.command_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size()
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("FormerSystemHardwareInterface"),
                "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
                joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_EFFORT
            );
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    l_last_enc_ = 0;
    r_last_enc_ = 0;

    enable_motor_cmd_ = 0.0;
    enable_motor_state_= 0.0;
    estop_button_state_= 0.0;
    system_voltage_= 0.0;
    charging_voltage_= 0.0;
    user_power_current1_= 0.0;
    user_power_current2_= 0.0;
    current_temperature_= 0.0;
    fault_flags_= 0.0;

    is_estop_processed_ = estop_button_state_;

    auto port_name = info_.hardware_parameters["port_name"];
    auto baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Get port_name [%s] and baudrate [%d]", port_name.c_str(), baudrate);

    // open serial port, initialize
    try
    {
        ser_.Open(port_name);
    }
    catch(LibSerial::OpenFailed &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FormerSystemHardwareInterface"), "Exceptions: \033[0;91m%s\033[0m", e.what());
        RCLCPP_ERROR(rclcpp::get_logger("FormerSystemHardwareInterface"), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    ser_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    ser_.Write("^ECHOF 1\r"); // Disable Serial ECHO
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Wait connection to Motor Driver");
    while(rclcpp::ok())
    {
        ser_.Write("?FID\r"); // Disable Serial ECHO
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        try
        {
            std::string recv_version;
            ser_.ReadLine(recv_version, '\r', 500);

            if(recv_version[0] == 'F' && recv_version[1] == 'I' && recv_version[2] == 'D')
            {
                RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Connected %s", recv_version.c_str());
                break;
            }
        }
        catch(LibSerial::ReadTimeout &e)
        {
            assert(false);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    ser_.Write("!R 2\r"); // Restart Script
    rclcpp::sleep_for(std::chrono::milliseconds(2000));
    ser_.FlushIOBuffers();

    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "Successfully initialized!");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FormerSystemHardwareInterface::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "export_state_interfaces");

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }

    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "motor_enabled", &enable_motor_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "estop_button_state", &estop_button_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "system_voltage", &system_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "charging_voltage", &charging_voltage_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "user_power_current1", &user_power_current1_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "user_power_current2", &user_power_current2_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "curent_temperature", &current_temperature_));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "fault_flags", &fault_flags_));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FormerSystemHardwareInterface::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(size_t i = 0; i < info_.joints.size(); i++)
    {
        if(info_.joints[i].command_interfaces[0].name == "velocity")
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]
                )
            );
        }
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "set_enable_motor", &enable_motor_cmd_));

    return command_interfaces;
}

hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    // reset motor driver and initialize. And then, start motor driver
    ser_.Write("!C 1 0_!C 2 0\r");
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    ser_.Write("!MG\r");    // Enable Motor
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    ser_.FlushIOBuffers();


    for (auto i = 0u; i < hw_positions_.size(); i++)
    {
        if (std::isnan(hw_positions_[i]))
        {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_efforts_[i] = 0;
            hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "activated...");
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FormerSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "on_deactivate");

    ser_.Write("!EX\r");
    rclcpp::sleep_for(std::chrono::milliseconds(10));
    std::string ret;
    ser_.ReadLine(ret, '\r', 100);
    // stop motor driver

    // ser_.Write("!R 0\r"); // Stop Script from START
    // rclcpp::sleep_for(std::chrono::milliseconds(100));
    // ser_.FlushIOBuffers();

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FormerSystemHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "read");
    // struct timespec start, end;
    // clock_gettime(CLOCK_MONOTONIC, &start);

    ser_.Write("?A_?AI_?C_?FF_?T 1_?V 2_?DI\r");
    ser_.DrainWriteBuffer();

    double motor_current[2] = {0, };
    long int current_encoder[2] = {0, };

    while(rclcpp::ok())
    {
        std::string recv_data;
        try
        {
            ser_.ReadLine(recv_data, '\r', 100);
            RCLCPP_DEBUG(rclcpp::get_logger("FormerSystemHardwareInterface"), "%s", recv_data.c_str());
            if(recv_data[0] == 'A' && recv_data[1] == '=')
            {
                auto data = recv_data.substr(2);
                std::vector<std::string> data_array;
                boost::split(data_array, data, boost::algorithm::is_any_of(":"));
                motor_current[0] = atof(data_array[0].c_str()) / 10.0;
                motor_current[1] = atof(data_array[1].c_str()) / 10.0;
            }
            if(recv_data[0] == 'A' && recv_data[1] == 'I')
            {
                auto data = recv_data.substr(3);
                std::vector<std::string> data_array;
                boost::split(data_array, data, boost::algorithm::is_any_of(":"));
                charging_voltage_ = atof(data_array[2].c_str());
                user_power_current1_ = atof(data_array[3].c_str()) / 1000.0;
                user_power_current2_ = atof(data_array[4].c_str()) / 1000.0;
            }
            if(recv_data[0] == 'C' && recv_data[1] == '=')
            {
                auto data = recv_data.substr(2);
                std::vector<std::string> data_array;
                boost::split(data_array, data, boost::algorithm::is_any_of(":"));
                current_encoder[0] = atoi(data_array[0].c_str());
                current_encoder[1] = atoi(data_array[1].c_str());
            }
            if(recv_data[0] == 'F' && recv_data[1] == 'F')
            {
                auto data = recv_data.substr(3);
                enable_motor_state_ = atoi(data.c_str()) == 16 ? 0.0 : 1.0;
                fault_flags_ = atoi(data.c_str());
            }
            if(recv_data[0] == 'T' && recv_data[1] == '=')
            {
                auto data = recv_data.substr(2);
                current_temperature_ = atof(data.c_str());
            }
            if(recv_data[0] == 'V' && recv_data[1] == '=')
            {
                auto data = recv_data.substr(2);
                system_voltage_ = atof(data.c_str()) / 10.0;
            }
            if(recv_data[0] == 'D' && recv_data[1] == 'I')
            {
                auto data = recv_data.substr(3);
                std::vector<std::string> data_array;
                boost::split(data_array, data, boost::algorithm::is_any_of(":"));
                estop_button_state_ = atoi(data_array[0].c_str()) == 1 ? 0.0 : 1.0;
                break;
            }
        }
        catch(LibSerial::ReadTimeout &e)
        {
            // assert(false && "Timeout for read motor states...");
            break;
        }
    }

    // clock_gettime(CLOCK_MONOTONIC, &end);
    // auto diff = (end.tv_sec - start.tv_sec) * 1000000000LL + (end.tv_nsec - start.tv_nsec);
    // RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "READ: %lld", diff);

    hw_positions_[0] += (double)(l_last_enc_ - current_encoder[0]) / 16384.0 * (2.0 * M_PI) * -1.0;
    hw_positions_[1] += (double)(r_last_enc_ - current_encoder[1]) / 16384.0 * (2.0 * M_PI) * -1.0;
    hw_velocities_[0] = 0.0;
    hw_velocities_[1] = 0.0;
    hw_efforts_[0] = motor_current[0];
    hw_efforts_[1] = motor_current[1];

    l_last_enc_ = current_encoder[0];
    r_last_enc_ = current_encoder[1];

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type FormerSystemHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration & /*period*/)
{
    // RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "write");
    // write command to motor driver

    int16_t l_rpm = hw_commands_[0] / (2.0 * M_PI) * 60.0;
    int16_t r_rpm = hw_commands_[1] / (2.0 * M_PI) * 60.0;

    int16_t cmd_l_motor = l_rpm / 200.0 * 1000.0;
    int16_t cmd_r_motor = r_rpm / 200.0 * 1000.0;

    // RCLCPP_INFO(rclcpp::get_logger("FormerSystemHardwareInterface"), "%f %f %d %d", hw_commands_[0], hw_commands_[1], l_rpm, r_rpm);

    auto cmd_str = boost::format("");
    if(enable_motor_state_ == 1.0 && estop_button_state_ == 0)
    {
        cmd_str = boost::format("!G 1 %1%_!G 2 %2%}\r") % cmd_l_motor % cmd_r_motor;
        RCLCPP_DEBUG(rclcpp::get_logger("FormerSystemHardwareInterface"), "%s", cmd_str.str().c_str());
    }

    if(is_estop_processed_ != estop_button_state_)
    {
        if(estop_button_state_ == 1.0)
        {
            cmd_str = boost::format("!EX\r");
        }
        else
        {
            cmd_str = boost::format("!MG\r");
        }
        is_estop_processed_ = estop_button_state_;
    }

    ser_.Write(cmd_str.str());
    ser_.DrainWriteBuffer();
    ser_.FlushIOBuffers();

    return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(former_hardware_interface::FormerSystemHardwareInterface, hardware_interface::SystemInterface)