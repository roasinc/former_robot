#include <string>
#include <vector>
#include <csignal>
#include <fcntl.h>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "former_interfaces/srv/set_lamp.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class FormerGPIOBoardNode : public rclcpp::Node
{
    public:
        FormerGPIOBoardNode() : Node("former_gpio_board")
        {
            this->declare_parameter<std::string>("port_name", "/dev/ttyACM0");
            this->declare_parameter<int>("baudrate", 115200);
            this->declare_parameter<double>("rate", 20.0);

            auto port_name = this->get_parameter("port_name").get_parameter_value().get<std::string>();
            auto baudrate = this->get_parameter("baudrate").get_parameter_value().get<long int>();
            RCLCPP_INFO(this->get_logger(), "Port: [%s] and baudrate %ld", port_name.c_str(), baudrate);

            last_lamp_color_ = 0;
            last_lamp_mode_ = 2;

            // open serial port, initialize
            try
            {
                ser_.Open(port_name);
            }
            catch(LibSerial::OpenFailed &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exceptions: \033[0;91m%s\033[0m", e.what());
                RCLCPP_ERROR(this->get_logger(), "\033[0;91mFailed to open port\033[0m [\033[0;92m%s\033[0m]...", port_name.c_str());
                assert(false);
            }

            ser_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            rclcpp::sleep_for(std::chrono::milliseconds(1500));
            ser_.FlushIOBuffers();

            pub_l_sonar_range_ = this->create_publisher<sensor_msgs::msg::Range>("l_sonar_range", 10);
            pub_r_sonar_range_ = this->create_publisher<sensor_msgs::msg::Range>("r_sonar_range", 10);
            pub_dock_state_ = this->create_publisher<std_msgs::msg::Bool>("dock_state", 10);

            srv_set_lamp_ = this->create_service<former_interfaces::srv::SetLamp>("set_lamp",
                                        std::bind(&FormerGPIOBoardNode::set_lamp_callback, this, _1, _2));


            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&FormerGPIOBoardNode::timer_callback, this));


            last_lamp_color_ = 3;
            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }

        ~FormerGPIOBoardNode() {}

    private:
        void timer_callback()
        {
            auto lamp_command_str = boost::format("!C:%1%:\r!M:%2%:\r") % last_lamp_color_ % last_lamp_mode_;
            ser_.Write(lamp_command_str.str());
            ser_.DrainWriteBuffer();

            try
            {
                ser_.Write("!F:\r");
                ser_.DrainWriteBuffer();

                std::string recv_data;
                ser_.ReadLine(recv_data, '\n', 50);

                // IO:114:110:0:
                auto data = recv_data.substr(3);
                std::vector<std::string> data_array;
                boost::split(data_array, data, boost::algorithm::is_any_of(":"));

                auto l_sonar_msg = sensor_msgs::msg::Range();
                l_sonar_msg.header.frame_id = "l_sonar_sensor";
                l_sonar_msg.header.stamp = this->now();
                l_sonar_msg.radiation_type = 0;
                l_sonar_msg.field_of_view = 9.0 / 180.0 * M_PI;
                l_sonar_msg.min_range = 0.01;
                l_sonar_msg.max_range = 2.0;
                l_sonar_msg.range = (atof(data_array[0].c_str()) * 6.0 - 300.0) / 1000.0;

                auto r_sonar_msg = sensor_msgs::msg::Range();
                r_sonar_msg.header.frame_id = "r_sonar_sensor";
                r_sonar_msg.header.stamp = this->now();
                r_sonar_msg.radiation_type = 0;
                r_sonar_msg.field_of_view = 9.0 / 180.0 * M_PI;
                r_sonar_msg.min_range = 0.01;
                r_sonar_msg.max_range = 2.0;
                r_sonar_msg.range = (atof(data_array[1].c_str()) * 6.0 - 300.0) / 1000.0;

                auto dock_state_msg = std_msgs::msg::Bool();
                dock_state_msg.data = atoi(data_array[2].c_str()) == 0 ? false : true;

                pub_l_sonar_range_->publish(l_sonar_msg);
                pub_r_sonar_range_->publish(r_sonar_msg);
                pub_dock_state_->publish(dock_state_msg);
            }
            catch(LibSerial::ReadTimeout &e)
            {
                return;
            }
        }

        void set_lamp_callback(const std::shared_ptr<former_interfaces::srv::SetLamp::Request> req,
                                            std::shared_ptr<former_interfaces::srv::SetLamp::Response> res)
        {
            if(req->color <= 5 && req->mode <= 2)
            {
                last_lamp_color_ = req->color;
                last_lamp_mode_ = req->mode;

                res->success = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Out of range for color [0:5] & mode [0:2]...");
                res->success = false;
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        LibSerial::SerialPort ser_;
        int last_lamp_color_;
        int last_lamp_mode_;

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_l_sonar_range_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_r_sonar_range_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dock_state_;

        rclcpp::Service<former_interfaces::srv::SetLamp>::SharedPtr srv_set_lamp_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormerGPIOBoardNode>());
    rclcpp::shutdown();
    return 0;
}