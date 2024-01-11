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
#include "former_interfaces/msg/robot_feedback.hpp"


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

            feedback_lamp_color_ = 99;
            feedback_lamp_mode_ = 99;
            req_lamp_color_ = 5;
            req_lamp_mode_ = 2;
            is_estop_pressed_ = false;
            watchdog_count_ = 0;

            prev_lamp_color_ = 5;
            prev_lamp_mode_ = 2;

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
            rclcpp::sleep_for(std::chrono::milliseconds(2000));
            // ser_.FlushIOBuffers();

            pub_l_sonar_range_ = this->create_publisher<sensor_msgs::msg::Range>("l_sonar_range", 10);
            pub_r_sonar_range_ = this->create_publisher<sensor_msgs::msg::Range>("r_sonar_range", 10);
            pub_dock_state_ = this->create_publisher<std_msgs::msg::Bool>("dock_state", 10);

            sub_robot_feedback_ = this->create_subscription<former_interfaces::msg::RobotFeedback>(
                "former_io_controller/robot_feedback", 10, std::bind(&FormerGPIOBoardNode::callback_robot_feedback, this, std::placeholders::_1));

            srv_set_lamp_ = this->create_service<former_interfaces::srv::SetLamp>("set_lamp",
                                        std::bind(&FormerGPIOBoardNode::set_lamp_callback, this, _1, _2));

            auto period = std::chrono::duration<double>(1.0 / this->get_parameter("rate").as_double());
            timer_ = this->create_wall_timer(period, std::bind(&FormerGPIOBoardNode::timer_callback, this));

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }

        ~FormerGPIOBoardNode() {}

    private:
        void timer_callback()
        {
            is_docking_ = false;

            try
            {
                // ser_.FlushIOBuffers();

                std::vector<uint8_t> send_req {0xfa, 0xfe, 0x3, 0x1, 0x4, 0xfa, 0xfd};
                ser_.Write(send_req);
                ser_.DrainWriteBuffer();

                std::vector<uint8_t> recv_buf(14, 0);
                ser_.Read(recv_buf, 14, 100);

                if(recv_buf[2] != 0x93)
                {
                    return;
                }

                feedback_lamp_color_ = recv_buf[8];
                feedback_lamp_mode_ = recv_buf[9];

                auto l_sonar_msg = sensor_msgs::msg::Range();
                l_sonar_msg.header.frame_id = "l_sonar_sensor";
                l_sonar_msg.header.stamp = this->now();
                l_sonar_msg.radiation_type = 0;
                l_sonar_msg.field_of_view = 9.0 / 180.0 * M_PI;
                l_sonar_msg.min_range = 0.01;
                l_sonar_msg.max_range = 2.0;
                l_sonar_msg.range =  ((uint16_t)((recv_buf[3] << 8) + (recv_buf[4])) * 6.0 - 300.0) / 1000.0;

                auto r_sonar_msg = sensor_msgs::msg::Range();
                r_sonar_msg.header.frame_id = "r_sonar_sensor";
                r_sonar_msg.header.stamp = this->now();
                r_sonar_msg.radiation_type = 0;
                r_sonar_msg.field_of_view = 9.0 / 180.0 * M_PI;
                r_sonar_msg.min_range = 0.01;
                r_sonar_msg.max_range = 2.0;
                r_sonar_msg.range = ((uint16_t)((recv_buf[5] << 8) + (recv_buf[6])) * 6.0 - 300.0) / 1000.0;

                auto dock_state_msg = std_msgs::msg::Bool();
                dock_state_msg.data = recv_buf[7] == 0 ? false : true;
                is_docking_ = dock_state_msg.data;

                pub_l_sonar_range_->publish(l_sonar_msg);
                pub_r_sonar_range_->publish(r_sonar_msg);
                pub_dock_state_->publish(dock_state_msg);
            }
            catch(LibSerial::ReadTimeout &e)
            {
                // RCLCPP_INFO(this->get_logger(), "%d...", ser_.GetNumberOfBytesAvailable());
                // RCLCPP_INFO(this->get_logger(), "Timeout...");
                watchdog_count_++;
                assert(watchdog_count_ < 8);
                return;
            }
            watchdog_count_ = 0;


            if(is_docking_)
            {
                if(!is_charging_)
                {
                    prev_lamp_color_ = req_lamp_color_;
                    prev_lamp_mode_ = req_lamp_mode_;

                    req_lamp_color_ = 3;
                    req_lamp_mode_ = 2;
                    is_charging_ = true;
                }
            }
            else
            {
                if(is_charging_)
                {
                    req_lamp_color_ = prev_lamp_color_;
                    req_lamp_mode_ = prev_lamp_mode_;

                    is_charging_ = false;
                }
            }


            if(req_lamp_color_ != feedback_lamp_color_)
            {
                RCLCPP_INFO(this->get_logger(), "Color changed from %d to %d...", feedback_lamp_color_, req_lamp_color_);

                std::vector<uint8_t> send_color {0xfa, 0xfe, 0x1, 0x0, 0x0, 0x3, 0x0, 0xfa, 0xfd};
                send_color[3] = req_lamp_color_;
                uint16_t sum = 0;
                for(int i = 0; i < 4; i++)
                {
                    sum += send_color[2 + i];
                }
                send_color[6] = (uint8_t)sum;
                ser_.Write(send_color);
                ser_.DrainWriteBuffer();
            }
            if(req_lamp_mode_ != feedback_lamp_mode_)
            {
                RCLCPP_INFO(this->get_logger(), "Mode changed from %d to %d ", feedback_lamp_mode_, req_lamp_mode_);

                std::vector<uint8_t> send_mode {0xfa, 0xfe, 0x2, 0x0, 0x0, 0x3, 0x0, 0xfa, 0xfd};
                send_mode[3] = req_lamp_mode_;
                uint16_t sum = 0;
                for(int i = 0; i < 4; i++)
                {
                    sum += send_mode[2 + i];
                }
                send_mode[6] = (uint8_t)sum;
                ser_.Write(send_mode);
                ser_.DrainWriteBuffer();
            }
        }

        void set_lamp_callback(const std::shared_ptr<former_interfaces::srv::SetLamp::Request> req,
                                            std::shared_ptr<former_interfaces::srv::SetLamp::Response> res)
        {
            if(req->color <= 5 && req->mode <= 2)
            {
                req_lamp_color_ = req->color;
                req_lamp_mode_ = req->mode;

                res->success = true;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Out of range for color [0:5] & mode [0:2]...");
                res->success = false;
            }
        }

        void callback_robot_feedback(const former_interfaces::msg::RobotFeedback & msg)
        {
            if(msg.estop_button && !is_estop_pressed_)
            {
                last_lamp_color_ = feedback_lamp_color_;
                last_lamp_mode_ = feedback_lamp_mode_;

                req_lamp_color_ = 2;
                req_lamp_mode_ = 1;

                is_estop_pressed_ = true;
            }
            else if(!msg.estop_button && is_estop_pressed_)
            {
                req_lamp_color_ = last_lamp_color_;
                req_lamp_mode_ = last_lamp_mode_;

                is_estop_pressed_ = false;
            }
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        LibSerial::SerialPort ser_;
        int feedback_lamp_color_;
        int req_lamp_color_;
        int feedback_lamp_mode_;
        int req_lamp_mode_;


        bool is_docking_;
        bool is_charging_;
        int prev_lamp_color_;
        int prev_lamp_mode_;

        int last_lamp_color_;
        int last_lamp_mode_;
        bool is_estop_pressed_;
        int watchdog_count_;

        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_l_sonar_range_;
        rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_r_sonar_range_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_dock_state_;
        rclcpp::Subscription<former_interfaces::msg::RobotFeedback>::SharedPtr sub_robot_feedback_;

        rclcpp::Service<former_interfaces::srv::SetLamp>::SharedPtr srv_set_lamp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormerGPIOBoardNode>());
    rclcpp::shutdown();
    return 0;
}
