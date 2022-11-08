#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "angles/angles.h"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/utils.h"

#include "former_interfaces/action/docking.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "former_auto_docking/visibility_control.h"
#include "former_auto_docking/line_extraction.h"


class FormerAutoDockingNode: public rclcpp::Node
{
    public:
        FORMER_AUTO_DOCKING_PUBLIC
        explicit FormerAutoDockingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("auto_docking_node", options)
        {
            tf_buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
            tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
            tf_br_.reset(new tf2_ros::StaticTransformBroadcaster(this));

            data_cached_ = false;
            is_docked_n_charging_ = false;

            this->declare_parameter("bearing_variance", 1e-5);
            this->declare_parameter("range_variance", 0.012);
            this->declare_parameter("least_sq_angle_threshold", 0.0001);
            this->declare_parameter("least_sq_radius_threshold ", 0.0001);
            this->declare_parameter("max_line_gap", 0.5);
            this->declare_parameter("mine_line_length", 0.1);
            this->declare_parameter("min_range", 0.5);
            this->declare_parameter("max_range", 250.0);
            this->declare_parameter("min_split_distance", 0.03);
            this->declare_parameter("outlier_distance", 0.06);
            this->declare_parameter("min_line_points", 10);
            this->declare_parameter("distance_approach", 0.265);

            line_extraction_.setBearingVariance(pow(this->get_parameter("bearing_variance").get_parameter_value().get<double>(), 2));
            line_extraction_.setRangeVariance(pow(this->get_parameter("range_variance").get_parameter_value().get<double>(), 2));
            line_extraction_.setLeastSqAngleThresh(this->get_parameter("least_sq_angle_threshold").get_parameter_value().get<double>());
            line_extraction_.setLeastSqRadiusThresh(this->get_parameter("least_sq_radius_threshold").get_parameter_value().get<double>());
            line_extraction_.setMaxLineGap(this->get_parameter("max_line_gap").get_parameter_value().get<double>());
            line_extraction_.setMinLineLength(this->get_parameter("mine_line_length").get_parameter_value().get<double>());
            line_extraction_.setMinRange(this->get_parameter("min_range").get_parameter_value().get<double>());
            line_extraction_.setMaxRange(this->get_parameter("max_range").get_parameter_value().get<double>());
            line_extraction_.setMinSplitDist(this->get_parameter("min_split_distance").get_parameter_value().get<double>());
            line_extraction_.setOutlierDist(this->get_parameter("outlier_distance").get_parameter_value().get<double>());
            line_extraction_.setMinLinePoints(this->get_parameter("min_line_points").get_parameter_value().get<uint16_t>());

            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", 10, std::bind(&FormerAutoDockingNode::odom_callback, this, std::placeholders::_1));
            pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);


            sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&FormerAutoDockingNode::scan_callback, this, std::placeholders::_1));
            pub_vis_lines_ = this->create_publisher<visualization_msgs::msg::Marker>("line_markers", 10);

            this->as_ = rclcpp_action::create_server<former_interfaces::action::Docking>(
                this,
                "docking",
                std::bind(&FormerAutoDockingNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FormerAutoDockingNode::handle_cancel, this, std::placeholders::_1),
                std::bind(&FormerAutoDockingNode::handle_accepted, this, std::placeholders::_1)
            );

            RCLCPP_INFO(this->get_logger(), "Initialized...");
        }
        ~FormerAutoDockingNode() {}

    private:
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const former_interfaces::action::Docking::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received request docking/undocking with mode %d", goal->mode);
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<former_interfaces::action::Docking>> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<former_interfaces::action::Docking>> goal_handle)
        {
            using namespace std::placeholders;
            std::thread{std::bind(&FormerAutoDockingNode::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<former_interfaces::action::Docking>> goal_handle)
        {
            auto feedback = std::make_shared<former_interfaces::action::Docking::Feedback>();
            auto result = std::make_shared<former_interfaces::action::Docking::Result>();
            auto goal = goal_handle->get_goal();

            /*
             * Docking
             */
            if(goal->mode == 0)  // Docking
            {
                if(is_docked_n_charging_)
                {
                    RCLCPP_INFO(this->get_logger(), "Already docked and on the charging...");
                    result->success = true;
                    result->message = "OK";
                    goal_handle->succeed(result);
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Start docking and charging...");

                geometry_msgs::msg::PoseStamped charger_center_pose;
                if(!find_charger_pose(charger_center_pose))
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to find the charging station...");
                    result->success = false;
                    result->message = "Failed to find the charging station.";
                    goal_handle->succeed(result);
                    return;
                }

                // Convert goal_pose to on odom frame
                geometry_msgs::msg::PoseStamped charger_goal_pose_odom;
                try
                {
                    geometry_msgs::msg::PoseStamped charger_center_pose_odom;
                    tf_buffer_->transform(charger_center_pose, charger_center_pose_odom, "odom", tf2::durationFromSec(1.0));

                    geometry_msgs::msg::TransformStamped tf_goal_tansform;
                    tf_goal_tansform.header.frame_id = "odom";
                    tf_goal_tansform.header.stamp = this->now();
                    tf_goal_tansform.child_frame_id = "charger_center";

                    tf2::Quaternion q;
                    double theta = tf2::getYaw(charger_center_pose_odom.pose.orientation);
                    q.setRPY(0, 0, theta);

                    tf2::convert(tf2::Transform(q,
                                tf2::Vector3(charger_center_pose_odom.pose.position.x, charger_center_pose_odom.pose.position.y, 0.0)),
                                    tf_goal_tansform.transform);

                    tf_br_->sendTransform(tf_goal_tansform);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));


                    geometry_msgs::msg::PoseStamped charger_goal_pose;
                    charger_goal_pose.header.frame_id = "charger_center";
                    charger_goal_pose.header.stamp = this->now();

                    charger_goal_pose.pose.position.x = -1.0 * this->get_parameter("distance_approach").get_parameter_value().get<double>();
                    charger_goal_pose.pose.orientation.w = 1.0;

                    tf_buffer_->transform(charger_goal_pose, charger_goal_pose_odom, "odom", tf2::durationFromSec(1.0));
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Error to transform target goal pose...");
                }

                publish_pose_markers(charger_goal_pose_odom);

                RCLCPP_INFO(this->get_logger(), "Approach to charging station...");
                auto Kp_rho = 0.9;
                auto Kp_alpha = 1.5;
                auto Kp_beta = 0.3;

                geometry_msgs::msg::Twist cmd_vel;
                auto rho = sqrt(pow(charger_goal_pose_odom.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                    + pow(charger_goal_pose_odom.pose.position.y - current_odom_.pose.pose.position.y, 2));
                while(rclcpp::ok() && rho > 0.01)
                {
                    auto x_diff = charger_goal_pose_odom.pose.position.x - current_odom_.pose.pose.position.x;
                    auto y_diff = charger_goal_pose_odom.pose.position.y - current_odom_.pose.pose.position.y;
                    auto theta = tf2::getYaw(current_odom_.pose.pose.orientation);
                    auto theta_goal = tf2::getYaw(charger_goal_pose_odom.pose.orientation);

                    //
                    rho = sqrt(pow(charger_goal_pose_odom.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                    + pow(charger_goal_pose_odom.pose.position.y - current_odom_.pose.pose.position.y, 2));
                    auto alpha = std::fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI;
                    auto beta = std::fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;

                    auto v = Kp_rho * rho;
                    auto w = Kp_alpha * alpha - Kp_beta * beta;

                    if(alpha > (M_PI / 2) || (alpha < -M_PI / 2))
                        v = -v;

                    cmd_vel.linear.x = std::min(v, 0.1);
                    cmd_vel.angular.z = std::min(w, 0.5);
                    pub_cmd_vel_->publish(cmd_vel);
                }

                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                pub_cmd_vel_->publish(cmd_vel);

                is_docked_n_charging_ = true;
                // TODO: Charging start command?

                RCLCPP_INFO(this->get_logger(), "Done approach to charging station...");
            }
            /*
             * Undocking
             */
            if(goal->mode == 1)  // Undocking
            {
                if(!is_docked_n_charging_)
                {
                    RCLCPP_INFO(this->get_logger(), "Robot is not on the charging...");
                    result->success = false;
                    result->message = "Robot is not on the charging.";
                    goal_handle->succeed(result);
                    return;
                }

                // TODO: Charging stop command?
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // Set the goal pose to behind of the robot
                geometry_msgs::msg::PoseStamped goal_pose;
                goal_pose.header.frame_id = "charger_center";
                goal_pose.header.stamp = this->now();

                goal_pose.pose.position.x = -2.0 * this->get_parameter("distance_approach").get_parameter_value().get<double>();
                goal_pose.pose.orientation.z = M_PI;
                goal_pose.pose.orientation.w = 0.0;

                // Convert goal_pose to odom frame
                geometry_msgs::msg::PoseStamped goal_pose_odom;
                try
                {
                    tf_buffer_->transform(goal_pose, goal_pose_odom, "odom", tf2::durationFromSec(1.0));
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN(this->get_logger(), "Error to transform target goal pose...");
                }
                publish_pose_markers(goal_pose_odom);

                RCLCPP_INFO(this->get_logger(), "Bear off from charging station...");
                auto Kp_rho = 0.9;
                // auto Kp_alpha = 1.5;
                // auto Kp_beta = 0.3;

                geometry_msgs::msg::Twist cmd_vel;
                auto rho = sqrt(pow(goal_pose_odom.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                    + pow(goal_pose_odom.pose.position.y - current_odom_.pose.pose.position.y, 2));

                while(rclcpp::ok() && rho > 0.02)
                {
                    rho = sqrt(pow(goal_pose_odom.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                    + pow(goal_pose_odom.pose.position.y - current_odom_.pose.pose.position.y, 2));

                    auto v = Kp_rho * rho;

                    cmd_vel.linear.x = -1.0 * std::min(v, 0.1);
                    cmd_vel.angular.z = 0.0;
                    pub_cmd_vel_->publish(cmd_vel);
                }

                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                pub_cmd_vel_->publish(cmd_vel);

                RCLCPP_INFO(this->get_logger(), "Rotate to forwaring to map...");

                auto diff_heading = angles::shortest_angular_distance(tf2::getYaw(goal_pose_odom.pose.orientation),
                                                                    tf2::getYaw(current_odom_.pose.pose.orientation));
                auto rotate_dir = copysign(1.0, diff_heading);

                while(rclcpp::ok() && abs(diff_heading) > 0.05)
                {
                    diff_heading = angles::shortest_angular_distance(tf2::getYaw(goal_pose_odom.pose.orientation),
                                                                    tf2::getYaw(current_odom_.pose.pose.orientation));

                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = rotate_dir * std::min(abs(Kp_rho * diff_heading), 0.3);
                    pub_cmd_vel_->publish(cmd_vel);
                }

                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                pub_cmd_vel_->publish(cmd_vel);

                is_docked_n_charging_ = false;
                RCLCPP_INFO(this->get_logger(), "Done bear off from charging station...");
            }

            if (rclcpp::ok())
            {
                result->success = true;
                result->message = "OK";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
        }

        bool find_charger_pose(geometry_msgs::msg::PoseStamped &pose)
        {
            // Find Charging Station Pattern Line
            std::vector<Line> lines;
            geometry_msgs::msg::PoseStamped target_pose;

            int16_t error_count = 0;
            bool found_pattern = false;

            while(rclcpp::ok())
            {
                line_extraction_.extractLines(lines, 0.5);
                publish_line_markers(lines);

                if(lines.size() < 2)
                {
                    RCLCPP_WARN(this->get_logger(), "Found only 1 line. Searchn again...");
                    error_count++;
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Found %d line segments...", (int)lines.size());

                    for(size_t i = 0; (i < lines.size() - 1) && !found_pattern; i++)
                    {
                        Eigen::Vector3d v1(
                                (lines[i].getStart()[0] - lines[i].getEnd()[0]), (lines[i].getStart()[1] - lines[i].getEnd()[1]),
                                0.0
                        );

                        Eigen::Vector3d v2(
                                (lines[i+1].getStart()[0] - lines[i+1].getEnd()[0]), (lines[i+1].getStart()[1] - lines[i+1].getEnd()[1]),
                                0.0
                        );

                        auto dot_prod = v1.dot(v2);
                        auto mag_v1 = pow(v1.dot(v1), 0.5);
                        auto mag_v2 = pow(v2.dot(v2), 0.5);
                        auto angle = M_PI - acos(dot_prod / mag_v2 / mag_v1);

                        // Reference: 145(deg) := 2.5307(rad)
                        if(abs(2.5307 - angle) < 0.0873)
                        {
                            found_pattern = true;
                            RCLCPP_INFO(this->get_logger(), "Found matched charging pattern: %f", angle);

                            Eigen::Hyperplane<double, 2> l1, l2;
                            l1 = Eigen::Hyperplane<double, 2>::Through(
                                    Eigen::Vector2d(lines[i].getStart()[0], lines[i].getStart()[1]), Eigen::Vector2d(lines[i].getEnd()[0], lines[i].getEnd()[1]));
                            l2 = Eigen::Hyperplane<double, 2>::Through(
                                    Eigen::Vector2d(lines[i+1].getStart()[0], lines[i+1].getStart()[1]), Eigen::Vector2d(lines[i+1].getEnd()[0], lines[i+1].getEnd()[1]));

                            Eigen::Hyperplane<double, 2>::VectorType icp = l1.intersection(l2);
                            RCLCPP_INFO(this->get_logger(), "ICP x: %f, y: %f", icp.x(), icp.y());

                            auto heading = atan2((lines[i+1].getEnd()[1] - lines[i].getStart()[1]), (lines[i+1].getEnd()[0] - lines[i].getStart()[0]));
                            RCLCPP_INFO(this->get_logger(), "Heading: %f", heading - (M_PI/2));

                            // Save the goal pose
                            target_pose.header.frame_id = "laser_link";
                            target_pose.header.stamp = this->now();

                            target_pose.pose.position.x = icp.x();
                            target_pose.pose.position.y = icp.y();
                            target_pose.pose.position.z = 0.0;

                            tf2::Quaternion q;
                            q.setRPY(0, 0, heading - (M_PI/2));

                            target_pose.pose.orientation.x = q[0];
                            target_pose.pose.orientation.y = q[1];
                            target_pose.pose.orientation.z = q[2];
                            target_pose.pose.orientation.w = q[3];
                        }
                    }

                    if(found_pattern)
                        break;
                }

                if(error_count > 5)
                {
                    return false;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            pose = target_pose;
            return true;
        }


        void cacheData(const sensor_msgs::msg::LaserScan & msg)
        {
            std::vector<double> bearings, cos_bearings, sin_bearings;
            std::vector<unsigned int> indices;
            const std::size_t num_measurements = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment);

            for (std::size_t i = 0; i < num_measurements; ++i)
            {
                const double b = msg.angle_min + i * msg.angle_increment;
                bearings.push_back(b);
                cos_bearings.push_back(cos(b));
                sin_bearings.push_back(sin(b));
                indices.push_back(i);
            }

            line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
            RCLCPP_INFO(this->get_logger(), "Data has been cached.");
        }

        void scan_callback(const sensor_msgs::msg::LaserScan & msg)
        {
            if(!data_cached_)
            {
                cacheData(msg);
                data_cached_ = true;
            }

            std::vector<double> scan_data(msg.ranges.begin(), msg.ranges.end());
            line_extraction_.setRangeData(scan_data);
        }

        void odom_callback(const nav_msgs::msg::Odometry & msg)
        {
            current_odom_ = msg;
        }

        void publish_line_markers(const std::vector<Line> &lines)
        {
            visualization_msgs::msg::Marker line_marker;

            line_marker.ns = "lines";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_marker.scale.x = 0.01;
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;

            for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
            {
                geometry_msgs::msg::Point p_start;
                p_start.x = cit->getStart()[0];
                p_start.y = cit->getStart()[1];
                p_start.z = 0;
                line_marker.points.push_back(p_start);

                geometry_msgs::msg::Point p_end;
                p_end.x = cit->getEnd()[0];
                p_end.y = cit->getEnd()[1];
                p_end.z = 0;
                line_marker.points.push_back(p_end);
            }

            line_marker.header.frame_id = "laser_link";
            line_marker.header.stamp = this->now();

            pub_vis_lines_->publish(line_marker);
        }

        void publish_pose_markers(const geometry_msgs::msg::PoseStamped pose_stamped)
        {
            visualization_msgs::msg::Marker pose_marker;

            pose_marker.ns = "goal_pose";
            pose_marker.id = 1;
            pose_marker.type = visualization_msgs::msg::Marker::ARROW;

            pose_marker.pose = pose_stamped.pose;

            pose_marker.scale.x = 0.2;
            pose_marker.scale.y = 0.05;
            pose_marker.scale.z = 0.05;

            pose_marker.color.r = 0.0;
            pose_marker.color.g = 0.0;
            pose_marker.color.b = 1.0;
            pose_marker.color.a = 1.0;

            pose_marker.header.frame_id = pose_stamped.header.frame_id;
            pose_marker.header.stamp = this->now();

            pub_vis_lines_->publish(pose_marker);
        }

    private:
        LineExtraction line_extraction_;
        bool data_cached_;
        bool is_docked_n_charging_;

        nav_msgs::msg::Odometry current_odom_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_br_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

        rclcpp_action::Server<former_interfaces::action::Docking>::SharedPtr as_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vis_lines_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FormerAutoDockingNode>());
    rclcpp::shutdown();
    return 0;
}