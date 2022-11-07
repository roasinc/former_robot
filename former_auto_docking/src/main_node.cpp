#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/utils.h"

#include "former_interfaces/action/docking.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

            line_extraction_.setBearingVariance(1e-5 * 1e-5);
            line_extraction_.setRangeVariance(0.012 * 0.012);
            line_extraction_.setLeastSqAngleThresh(0.0001);
            line_extraction_.setLeastSqRadiusThresh(0.0001);
            line_extraction_.setMaxLineGap(0.5);
            line_extraction_.setMinLineLength(0.1);
            line_extraction_.setMinRange(0.5);
            line_extraction_.setMaxRange(250.0);
            line_extraction_.setMinSplitDist(0.03);
            line_extraction_.setOutlierDist(0.06);
            line_extraction_.setMinLinePoints(10);

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
            RCLCPP_INFO(this->get_logger(), "Executing goal");

            auto feedback = std::make_shared<former_interfaces::action::Docking::Feedback>();
            auto result = std::make_shared<former_interfaces::action::Docking::Result>();

            // Find Charging Station Pattern Line
            std::vector<Line> lines;
            geometry_msgs::msg::PoseStamped charger_center_pose;

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
                            charger_center_pose.header.frame_id = "laser_link";
                            charger_center_pose.header.stamp = this->now();

                            charger_center_pose.pose.position.x = icp.x();
                            charger_center_pose.pose.position.y = icp.y();
                            charger_center_pose.pose.position.z = 0.0;

                            tf2::Quaternion q;
                            q.setRPY(0, 0, heading - (M_PI/2));

                            charger_center_pose.pose.orientation.x = q[0];
                            charger_center_pose.pose.orientation.y = q[1];
                            charger_center_pose.pose.orientation.z = q[2];
                            charger_center_pose.pose.orientation.w = q[3];
                        }
                    }

                    if(found_pattern)
                        break;
                }

                if(error_count > 5)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to find the charging station...");

                    result->success = false;
                    result->message = "Failed to find the charging station...";
                    goal_handle->succeed(result);

                    RCLCPP_INFO(this->get_logger(), "Goal failed");
                    return;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
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

                charger_goal_pose.pose.position.x = -0.3;
                charger_goal_pose.pose.orientation.w = 1.0;

                tf_buffer_->transform(charger_goal_pose, charger_goal_pose_odom, "odom", tf2::durationFromSec(1.0));

            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "Error to transform target goal pose...");
            }

            publish_pose_markers(charger_goal_pose_odom);

            // Move to charger_goal_pose_odom









            if (rclcpp::ok())
            {
                result->success = true;
                result->message = "OK";
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            }
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

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_br_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

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