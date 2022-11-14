#include <ros/ros.h>
#include <cmath>
#include <queue>
#include <boost/thread/thread.hpp>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "angles/angles.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class MoveToTarget
{
    public:
        uint8_t mode;
        double target_val;
};

class MoveToNode
{
    public:
        MoveToNode()
        : tf_listener_(tf_buffer_), pnh_("~")
        {
            is_moving_ = false;

            pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
            pnh_.param<std::string>("base_footprint_frame", base_footprint_frame_, "base_footprint");

            sub_odom_ = nh_.subscribe("base_controller/odom", 100, &MoveToNode::callback_odometry, this);
            pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

            sub_target_dist_ = nh_.subscribe("move_to_target_dist", 10, &MoveToNode::callback_move_to_target_dist, this);
            sub_target_rotate_ = nh_.subscribe("move_to_target_rotate", 10, &MoveToNode::callback_move_to_target_rotate, this);

            move_queue_ = {};
            move_thread_ = boost::make_shared<boost::thread>(&MoveToNode::move_to_function, this);
            ROS_INFO("[%s] initialized successfully.", ros::this_node::getName().c_str());
        }
        ~MoveToNode() {}

    private:
        void move_to_function()
        {
            while(ros::ok())
            {
                if(move_queue_.empty())
                {
                    ros::Duration(0.01).sleep();
                    continue;
                }

                ROS_INFO("[%s] start move...", ros::this_node::getName().c_str());

                is_moving_ = true;
                MoveToTarget target = move_queue_.front();

                if(target.mode == 0) // Linear Move
                {
                    geometry_msgs::PoseStamped local_target_pose;
                    local_target_pose.header.frame_id = base_footprint_frame_;
                    local_target_pose.pose.position.x = target.target_val;
                    local_target_pose.pose.orientation.z = 1.0;

                    geometry_msgs::PoseStamped odom_target_pose;
                    try
                    {
                        tf_buffer_.transform(local_target_pose, odom_target_pose, odom_frame_, ros::Duration(0.01));
                    }
                    catch (tf2::TransformException &ex)
                    {
                        ROS_WARN("[%s] Error to transform target pose...", ros::this_node::getName().c_str());
                        move_queue_.pop();
                        is_moving_ = false;
                        continue;
                    }

                    double dist_to_target = sqrt(pow(odom_target_pose.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                            + pow(odom_target_pose.pose.position.y - current_odom_.pose.pose.position.y, 2));

                    ROS_INFO("[%s] remain distance... %f", ros::this_node::getName().c_str(), dist_to_target);

                    geometry_msgs::Twist cmd_vel;
                    while(ros::ok() && abs(dist_to_target) > 0.01)
                    {
                        cmd_vel.linear.x = std::min(0.2, dist_to_target * 0.9);
                        pub_cmd_vel_.publish(cmd_vel);

                        dist_to_target = sqrt(pow(odom_target_pose.pose.position.x - current_odom_.pose.pose.position.x, 2)
                                            + pow(odom_target_pose.pose.position.y - current_odom_.pose.pose.position.y, 2));
                        ROS_INFO("[%s] remain distance... %f", ros::this_node::getName().c_str(), dist_to_target);
                    }

                    cmd_vel.linear.x = 0.0;
                    pub_cmd_vel_.publish(cmd_vel);
                }
                else if(target.mode == 1) // Rotate Move
                {
                    double current_theta = tf2::getYaw(current_odom_.pose.pose.orientation);
                    double goal_theta = angles::normalize_angle(current_theta + target.target_val);

                    double error_theta = angles::shortest_angular_distance(current_theta, goal_theta);

                    ROS_INFO("[%s] remain theta... %f", ros::this_node::getName().c_str(), error_theta);

                    geometry_msgs::Twist cmd_vel;
                    while(ros::ok() && abs(error_theta) > 0.025)
                    {

                        cmd_vel.angular.z = copysign(1.0, error_theta) * std::min(0.3, error_theta * 0.9);
                        pub_cmd_vel_.publish(cmd_vel);

                        current_theta = tf2::getYaw(current_odom_.pose.pose.orientation);
                        error_theta = angles::shortest_angular_distance(current_theta, goal_theta);
                        ROS_INFO("[%s] remain theta... %f", ros::this_node::getName().c_str(), error_theta);
                    }

                    cmd_vel.angular.z = 0.0;
                    pub_cmd_vel_.publish(cmd_vel);
                }

                move_queue_.pop();
                is_moving_ = false;
            }

        }

        void callback_move_to_target_dist(const std_msgs::Float64ConstPtr &msg)
        {
            if(is_moving_)
                return;

            ROS_INFO("[%s] request to move distance %f ", ros::this_node::getName().c_str(), msg->data);

            MoveToTarget target;
            target.mode = 0;
            target.target_val = msg->data;

            move_queue_.push(target);
        }

        void callback_move_to_target_rotate(const std_msgs::Float64ConstPtr &msg)
        {
            if(is_moving_)
                return;

            MoveToTarget target;
            target.mode = 1;
            target.target_val = msg->data;

            move_queue_.push(target);
        }

        void callback_odometry(const nav_msgs::OdometryConstPtr &msg)
        {
            current_odom_ = *msg;
        }

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber sub_odom_;
        ros::Publisher pub_cmd_vel_;

        nav_msgs::Odometry current_odom_;

        std::string base_footprint_frame_;
        std::string odom_frame_;

        boost::shared_ptr<boost::thread> move_thread_;
        std::queue<MoveToTarget> move_queue_;

        bool is_moving_;
        ros::Subscriber sub_target_dist_;
        ros::Subscriber sub_target_rotate_;

        tf2_ros::StaticTransformBroadcaster br_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_to_node");
    MoveToNode m;
    ros::spin();

    return 0;
}