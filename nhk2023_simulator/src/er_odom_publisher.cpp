// This node is used to publish the odometry of the robot in simulation
// the odometry is calculated from ignition gazebo's pose information

#include <memory>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Matrix3x3.h>

class er_odom_publisher : public rclcpp::Node{
    public:
        er_odom_publisher() : Node("er_odom_publisher"){
            publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

            subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
                "er/pose", rclcpp::QoS(10),
                std::bind(&er_odom_publisher::PoseCallback, this,
                    std::placeholders::_1));

            reset_ = this->create_subscription<std_msgs::msg::Bool>(
                "odom_reset", rclcpp::QoS(10),
                std::bind(&er_odom_publisher::ResetCallback, this,
                          std::placeholders::_1));

            this->declare_parameter<std::string>("base_frame_id", "er_base_link");
            this->declare_parameter<int>("loop_rate", 100);
            this->get_parameter<std::string>("base_frame_id", base_frame_id);
            this->get_parameter<int>("loop_rate", loop_rate);

            odom_quat.x = 0;
            odom_quat.y = 0;
            odom_quat.z = 0;
            odom_quat.w = 1;

            rclcpp::WallRate loop_rate(this->loop_rate);

            while (rclcpp::ok()){
                robot_odom_handler();
                loop_rate.sleep();
            }
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_;
        int loop_rate = 100;
        std::string base_frame_id = "base_link";

        //previous pose
        double prev_position[3] = {};
        double prev_rot[3] = {};

        //current pose
        double position[3] = {};
        double rot[3] = {};
        geometry_msgs::msg::Quaternion odom_quat;

        //current velocity
        double dx, dy, dz = 0;
        double droll, dpitch, dyaw = 0;

        void PoseCallback(const geometry_msgs::msg::Pose& msg){
            position[0] = msg.position.x;
            position[1] = msg.position.y;
            position[2] = msg.position.z;
            odom_quat = msg.orientation;
        }

        void robot_odom_handler() {
            auto message = nav_msgs::msg::Odometry();
            tf2::Quaternion tf_odom_quat(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
            tf2::Matrix3x3(tf_odom_quat).getRPY(rot[0], rot[1], rot[2]);
            // twistを算出
            dx = prev_position[0] - position[0];
            dy = prev_position[1] - position[1];
            dz = prev_position[2] - position[2];
            memcpy(prev_position, position, sizeof(position));

            dyaw = prev_rot[0] - rot[0];
            droll = prev_rot[1] - rot[1];
            dpitch = prev_rot[2] - rot[2];
            memcpy(prev_rot, rot, sizeof(rot));

            // odometryに値を入れる
            message.twist.twist.linear.x = dx*loop_rate;
            message.twist.twist.linear.y = dy*loop_rate;
            message.twist.twist.linear.z = dz*loop_rate;
            message.twist.twist.angular.x = droll*loop_rate;
            message.twist.twist.angular.y = dpitch*loop_rate;
            message.twist.twist.angular.z = dyaw*loop_rate;
            message.header.frame_id = "odom";
            message.child_frame_id = base_frame_id;
            message.header.stamp = this->now();
            message.pose.pose.position.x = position[0];
            message.pose.pose.position.y = position[1];
            message.pose.pose.position.z = position[2];
            message.pose.pose.orientation = odom_quat;

            // publish odometry topic
            publisher_->publish(message);
        }

        void ResetCallback(const std_msgs::msg::Bool& msg) {
            // make all values zero when reset is true
            if(msg.data){
                position[0] = 0;
                position[1] = 0;
                position[2] = 0;
                rot[0] = 0;
                rot[1] = 0;
                rot[2] = 0;
                odom_quat.x = 0;
                odom_quat.y = 0;
                odom_quat.z = 0;
                odom_quat.w = 1;
                RCLCPP_INFO(this->get_logger(), "odom reset");
            }
            else{
                RCLCPP_INFO(this->get_logger(), "Reset signal is false");
            }
        }
};

int main(int argc, char * argv[]){
     rclcpp::init(argc, argv);
     rclcpp::spin(std::make_shared<er_odom_publisher>());
     rclcpp::shutdown();
     return 0;
}