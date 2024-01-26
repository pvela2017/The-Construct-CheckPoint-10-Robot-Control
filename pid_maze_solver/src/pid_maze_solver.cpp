#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <cmath>

class MazeSolver : public rclcpp::Node {
public:
    MazeSolver() : Node("turn_controller") 
    {
        // Callbacks
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;

        callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = callback_group2_;

        callback_group3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options3;
        options3.callback_group = callback_group3_;

        // Publisher for velocity commands
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Subscriber for odometry data
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&MazeSolver::odometryCallback, this, std::placeholders::_1), options1);
        // Timers
        dt_ = 25;
        timer_distance_controller_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&MazeSolver::timer_distance_controller_callback, this), callback_group_);
        timer_turn_controller_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&MazeSolver::timer_turn_controller_callback, this), callback_group2_);
        timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MazeSolver::timer_publisher_callback, this), callback_group3_);

        timer_distance_controller_->cancel();
        timer_turn_controller_->cancel();

        // PID gains
        kp_yaw_ = 0.10;  // Proportional gain
        ki_yaw_ = 0.10;  // Integral gain
        kd_yaw_ = 0.50; // Derivative gain

        kp_distance_ = 1.5;  // Proportional gain
        ki_distance_ = 0.15;  // Integral gain
        kd_distance_ = 0.20; // Derivative gain

        // limits
        goal_tolerance_yaw_ = 0.017;
        upper_limit_yaw_ = 3.14;
        lower_limit_yaw_ = -3.14;
        windup_limit_yaw_ = 1;

        goal_tolerance_distance_ = 0.02;
        upper_limit_distance_ = 0.8;
        lower_limit_distance_ = -0.8;
        windup_limit_distance_ = 1;

        // Hardcoded goals real
        goals_ = {{1.60, -1.37}, 
                  {0.35, -1.37},
                  {0.25, 1.37},
                  {0.35, 1.37},
                  {0.25, -1.47},
                  {0.35, -1.37},
                  {0.30, 1.37},
                  {0.25, -1.37},
                  {0.35, -1.47},
                  {0.60, 1.37},
                  {0.40, -1.37},
                  {0.35, 1.37},
                  {0.20, 0.00}};

        // ========== SIMULATION ========== 
        // PID gains simu
        /*kp_yaw_ = 1.50;  // Proportional gain
        ki_yaw_ = 0.20;  // Integral gain
        kd_yaw_ = 0.50; // Derivative gain

        kp_distance_ = 1.5;  // Proportional gain
        ki_distance_ = 0.15;  // Integral gain
        kd_distance_ = 0.20; // Derivative gain

        // Hardcoded goals simu
        goals_ = {{0.30, -0.65},
                  {0.20, -0.65}, 
                  {1.2, 1.37},
                  {0.50, 1.47},
                  {0.55, -1.47},
                  {0.40, 1.47},
                  {0.50, -1.47},
                  {0.48, 1.47},
                  {0.95, 1.47},
                  {0.48, 1.47},
                  {0.35, -1.47},
                  {0.45, -1.47},
                  {0.35, 1.47},
                  {0.85, 0.00}};*/

        // ========== SIMULATION ========== 

        current_goal_index_ = 0;

        wait_for_odom_flag_ = true;
    }

    void timer_distance_controller_callback()
    {
        // Calculate error
        double error_distance = (initial_distance_ + goals_[current_goal_index_].first) - total_distance_;

        if (std::abs(error_distance) < goal_tolerance_distance_) 
        {
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.linear.x = 0.0;
            error_distance = 0;
            integral_distance_ = 0;
            initial_yaw_ = current_yaw_;
            rclcpp::sleep_for(std::chrono::seconds(1));
            timer_distance_controller_->cancel();
            timer_turn_controller_->reset();
        }

        // Update integral and derivative terms
        integral_distance_ += error_distance;
        double derivative_distance = (error_distance - prev_error_distance_);

        // Apply windup limit to limit the size of the integral term
        if (integral_distance_ > fabsf(windup_limit_distance_))
            integral_distance_ = fabsf(windup_limit_distance_);
        if (integral_distance_ < -fabsf(windup_limit_distance_))
            integral_distance_ = -fabsf(windup_limit_distance_);
        

        // PID control signal
        double control_signal_distance = kp_distance_ * error_distance + ki_distance_ * integral_distance_ + kd_distance_ * derivative_distance;

        // Apply saturation limits
        if (control_signal_distance > upper_limit_distance_)
            control_signal_distance = upper_limit_distance_;
        else if (control_signal_distance < lower_limit_distance_)
            control_signal_distance = lower_limit_distance_;

        // Create velocity command
        cmd_vel_.linear.z = 0.0;
        cmd_vel_.linear.x = control_signal_distance;

        // Update previous error for the next iteration
        prev_error_distance_ = error_distance;
    }

    void timer_turn_controller_callback()
    {
        // Calculate error
        double error_yaw = (initial_yaw_ + goals_[current_goal_index_].second) - current_yaw_;
        error_yaw = wrapToPi(error_yaw);

        if (std::abs(error_yaw) < goal_tolerance_yaw_) 
        {
            cmd_vel_.linear.z = 0.0;
            cmd_vel_.linear.x = 0.0;
            error_yaw = 0;
            integral_yaw_ = 0;
            initial_distance_ = total_distance_;
            rclcpp::sleep_for(std::chrono::seconds(1));
            timer_turn_controller_->cancel();
            // Move to the next goal
            current_goal_index_++;
            timer_distance_controller_->reset();
        }

        // Update integral and derivative terms
        integral_yaw_ += error_yaw;
        double derivative_yaw_ = (error_yaw - prev_error_yaw_);

        // Apply windup limit to limit the size of the integral term
        if (integral_yaw_ > fabsf(windup_limit_yaw_))
            integral_yaw_ = fabsf(windup_limit_yaw_);
        if (integral_yaw_ < -fabsf(windup_limit_yaw_))
            integral_yaw_ = -fabsf(windup_limit_yaw_);
        

        // PID control signal
        double control_signal_yaw_ = kp_yaw_ * error_yaw + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw_;

        // Apply saturation limits
        if (control_signal_yaw_ > upper_limit_yaw_)
            control_signal_yaw_ = upper_limit_yaw_;
        else if (control_signal_yaw_ < lower_limit_yaw_)
            control_signal_yaw_ = lower_limit_yaw_;

        // Create velocity command
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = control_signal_yaw_;

        // Stop the robot if all goals are reached
        if (current_goal_index_ >= goals_.size()) 
        {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            timer_turn_controller_->cancel();
            timer_distance_controller_->cancel();
        }

        // Update previous error for the next iteration
        prev_error_yaw_ = error_yaw;
    }

    void timer_publisher_callback()
    {
        velocity_pub_->publish(cmd_vel_);
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_distance_controller_;
    rclcpp::TimerBase::SharedPtr timer_turn_controller_;
    rclcpp::TimerBase::SharedPtr timer_publisher_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;
    rclcpp::CallbackGroup::SharedPtr callback_group3_;

    geometry_msgs::msg::Twist cmd_vel_;

    std::vector<std::pair<double,double>> goals_;
    unsigned current_goal_index_;
    double current_yaw_;
    double initial_yaw_;
    double initial_distance_;

    double kp_yaw_;
    double ki_yaw_;
    double kd_yaw_;
    double kp_distance_;
    double ki_distance_;
    double kd_distance_;

    double upper_limit_yaw_;
    double lower_limit_yaw_;
    double windup_limit_yaw_;
    double goal_tolerance_yaw_;
    double upper_limit_distance_;
    double lower_limit_distance_;
    double windup_limit_distance_;
    double goal_tolerance_distance_;

    double integral_yaw_ = 0.0;
    double prev_error_yaw_ = 0.0;
    double integral_distance_ = 0.0;
    double prev_error_distance_ = 0.0;

    double total_distance_ = 0.0;

    int dt_;

    bool wait_for_odom_flag_;

    geometry_msgs::msg::Pose previous_pose_;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, current_yaw_);

        // Assuming that the pose information is present in the odometry message
        geometry_msgs::msg::Pose current_pose = msg->pose.pose;

        // Calculate the change in pose since the last update
        double delta_x = current_pose.position.x - previous_pose_.position.x;
        double delta_y = current_pose.position.y - previous_pose_.position.y;

        // Update the total distance traveled 
        total_distance_ += std::sqrt(delta_x * delta_x + delta_y * delta_y);

        // Update the previous pose for the next iteration
        previous_pose_ = current_pose;

        if (wait_for_odom_flag_)
        {
            wait_for_odom_flag_ = false;
            initial_yaw_ = current_yaw_;
            initial_distance_ = total_distance_;
            timer_distance_controller_->reset();
        }
    }

    double wrapToPi(double angle) 
    {
        while (angle > M_PI) 
        {
            angle -= 2.0 * M_PI;
        }

        while (angle < -M_PI) 
        {
            angle += 2.0 * M_PI;
        }

        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<MazeSolver> node = std::make_shared<MazeSolver>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}