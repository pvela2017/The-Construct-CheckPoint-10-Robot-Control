#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

class TurnController : public rclcpp::Node {
public:
    TurnController() : Node("turn_controller") 
    {
        // Callbacks
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options1;
        options1.callback_group = callback_group_;

        callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options2;
        options2.callback_group = callback_group2_;

        // Publisher for velocity commands
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Subscriber for odometry data
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&TurnController::odometryCallback, this, std::placeholders::_1), options1);
        // Timers
        dt_ = 25;
        timer_controller_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&TurnController::timer_controller_callback, this), callback_group_);
        timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&TurnController::timer_publisher_callback, this), callback_group2_);

        timer_controller_->cancel();

        // PID gains
        kp_ = 1.50;  // Proportional gain
        ki_ = 0.20;  // Integral gain
        kd_ = 0.50; // Derivative gain

        // limits
        goal_tolerance_ = 0.034;
        upper_limit_ = 3.14;
        lower_limit_ = -3.14;
        windup_limit_ = 1;

        // Hardcoded distance goals
        yaw_goals_ = {-1.2, 0.8, 0.8};
        current_goal_index_ = 0;

        wait_for_odom_flag_ = true;
    }

    void timer_controller_callback()
    {
        // Calculate error
        double error = (initial_yaw_ + yaw_goals_[current_goal_index_]) - current_yaw_;
        error = wrapToPi(error);

        if (std::abs(error) < goal_tolerance_) 
        {
            error = 0;
            integral_ = 0;
            cmd_vel_.angular.z = 0.0;
            rclcpp::sleep_for(std::chrono::seconds(2));
            // Move to the next goal
            initial_yaw_ = current_yaw_;
            current_goal_index_++;
        }

        // Update integral and derivative terms
        integral_ += error;
        double derivative = (error - prev_error_);

        // Apply windup limit to limit the size of the integral term
        if (integral_ > fabsf(windup_limit_))
            integral_ = fabsf(windup_limit_);
        if (integral_ < -fabsf(windup_limit_))
            integral_ = -fabsf(windup_limit_);
        

        // PID control signal
        double control_signal = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // Apply saturation limits
        if (control_signal > upper_limit_)
            control_signal = upper_limit_;
        else if (control_signal < lower_limit_)
            control_signal = lower_limit_;

        // Create velocity command
        cmd_vel_.angular.z = control_signal;

        // Stop the robot if all goals are reached
        if (current_goal_index_ >= yaw_goals_.size()) 
        {
            cmd_vel_.angular.z = 0.0;
            timer_controller_->cancel();
        }

        // Update previous error for the next iteration
        prev_error_ = error;
    }

    void timer_publisher_callback()
    {
        velocity_pub_->publish(cmd_vel_);
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::TimerBase::SharedPtr timer_controller_;
    rclcpp::TimerBase::SharedPtr timer_publisher_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::CallbackGroup::SharedPtr callback_group2_;

    geometry_msgs::msg::Twist cmd_vel_;

    std::vector<double> yaw_goals_;
    unsigned current_goal_index_;
    double current_yaw_;
    double initial_yaw_;

    double kp_;
    double ki_;
    double kd_;

    double upper_limit_;
    double lower_limit_;
    double windup_limit_;
    double goal_tolerance_;

    double integral_ = 0.0;
    double prev_error_ = 0.0;

    int dt_;

    bool wait_for_odom_flag_;

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg->pose.pose.orientation, quaternion);
        double roll, pitch;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, current_yaw_);

        if (wait_for_odom_flag_)
        {
            wait_for_odom_flag_ = false;
            initial_yaw_ = current_yaw_;
            timer_controller_->reset();
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
    std::shared_ptr<TurnController> node = std::make_shared<TurnController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}