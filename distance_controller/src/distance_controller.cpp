#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"


class DistanceController : public rclcpp::Node {
public:
    DistanceController() : Node("distance_controller") 
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
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", 10, std::bind(&DistanceController::odometryCallback, this, std::placeholders::_1), options1);
        // Timers
        dt_ = 25;
        timer_controller_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&DistanceController::timer_controller_callback, this), callback_group_);
        timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(dt_), std::bind(&DistanceController::timer_publisher_callback, this), callback_group2_);

        timer_controller_->cancel();

        // PID gains
        kp_ = 1.5;  // Proportional gain
        ki_ = 0.15;  // Integral gain
        kd_ = 0.20; // Derivative gain

        // limits
        goal_tolerance_ = 0.02;
        upper_limit_ = 0.8;
        lower_limit_ = -0.8;
        windup_limit_ = 1;

        // Hardcoded distance goals
        distance_goals_ = {1.0, 2.0, 3.0};
        current_goal_index_ = 0;
        
        wait_for_odom_flag_ = true;
    }

    void timer_controller_callback()
    {
        // Calculate error
        double error = (initial_distance_ + distance_goals_[current_goal_index_]) - total_distance_;

        if (std::abs(error) < goal_tolerance_) 
        {
            error = 0;
            integral_ = 0;
            cmd_vel_.linear.x = 0.0;
            rclcpp::sleep_for(std::chrono::seconds(2));
            // Move to the next goal
            initial_distance_ = total_distance_;
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
        cmd_vel_.linear.x = control_signal;

        // Stop the robot if all goals are reached
        if (current_goal_index_ >= distance_goals_.size()) 
        {
            cmd_vel_.linear.x = 0.0;
            timer_controller_->cancel();
            RCLCPP_INFO(this->get_logger(), "Completed");
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

    std::vector<double> distance_goals_;
    unsigned current_goal_index_;
    double initial_distance_;

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

    double total_distance_ = 0.0;

    bool wait_for_odom_flag_;

    geometry_msgs::msg::Pose previous_pose_;
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
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
            initial_distance_ = total_distance_;
            timer_controller_->reset();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<DistanceController> node = std::make_shared<DistanceController>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}