#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// Controller node:
// 1. Subscribe to joystick for thrust and steering command
// 2. Calculate the differential command for left and right tracks (current or speed)
// 3. Send to gazebo topic for simulation
class Controller : public rclcpp::Node {
    public:
	Controller() : Node("controller_node") {
	    joystick_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
	"joy", 100, 
	std::bind(&Controller::joystickCallback, this, std::placeholders::_1));
	    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/simple_tracked/cmd_vel", 100);
	    // cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/simple_tracked/link/right_track/track_cmd_vel", 100);
	    timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_callback_duration), std::bind(&Controller::timerCallback, this));
	}

	void joystickCallback(const sensor_msgs::msg::Joy & msg) {
	    this->thrust = (double) msg.axes[1];
	    this->steer = (double) msg.axes[3];
	    
	    if (this->thrust > 0.01 or this->thrust < -0.01) {
	        thrust_pressed = true;
	    } else {
	        thrust_pressed = false;
	    }
	    
	    if (this->steer > 0.01 or this->steer < -0.01) {
	        steer_pressed = true;
	    } else {
	        steer_pressed = false;
	    }
	}
	
	void timerCallback() {
	    if (thrust_pressed) {
	        this->cmd_vel.linear.x = this->cmd_vel.linear.x + thrust * timer_callback_duration / 1000;
	    } else {
	        this->cmd_vel.linear.x = 0.0;
	    }
	    
	    if (steer_pressed) {
	        this->cmd_vel.angular.z = this->cmd_vel.angular.z + steer * timer_callback_duration / 1000;
	    } else {
	        this->cmd_vel.angular.z = 0.0;
	    }
            
	    cmd_vel_pub_->publish(this->cmd_vel);
	    // RCLCPP_INFO(get_logger(), "Publishing command: linear %f, angular %f", this->cmd_vel.linear.x, this->cmd_vel.angular.z);
	}

        geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
	rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
	double thrust = 0.0; // [-1, 1]
	double steer = 0.0;  // [-1, 1]
	rclcpp::TimerBase::SharedPtr timer_;
	int timer_callback_duration = 10;
	bool thrust_pressed = false;
	bool steer_pressed = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto controller_node = std::make_shared<Controller>();
    
    RCLCPP_INFO(controller_node->get_logger(), "Controller node starting");

    rclcpp::spin(controller_node);
}

