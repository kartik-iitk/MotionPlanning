#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include <iostream>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

USING_NAMESPACE_ACADO

double current_x = 0.0, target_x =0, next_x = 0.0;
double current_y = 0.0, target_y = 0, next_y = 0.0;
double current_theta = 0.0, target_theta = 0.0, next_theta = 0.0;
double current_vx = 0.0, next_vx = 0.0;
double current_vy = 0.0, next_vy = 0.0;
double current_omega = 0.0, next_omega = 0.0;
double idx = 0.0;
double path_size = 0.0;

// Differential States and Controls
DifferentialState x, y, theta, vx, vy, omega;
Control ax, ay, alpha;
TIME t;

// Parameters for robot model
double R = 0.05;
double d = 0.5;

DifferentialEquation f;

class MyRobotNode : public rclcpp::Node {
public:
    MyRobotNode() : Node("my_robot_node") {
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/self_cmd_vel", 10);
        self_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/self_position", 10, std::bind(&MyRobotNode::mpc_callback, this, std::placeholders::_1));
        target_subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/target_pos", 10, std::bind(&MyRobotNode::update_target_callback, this, std::placeholders::_1));

	f << dot(x) == vx;
	f << dot(y) == vy;
	f << dot(theta) == omega;
	f << dot(vx) == ax;
	f << dot(vy) == ay;
	f << dot(omega) == alpha;
    }

    // Callback for running the MPC
    void mpc_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
	current_x = msg->data[0];
	current_y = msg->data[1];
	current_theta = msg->data[2];
	
	current_vx = next_vx;
	current_vy = next_vy;
	current_omega = next_omega;

	// (start_time, end_time, steps)
        OCP ocp(0.0, 2.0, 20);

        double Q_pos = 10.0; // Loss function weights
        double Q_theta = 1.0e5;

        ocp.minimizeMayerTerm( Q_pos*(x - target_x)*(x - target_x) + Q_pos*(y - target_y)*(y - target_y) + Q_theta*(theta - target_theta)*(theta - target_theta)); // Loss function

        ocp.subjectTo(f);

        // State and control constraints
        ocp.subjectTo(-7.0 <= vx <= 7.0);
        ocp.subjectTo(-7.0 <= vy <= 7.0);
        ocp.subjectTo(-4.0 <= omega <= 4.0);
        ocp.subjectTo(-6.0 <= ax <= 6.0);
        ocp.subjectTo(-6.0 <= ay <= 6.0);
        ocp.subjectTo(-5.0 <= alpha <= 5.0);

        // Initial conditions
        ocp.subjectTo(AT_START, x == current_x);
        ocp.subjectTo(AT_START, y == current_y);
        ocp.subjectTo(AT_START, theta == current_theta);
        ocp.subjectTo(AT_START, vx == current_vx);
        ocp.subjectTo(AT_START, vy == current_vy);
        ocp.subjectTo(AT_START, omega == current_omega);
	
	std::cout << "Current State -> X: " << current_x << ", Y: " << current_y
                  << ", Theta: " << current_theta << ", Vx: " << current_vx
                  << ", Vy: " << current_vy << ", Omega: " << current_omega << std::endl;

        // Final state constraints towards target (moving only in x-direction)
        // ocp.subjectTo(AT_END, x == target_x);   // Target position in x
        // ocp.subjectTo(AT_END, y == target_y);       // Stop at the target
        // ocp.subjectTo(AT_END, theta == target_theta);
	// ocp.subjectTo(AT_END, vx == 0);
        // ocp.subjectTo(AT_END, vy == 0);
        // ocp.subjectTo(AT_END, omega == 0);
	
	std::cout << "Target State -> X: " << target_x << ", Y: " << target_y
                  << ", Theta: " << target_theta << std::endl;

	// Solver setup
        OptimizationAlgorithm algorithm(ocp);
        algorithm.set(PRINTLEVEL, LOW);
	if (algorithm.solve() == SUCCESSFUL_RETURN) {
		std::cout << "Algorithm successfully solved" << std::endl;
	} else {
		std::cout << "Algorithm not solved" << std::endl;
		return;
	}

	// Extract solution
        VariablesGrid states;
        algorithm.getDifferentialStates(states);

	// Update the state variables with the new state from the solution
        DVector next_state = states.getVector(1);  // Get state at next time step
        next_x = next_state(0);
        next_y = next_state(1);
        next_theta = next_state(2);
        next_vx = next_state(3);
        next_vy = next_state(4);
	next_omega = next_state(5);

	// Log the state
        std::cout << "Next State -> X: " << next_x << ", Y: " << next_y
                  << ", Theta: " << next_theta << ", Vx: " << next_vx
                  << ", Vy: " << next_vy << ", Omega: " << next_omega << std::endl;

        // if((target_x-current_x)*(target_x-current_x) + (target_y-current_y)*(target_y-current_y) < 0.1*0.1) {
        //         geometry_msgs::msg::Twist twist_msg;
        // 	twist_msg.linear.x = 0;
        // 	twist_msg.linear.y = 0;
	// 	if (abs(target_theta - current_theta) < 0.05) {
	// 		twist_msg.angular.z = 0;
	// 		next_omega = 0;
	// 	} else if (abs(target_theta - current_theta) < 0.15) {
	// 		twist_msg.angular.z = next_omega / 2;
	// 		next_omega /= 2;
	// 	} else {
	// 		twist_msg.angular.z = next_omega;
	// 	}
        // 	cmd_vel_publisher->publish(twist_msg);
        // 	std::cout<<"Bot stopped, next_omega = "<< next_omega <<std::endl;
        // 	return;
    	// }
	
	// Publish the Twist message
	geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = next_vx;
	twist_msg.linear.y = next_vy;
        twist_msg.angular.z = next_omega;
        cmd_vel_publisher->publish(twist_msg);
    }

    // Callback for updating the target position
    void update_target_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        target_mutex.lock();
	target_x = msg->data[0];
	target_y = msg->data[1];
	target_theta = msg->data[2];
        idx = msg->data[3];
        path_size = msg->data[4];
        target_mutex.unlock();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr self_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_subscription;

    std::mutex target_mutex;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyRobotNode>());
    rclcpp::shutdown();
    return 0;
}