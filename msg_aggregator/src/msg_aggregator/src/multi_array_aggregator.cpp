#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <string>

class Robot {
public:
    std::string name;
    double x, y, theta;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber;

    Robot(rclcpp::Node* node, const std::string& name, double x, double y, double theta) 
        : name(name), x(x), y(y), theta(theta) {
        subscriber = node->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/" + name + "_data", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 3) {
                    this->x = msg->data[0];
                    this->y = msg->data[1];
                    this->theta = msg->data[2];
                }
            }
        );
    }
};

class MultiRobotManager : public rclcpp::Node {
public:
    MultiRobotManager() : Node("multi_robot_manager") {
        // Create multiple robots
        robots_.emplace_back(std::make_shared<Robot>(this, "o2", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "o3", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "o4", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "o5", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "b1", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "b2", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "b3", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "b4", 0.0, 0.0, 0.0));
        robots_.emplace_back(std::make_shared<Robot>(this, "b5", 0.0, 0.0, 0.0));

        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obstacles", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MultiRobotManager::publish_aggregated_positions, this)
        );
    }

private:
    void publish_aggregated_positions() {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back(robots_.size());
        for (const auto& robot : robots_) {
            msg.data.push_back(robot->x);
            msg.data.push_back(robot->y);
            msg.data.push_back(robot->theta);
        }
        publisher_->publish(msg);
    }

    std::vector<std::shared_ptr<Robot>> robots_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRobotManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
