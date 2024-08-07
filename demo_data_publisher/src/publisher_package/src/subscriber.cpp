#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener_node")
    {
        // Create subscriptions for two topics
        subscription1_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose1", 10, std::bind(&ListenerNode::pose1_callback, this, std::placeholders::_1));
        
        subscription2_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pose2", 10, std::bind(&ListenerNode::pose2_callback, this, std::placeholders::_1));
    }

private:
    void pose1_callback(const geometry_msgs::msg::Pose::SharedPtr message)
    {
        // Process message from topic pose1
        RCLCPP_INFO(this->get_logger(), "Received message from pose1: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
            message->position.x, message->position.y, message->position.z,
            message->orientation.x, message->orientation.y, message->orientation.z, message->orientation.w);
    }

    void pose2_callback(const geometry_msgs::msg::Pose::SharedPtr message)
    {
        // Process message from topic pose2
        RCLCPP_INFO(this->get_logger(), "Received message from pose2: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
            message->position.x, message->position.y, message->position.z,
            message->orientation.x, message->orientation.y, message->orientation.z, message->orientation.w);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription1_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription2_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}
