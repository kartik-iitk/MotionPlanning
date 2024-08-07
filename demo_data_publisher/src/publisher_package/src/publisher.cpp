#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("talker_node")
    {
        // Create publishers for topics
        my_publisher = this->create_publisher<geometry_msgs::msg::Pose>("my_pose", 10);
        bot1_publisher = this->create_publisher<geometry_msgs::msg::Pose>("bot1_pose", 10);
        bot2_publisher = this->create_publisher<geometry_msgs::msg::Pose>("bot2_pose", 10);
        bot3_publisher = this->create_publisher<geometry_msgs::msg::Pose>("bot3_pose", 10);
        bot4_publisher = this->create_publisher<geometry_msgs::msg::Pose>("bot4_pose", 10);
        bot5_publisher = this->create_publisher<geometry_msgs::msg::Pose>("bot5_pose", 10);
        ball_publisher = this->create_publisher<geometry_msgs::msg::Pose>("ball_pose", 10);
        target_publisher = this->create_publisher<geometry_msgs::msg::Pose>("target_pose", 10);

        // Create a timer to publish messages periodically
        timer_ = this->create_wall_timer(200ms, std::bind(&TalkerNode::publish_messages, this));
    }

private:
    void publish_messages()
    {
        auto my_pose = geometry_msgs::msg::Pose();
        my_pose.position.x = 0.0;
        my_pose.position.y = 0.0;
        my_pose.position.z = 0.0;
        my_pose.orientation.x = 0.0;
        my_pose.orientation.y = 0.0;
        my_pose.orientation.z = 0.0;
        my_pose.orientation.w = 0.0;
        my_publisher->publish(my_pose);

        auto bot1_pose = geometry_msgs::msg::Pose();
        bot1_pose.position.x = 2.0;
        bot1_pose.position.y = -2.0;
        bot1_pose.position.z = 1.0;
        bot1_pose.orientation.x = 0.0;
        bot1_pose.orientation.y = 0.0;
        bot1_pose.orientation.z = 0.0;
        bot1_pose.orientation.w = 0.0;
        bot1_publisher->publish(bot1_pose);

        auto bot2_pose = geometry_msgs::msg::Pose();
        bot2_pose.position.x = 2.0;
        bot2_pose.position.y = -1.0;
        bot2_pose.position.z = 0.0;
        bot2_pose.orientation.x = 0.0;
        bot2_pose.orientation.y = 0.0;
        bot2_pose.orientation.z = 0.0;
        bot2_pose.orientation.w = 0.0;
        bot2_publisher->publish(bot2_pose);

        auto bot3_pose = geometry_msgs::msg::Pose();
        bot3_pose.position.x = 2.0;
        bot3_pose.position.y = 0.0;
        bot3_pose.position.z = 0.0;
        bot3_pose.orientation.x = 0.0;
        bot3_pose.orientation.y = 0.0;
        bot3_pose.orientation.z = 0.0;
        bot3_pose.orientation.w = 0.0;
        bot3_publisher->publish(bot3_pose);

        auto bot4_pose = geometry_msgs::msg::Pose();
        bot4_pose.position.x = 2.0;
        bot4_pose.position.y = 1.0;
        bot4_pose.position.z = 0.0; 
        bot4_pose.orientation.x = 0.0;  
        bot4_pose.orientation.y = 0.0;
        bot4_pose.orientation.z = 0.0;
        bot4_pose.orientation.w = 0.0;
        bot4_publisher->publish(bot4_pose);
        
        auto bot5_pose = geometry_msgs::msg::Pose();
        bot5_pose.position.x = 2.0;
        bot5_pose.position.y = 2.0;
        bot5_pose.position.z = 0.0;
        bot5_pose.orientation.x = 0.0;
        bot5_pose.orientation.y = 0.0;
        bot5_pose.orientation.z = 0.0;
        bot5_pose.orientation.w = 0.0;
        bot5_publisher->publish(bot5_pose);

        auto ball_pose = geometry_msgs::msg::Pose();
        ball_pose.position.x = -5.0;
        ball_pose.position.y = 0.0;
        ball_pose.position.z = 0.0;
        ball_pose.orientation.x = 0.0;
        ball_pose.orientation.y = 0.0;
        ball_pose.orientation.z = 0.0;
        ball_pose.orientation.w = 0.0;
        ball_publisher->publish(ball_pose);

        auto target_pose = geometry_msgs::msg::Pose();
        target_pose.position.x = -7.0;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;
        target_pose.orientation.w = 0.0;
        target_publisher->publish(target_pose);
    }

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr my_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bot1_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bot2_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bot3_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bot4_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr bot5_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ball_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}
