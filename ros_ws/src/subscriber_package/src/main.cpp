#include <iostream>
#include <cmath>
#include <vector>

#include "Coordinate.hpp"
#include "icecream.hpp"
#include "Motion.hpp"
#include "PathPlanner.hpp"
#include "Robot.hpp"
#include "Visualize.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using namespace std;

using namespace robot;

Visualize *window = new Visualize(1600);
double runTime = 0.01;
double A = 22, B = 14;
optimalPlanner plannerType = PLANNER_RRTSTAR;
planningObjective objectiveType = OBJECTIVE_PATHLENGTH;

long long int count1 = 0;
long long int count2 = 1;
int num_of_bots = 1, count3 = 0, flag = 0;
double obs_size = 0.45;
double yaw = 0.0;
std::vector<Point2D> obs(6), targetPos;
Point2D nowPos(0, 0, 0), finalPos(0,0,0), ballPos(0,0,0);
std::vector<pair<double, double>> points;
Point2D outputPID(0, 0, 0.654);
wheelAngularVel outInvers;
std::vector<double> encData{0, 0, 0, 0};
double enc[4]={0,0,0,0};
Motion *mot = new Motion();
auto ang_vel = std::make_shared<geometry_msgs::msg::Pose>();

void set_obstacles(geometry_msgs::msg::Pose bot1_pose, geometry_msgs::msg::Pose bot2_pose, geometry_msgs::msg::Pose bot3_pose, geometry_msgs::msg::Pose bot4_pose, geometry_msgs::msg::Pose bot5_pose, geometry_msgs::msg::Pose ball_pose)
{
    obs[0] = Point2D(ball_pose.position.x, ball_pose.position.y, ball_pose.position.z);
    obs[1] = Point2D(bot1_pose.position.x, bot1_pose.position.y, bot1_pose.position.z);
    obs[2] = Point2D(bot2_pose.position.x, bot2_pose.position.y, bot2_pose.position.z);
    obs[3] = Point2D(bot3_pose.position.x, bot3_pose.position.y, bot3_pose.position.z);
    obs[4] = Point2D(bot4_pose.position.x, bot4_pose.position.y, bot4_pose.position.z); 
    obs[5] = Point2D(bot5_pose.position.x, bot5_pose.position.y, bot5_pose.position.z);
}

int findclosestpoint(std::vector<Point2D> &targetPos, Point2D &nowPos) {
    double min = 1000000;
    int idx = 0;
    for (int i = 0; i < targetPos.size() - 1; i++)  // excluding last point and start
    {
        if (sqrtl((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                 (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y)) <
            min) {
            min =
                sqrt((targetPos[i].x - nowPos.x) * (targetPos[i].x - nowPos.x) +
                     (targetPos[i].y - nowPos.y) * (targetPos[i].y - nowPos.y));
            idx = i;
        }
    }
    return idx;
}

// Function to check if a point lies within the extended region of a line
bool isok(const Point2D &p1, const Point2D &p2, const Point2D &testPoint) {
    // finds perpendicular isstance between line joining p1 and p2 and the
    // obstacles
    double distance =
        std::abs((p2.y - p1.y) * testPoint.x - (p2.x - p1.x) * testPoint.y +
                 p2.x * p1.y - p2.y * p1.x) /
        std::sqrt(std::pow(p2.y - p1.y, 2) + std::pow(p2.x - p1.x, 2));

    return distance < 0.45;  // Check if the distance is within the extended region
}

void get_Trajectory(std::vector<Point2D> &path, Point2D &outputPID,
                    Point2D &nowPos, wheelAngularVel &outInvers, double yaw,
                    std::vector<Point2D> &obstacles, Point2D &ball) {
    window->visualizeGame(path, nowPos, count3, yaw, obstacles, ball);

    path[count3].theta = (180.0 / 3.14159) * atan2((ball.y - path[count3].y),
                                                   (ball.x - path[count3].x));

    double errorX = path[count3].x - nowPos.x;
    double errorY = path[count3].y - nowPos.y;

    double dist = sqrt(errorX * errorX + errorY * errorY);
    double errTheta = path[count3].theta - (nowPos.theta * 180 / M_PI);

    // Limit errTheta to -180 to +180 degrees
    if (errTheta > 180) errTheta -= 360;
    if (errTheta < -180) errTheta += 360;

    double nearestX, nearestY,
        minDistance = std::numeric_limits<double>::infinity();
    for (int j = 0; j < path.size(); j++) {
        auto i = path[j];
        double dist = std::sqrt((i.x - nowPos.x) * (i.x - nowPos.x) +
                                (i.y - nowPos.y) * (i.y - nowPos.y));
        if (dist < minDistance) {
            minDistance = dist;
            nearestX = i.x;
            nearestY = i.y;
            count3 = j;
        }
    }

    mot->positionAngularControl(errorX, errorY, errTheta, yaw, outputPID,
                                nearestX, nearestY, minDistance, nowPos.x,
                                nowPos.y, path.back(), count3, path);

    RobotKinematic::getInstance()->inverseKinematics(
        outInvers, outputPID.x, outputPID.y, outputPID.theta);

    if (count3 > path.size() - 1) {
        count3 = path.size() - 1;
    }
}

void print_vel(wheelAngularVel &outMotor) {
    std::cout << "w1: " << outMotor.w1 << " "
              << "w2: " << outMotor.w2 << " "
              << "w3: " << outMotor.w3 << " "
              << "w4: " << outMotor.w4 << std::endl;
}


class ListenerNode : public rclcpp::Node
{
public:
    ListenerNode() : Node("listener_node")
    {
        // Create subscribers for two topics
        my_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "my_pose", 10, std::bind(&ListenerNode::my_callback, this, std::placeholders::_1));

        bot1_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "bot1_pose", 10, std::bind(&ListenerNode::bot1_callback, this, std::placeholders::_1));
        
        bot2_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "bot2_pose", 10, std::bind(&ListenerNode::bot2_callback, this, std::placeholders::_1));

        bot3_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "bot3_pose", 10, std::bind(&ListenerNode::bot3_callback, this, std::placeholders::_1));
        
        bot4_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "bot4_pose", 10, std::bind(&ListenerNode::bot4_callback, this, std::placeholders::_1));
        
        bot5_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "bot5_pose", 10, std::bind(&ListenerNode::bot5_callback, this, std::placeholders::_1));
        
        ball_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "ball_pose", 10, std::bind(&ListenerNode::ball_callback, this, std::placeholders::_1));
        
        target_subscriber = this->create_subscription<geometry_msgs::msg::Pose>(
            "target_pose", 10, std::bind(&ListenerNode::target_callback, this, std::placeholders::_1));

        target_array_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("target_array", 10);

        ang_vel_publisher = this->create_publisher<geometry_msgs::msg::Pose>("ang_vel", 10);
    }

    // Callback funtion
    void my_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        cout<<"my_callback"<<endl;
        my_pose = *msg;
        // swap(my_pose.position.x, my_pose.position.y);
        nowPos = Point2D(my_pose.position.x, my_pose.position.y, my_pose.position.z);
        my_pose_received = true;
        yaw = my_pose.position.z;
        enc[0] = my_pose.orientation.x;
        enc[1] = my_pose.orientation.y;
        enc[2] = my_pose.orientation.z;
        enc[3] = my_pose.orientation.w;
        listening();
    }

    void bot1_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        bot1_pose = *msg;
        bot1_pose_received = true;
        // listening();
    }

    void bot2_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        bot2_pose = *msg;
        bot2_pose_received = true;
        // listening();
    }

    void bot3_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        bot3_pose = *msg;
        bot3_pose_received = true;
        // listening();
    }

    void bot4_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        bot4_pose = *msg;
        bot4_pose_received = true;
        // listening();
    }

    void bot5_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        bot5_pose = *msg;
        bot5_pose_received = true;
        // listening();
    }

    void ball_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        ball_pose = *msg;
        ballPos = Point2D(ball_pose.position.x, ball_pose.position.y, ball_pose.position.z);
        ball_pose_received = true;
        // listening();
    }

    void target_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        target_pose = *msg;
        finalPos = Point2D(target_pose.position.x, target_pose.position.y, target_pose.position.z);
        target_pose_received = true;
        // listening();
    }

    void listening()
    {
        if (!my_pose_received || !bot1_pose_received || !bot2_pose_received || !bot2_pose_received || !bot3_pose_received || !bot4_pose_received || !bot5_pose_received || !ball_pose_received || !target_pose_received)
        {
            return;
        }

        set_obstacles(bot1_pose, bot2_pose, bot3_pose, bot4_pose, bot5_pose, ball_pose);

        if (count2 % 5 == 0) {
            int idx = findclosestpoint(targetPos, nowPos);  // excluding the last point
            for (auto &it : obstacles) {
                if (idx + 1 < obstacles.size() && idx - 1 > -1) {
                    if (!isok(targetPos[idx], targetPos[idx + 1], it) &&
                        !isok(targetPos[idx - 1], targetPos[idx], it))
                        count1++;
                } else if (idx + 1 >= obstacles.size()) {
                    if (!isok(targetPos[idx - 1], targetPos[idx], it)) count1++;
                } else if (idx - 1 < -1) {
                    if (!isok(targetPos[idx], targetPos[idx + 1], it)) count1++;
                }
            }
            if (count1 != obstacles.size()) flag = 1;
            std::cout << count1 << std::endl;
            count1 = 0;
        }

        if (flag || count2 == 1) {
            try
            {
                points = plan(runTime, A, B, obs, plannerType, objectiveType, nowPos, finalPos);
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
                points = plan(runTime, A, B, obs, plannerType, objectiveType, nowPos, nowPos);
            }
            
            targetPos.clear();
            for (auto ptr : points)
            {
                int angle = (180.0 / 3.14159) * atan2((ballPos.y - ptr.second), (ballPos.x - ptr.first));
                targetPos.push_back(Point2D(ptr.first, ptr.second, angle));
            }
            flag = 0;
        }

        count2++;
        count2 = count2 % 1000000000000000000;
        
        get_Trajectory(targetPos, outputPID, nowPos, outInvers, yaw, obs, ballPos);

        publish_next_point(targetPos, nowPos);

        // print_vel(outInvers);

        ang_vel->position.x = 0.0;
        ang_vel->position.y = 0.0;
        ang_vel->position.z = 0.0;
        ang_vel->orientation.x = outInvers.w1;
        ang_vel->orientation.y = outInvers.w2;
        ang_vel->orientation.z = outInvers.w3;
        ang_vel->orientation.w = outInvers.w4;

        ang_vel_publisher->publish(*ang_vel);

        // Assign Enc val
        for (int i = 0; i < 4; i++) {
            RobotKinematic::getInstance()->encData[i] = enc[i];
        }
        RobotKinematic::getInstance()->calculateOdometry(yaw);
    }

    // Variables to store received messages
    geometry_msgs::msg::Pose my_pose;
    geometry_msgs::msg::Pose bot1_pose;
    geometry_msgs::msg::Pose bot2_pose;
    geometry_msgs::msg::Pose bot3_pose;
    geometry_msgs::msg::Pose bot4_pose;
    geometry_msgs::msg::Pose bot5_pose;
    geometry_msgs::msg::Pose ball_pose;
    geometry_msgs::msg::Pose target_pose;
    bool my_pose_received{false};
    bool bot1_pose_received{false};
    bool bot2_pose_received{false};
    bool bot3_pose_received{false};
    bool bot4_pose_received{false};
    bool bot5_pose_received{false};
    bool ball_pose_received{false};
    bool target_pose_received{false};

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr my_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bot1_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bot2_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bot3_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bot4_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr bot5_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ball_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_subscriber;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ang_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_array_publisher;

    void publish_next_point(const std::vector<Point2D> &path, const Point2D &nowPos)
{
    if (!path.empty())
    {
        auto next_point = path[count3];  // Publishing the next point in the path
        // auto message = std_msgs::msg::Float32MultiArray();
        
        // Push x, y, theta, vx, vy, omega
        // messages.data.push_back(next_point.x);
        // messages.data.push_back(next_point.y);
        // messages.data.push_back(next_point.theta);
        float x = next_point.x;
        float y = next_point.y;
        float theta = next_point.theta;

        //angle between nowpos and next point
        int angle = (180.0 / 3.14159) * atan2((next_point.y - nowPos.y), (next_point.x - nowPos.x));

        float vx = 1 * cos(angle * 3.14159 / 180); 
        float vy = 1 * sin(angle * 3.14159 / 180);
        float omega = 1;

        // message.data.push_back(vx);
        // message.data.push_back(vy);
        // message.data.push_back(omega);

        // int message[6] = {x, y, theta, vx, vy, omega};
        std_msgs::msg::Float32MultiArray message;
        float data[6] = {x, y, theta, vx, vy, omega};
        message.data = {data[0], data[1], data[2], data[3], data[4], data[5]};

        target_array_publisher->publish(message);
    }
}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ListenerNode>());
    rclcpp::shutdown();
    return 0;
}