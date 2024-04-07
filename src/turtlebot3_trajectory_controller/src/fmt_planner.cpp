//
// Created by lay on 24-4-7.
//
//
// Created by lay on 23-9-1.
//
#include <string>
#include <chrono>
#include <Eigen/Eigen>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "gtec_msgs/msg/formation.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

void traj_gen(Vector2d &pos_d, const double t) {
    double cruising_vel = 0.5;
    double transition_vel = 0.3;
    double round_r = 2;
    double length = 10;
    auto transition_time = round_r * 3.14 / 2 / transition_vel;
    auto cruising_time1 = (length - round_r) / cruising_vel;
    auto cruising_time2 = (length - 2 * round_r) / cruising_vel;

    auto ts1 = cruising_time1;
    auto ts2 = ts1 + transition_time;
    auto ts3 = ts2 + cruising_time2;
    auto ts4 = ts3 + transition_time;
    auto ts5 = ts4 + cruising_time2;
    auto ts6 = ts5 + transition_time;
    auto ts7 = ts6 + cruising_time1;
    if (t < ts1) {
        pos_d(0) = cruising_vel * t;
        pos_d(1) = 0;
    } else if (t < ts2) {
        pos_d(0) = length - round_r + round_r * sin((t - ts1) * transition_vel / round_r);
        pos_d(1) = round_r - round_r * cos((t - ts1) * transition_vel / round_r);
    } else if (t < ts3) {
        pos_d(0) = length;
        pos_d(1) = round_r + cruising_vel * (t - ts2);
    } else if (t < ts4) {
        pos_d(0) = length - round_r + round_r * cos((t - ts3) * transition_vel / round_r);
        pos_d(1) = length - round_r + round_r * sin((t - ts3) * transition_vel / round_r);
    } else if (t < ts5) {
        pos_d(0) = length - round_r - cruising_vel * (t - ts4);
        pos_d(1) = length;
    } else if (t < ts6) {
        pos_d(0) = round_r - round_r * sin((t - ts5) * transition_vel / round_r);
        pos_d(1) = length - round_r + round_r * cos((t - ts5) * transition_vel / round_r);
    } else if (t < ts7) {
        pos_d(0) = 0;
        pos_d(1) = length - round_r - cruising_vel * (t - ts6);
    } else {
        pos_d(0) = 0;
        pos_d(1) = 0;
    }

//    auto pos_bias1_B = Vector2d(-2, 2);
//    auto pos_bias2_B = Vector2d(-2, -2);
//    auto rot_WB = q.toRotationMatrix().block(0, 0, 2, 2);
//    auto pos_bias1_W = rot_WB * pos_bias1_B;
//    auto pos_bias2_W = rot_WB * pos_bias2_B;
//    auto pos_db1 = pos_d + pos_bias1_W;
//    auto pos_db2 = pos_d + pos_bias2_W;
//    pos_d = pos_db2;
///////////////////////////////
//    if (t < 2) {
//        pos_d(0) = t;
//        pos_d(1) = t;
//    } else if (t < 4) {
//        pos_d(0) = 4 - t;
//        pos_d(1) = t;
//    } else if (t < 8) {
//        pos_d(0) = 0;
//        pos_d(1) = 8 - t;
//    } else {
//        pos_d(0) = 0;
//        pos_d(1) = 0;
//    }
////////////////////////////////
//    if (t < 4) {
//        pos_d(0) = 0.5 * t;
//        pos_d(1) = 0;
//    } else if (t < 8) {
//        pos_d(0) = 2;
//        pos_d(1) = 0.5 * (t - 4);
//    } else if (t < 12) {
//        pos_d(0) = 0.5 * (12 - t);
//        pos_d(1) = 2;
//    } else if (t < 16) {
//        pos_d(0) = 0;
//        pos_d(1) = 0.5 * (16 - t);
//    } else {
//        pos_d(0) = 0;
//        pos_d(1) = 0;
//    }
}

class CarFmtPlanner : public rclcpp::Node {
public:
    CarFmtPlanner() : Node("car_fmt_planner") {

        std::string fmt_topic;
        this->declare_parameter<std::string>("namespace", "/scout");
        this->declare_parameter<std::string>("fmt_topic", "/formation");
        this->get_parameter("namespace", namespace_);
        this->get_parameter("fmt_topic", fmt_topic);

        isFmtSet = false;
        ist0set = false;
        isLeaderStateUpdate = false;

        fmt_sub = this->create_subscription<gtec_msgs::msg::Formation>(
                fmt_topic, 10, std::bind(&CarFmtPlanner::fmt_callback, this, _1));
        timer_ = this->create_wall_timer(20ms, std::bind(&CarFmtPlanner::timer_callback, this));
    }

private:
    void timer_callback();

    void fmt_callback(const gtec_msgs::msg::Formation::SharedPtr msg);

    void leader_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::string namespace_;
    Eigen::Vector2d pos_L;
    Eigen::Quaterniond q_L;
    bool isFmtSet;
    bool ist0set;
    bool isLeaderStateUpdate;
    int num_followers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr> follower_publisher;
    std::vector<Eigen::Vector2d> bias;

    rclcpp::Subscription<gtec_msgs::msg::Formation>::SharedPtr fmt_sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr leader_trajectory_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_ground_truth_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

void CarFmtPlanner::timer_callback() {
    if (isFmtSet and isLeaderStateUpdate) {
        static double t0;
        if (!ist0set) {
            t0 = this->get_clock()->now().seconds();
            ist0set = true;
        }
        double t = this->get_clock()->now().seconds() - t0;
        Vector2d pos_d;
        traj_gen(pos_d, t);
        {
            geometry_msgs::msg::Vector3 msg;
            msg.x = pos_d(0);
            msg.y = pos_d(1);
            leader_trajectory_pub->publish(msg);
        }
        auto rot_WB = q_L.toRotationMatrix().block(0, 0, 2, 2);
        for (int i = 0; i < num_followers; ++i) {
            auto pos_db = pos_L + rot_WB * bias[i];
            geometry_msgs::msg::Vector3 msg;
            msg.x = pos_db(0);
            msg.y = pos_db(1);
            follower_publisher[i]->publish(msg);
        }
    }
}

// 队形订阅回调
void CarFmtPlanner::fmt_callback(const gtec_msgs::msg::Formation::SharedPtr msg) {
    if (!isFmtSet) {
        num_followers = msg->num_fmt - 1;
        auto leader_id = msg->robotid_list[0];
        auto leader_trajectory_topic = namespace_ + std::to_string(leader_id) + "/trajectory";
        leader_trajectory_pub = this->create_publisher<geometry_msgs::msg::Vector3>(leader_trajectory_topic, 10);

        auto leader_state_topic = namespace_ + std::to_string(leader_id) + "/state_ground_truth";
        leader_ground_truth_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                leader_state_topic, 10,
                std::bind(&CarFmtPlanner::leader_ground_truth_callback, this, _1));

        for (int i = 1; i < msg->num_fmt; i++) {
            auto trajectory_topic = namespace_ + std::to_string(msg->robotid_list[i]) + "/trajectory";
            auto follower_pub = this->create_publisher<geometry_msgs::msg::Vector3>(trajectory_topic, 10);
            follower_publisher.push_back(follower_pub);

            Vector2d offset{msg->x[i] - msg->x[0], msg->y[i] - msg->y[0]};
            bias.push_back(offset);
        }
    }
    isFmtSet = true;
}

void CarFmtPlanner::leader_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    isLeaderStateUpdate = true;
    pos_L << msg->pose.pose.position.x, msg->pose.pose.position.y;
    q_L.coeffs() << msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarFmtPlanner>());
    rclcpp::shutdown();
    return 0;
}