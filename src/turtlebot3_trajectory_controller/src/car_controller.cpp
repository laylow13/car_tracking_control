//
// Created by lay on 23-9-1.
//
#include <string>
#include <chrono>
#include <Eigen/Eigen>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace Eigen;

class CarTrackingController {
public:
    CarTrackingController() = default;

    CarTrackingController(double kpL, double kiL, double kdL, double kpA, double kiA, double kdA, double _period)
            : kp_l(kpL),
              ki_l(kiL),
              kd_l(kdL),
              kp_a(kpA),
              ki_a(kiA),
              kd_a(kdA),
              period(_period) {};

    void set_param(double kpL, double kiL, double kdL, double kpA, double kiA, double kdA, double _period) {
        kp_l = kpL;
        ki_l = kiL;
        kd_l = kdL;
        kp_a = kpA;
        ki_a = kiA;
        kd_a = kdA;
        period = _period;
    };

    void calculate_cmd(const Vector2d &_pos, const Quaterniond &_q, const Vector2d &_pos_d) {
        Vector2d pos_err_W = _pos_d - _pos;
        auto rot_BW = _q.toRotationMatrix().transpose().block(0, 0, 2, 2);
        auto pos_err_B = rot_BW * pos_err_W;
        double ang_err = std::atan2(pos_err_B(1), pos_err_B(0));
        double dist_err = pos_err_B.norm();

//        if (abs(dist_err) < 0.05) {
//            ang_err = 0;
//            dist_err = 0;
//        }

        if (abs(dist_err) < 0.05) {
            dist_err *= cos(ang_err);
            ang_err = 0;
        } else
            dist_err *= cos(ang_err);

        dist_err = dist_err < 0 ? 0 : dist_err;

        dist_err_int += dist_err * period;
        ang_err_int += ang_err * period;
        limit(dist_err_int, 2);
        limit(ang_err_int, 2);

        cmd_vel_l =
                kp_l * dist_err + ki_l * dist_err_int +
                kd_l * (dist_err - last_dist_err) / period;
        cmd_vel_a =
                kp_a * ang_err + ki_a * ang_err_int +
                kd_a * (ang_err - last_ang_err) / period;

        limit(cmd_vel_l, 1.5);
        limit(cmd_vel_a, 1.0);

        last_dist_err = dist_err;
        last_ang_err = ang_err;
    }

    void get_cmd(double &cmd_vel_linear, double &cmd_vel_angular) const {
        cmd_vel_linear = cmd_vel_l;
        cmd_vel_angular = cmd_vel_a;
    }

    void get_ang_err(double &ang_err) { ang_err = last_ang_err; };

private:
    double kp_l;
    double ki_l;
    double kd_l;
    double kp_a;
    double ki_a;
    double kd_a;
    double period;
    double dist_err_int;
    double ang_err_int;
    double last_dist_err;
    double last_ang_err;
    double cmd_vel_l;
    double cmd_vel_a;

    void limit(double &value, double lim) {
        if (value > lim)
            value = lim;
        else if (value < -lim)
            value = -lim;
    }
};

class CarFmtTracker : public rclcpp::Node {
public:
    CarFmtTracker() : Node("car_fmt_tracker") {
        isPosUpdate = false;
        isTrajectoryUpdate = false;

        pos = Eigen::Vector2d(0, 0);
        pos_d = Eigen::Vector2d(0, 0);
        q = Eigen::Quaterniond(1, 0, 0, 0);
        controller = CarTrackingController(1.0, 0, 0.2, 1.0, 0, 0, 0.05);

        this->declare_parameter<std::string>("namespace", "/scout");
        this->declare_parameter<int>("node_id", 0);

        this->get_parameter("namespace", namespace_);
        this->get_parameter("node_id", id);

        auto cmd_topic_name = namespace_ + std::to_string(id) + "/cmd_vel";
        auto state_topic_name = namespace_ + std::to_string(id) + "/state_ground_truth";
        auto trajectory_name = namespace_ + std::to_string(id) + "/trajectory";

        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_name, 10);
        vehicle_ground_truth_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                state_topic_name, 10, std::bind(&CarFmtTracker::vehicle_ground_truth_callback, this, _1));
        trajectory_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                trajectory_name, 10, std::bind(&CarFmtTracker::desired_trajectory_callback, this, _1));
        timer_ = this->create_wall_timer(50ms, std::bind(&CarFmtTracker::timer_callback, this));
    }

private:
    void timer_callback();

    void vehicle_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void desired_trajectory_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

    bool isPosUpdate;
    bool isTrajectoryUpdate;
    uint16_t id;
    Eigen::Vector2d pos;
    Eigen::Vector2d pos_d;
    Eigen::Quaterniond q;
    CarTrackingController controller;
    std::string namespace_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_ground_truth_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr trajectory_sub;
    rclcpp::TimerBase::SharedPtr timer_;
};

void CarFmtTracker::timer_callback() {
    if (isTrajectoryUpdate && isPosUpdate) {
        controller.calculate_cmd(pos, q, pos_d);
        double cmd_vel_linear, cmd_vel_angular;
        controller.get_cmd(cmd_vel_linear, cmd_vel_angular);
        geometry_msgs::msg::Twist msg;
        msg.linear.x = cmd_vel_linear;
        msg.angular.z = cmd_vel_angular;
        cmd_pub->publish(msg);
    }
}

void CarFmtTracker::desired_trajectory_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    isTrajectoryUpdate = true;
    pos_d << msg->x, msg->y;
}

void CarFmtTracker::vehicle_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    isPosUpdate = true;
    pos << msg->pose.pose.position.x, msg->pose.pose.position.y;
    q.coeffs() << msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CarFmtTracker>());
    rclcpp::shutdown();
    return 0;
}