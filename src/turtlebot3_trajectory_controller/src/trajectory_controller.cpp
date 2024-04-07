#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <Eigen/Eigen>

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


void timer_callback();

void traj_gen(Vector2d &pos_d, const double t);

void state_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

void on_shutdown_cb();

std::shared_ptr<rclcpp::Node> controller_node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr monitor_pub;
const double period = 0.020; // s
Vector2d pos;
Quaterniond q;
CarTrackingController controller;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    controller_node = std::make_shared<rclcpp::Node>("trajectory_controller");
    controller_node->declare_parameter<double>("kp_pos", 1.0);
    controller_node->declare_parameter<double>("ki_pos", 0.0);
    controller_node->declare_parameter<double>("kd_pos", 0.2);
    controller_node->declare_parameter<double>("kp_ang", 1.0);
    controller_node->declare_parameter<double>("ki_ang", 0.0);
    controller_node->declare_parameter<double>("kd_ang", 0.0);
    double kp_pos = controller_node->get_parameter("kp_pos").get_value<double>();
    double ki_pos = controller_node->get_parameter("ki_pos").get_value<double>();
    double kd_pos = controller_node->get_parameter("kd_pos").get_value<double>();
    double kp_ang = controller_node->get_parameter("kp_ang").get_value<double>();
    double ki_ang = controller_node->get_parameter("ki_ang").get_value<double>();
    double kd_ang = controller_node->get_parameter("kd_ang").get_value<double>();
    controller.set_param(kp_pos, ki_pos, kd_pos, kp_ang, ki_ang, kd_ang, period);
    cmd_pub = controller_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    monitor_pub = controller_node->create_publisher<geometry_msgs::msg::Vector3>("/pos_d", 10);
    auto state_sub = controller_node->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, &state_cb);
    rclcpp::TimerBase::SharedPtr timer = controller_node->create_wall_timer(
            std::chrono::milliseconds(int(period * 1e3)), &timer_callback);
//    rclcpp::on_shutdown(on_shutdown_cb);
    rclcpp::spin(controller_node);
    rclcpp::shutdown();
    return 0;
}

void timer_callback() {
    static double t = 0;
    Vector2d pos_d;
    traj_gen(pos_d, t);
    controller.calculate_cmd(pos, q, pos_d);
//    {
//        double ang_err;
//        controller.get_ang_err(ang_err);
//        RCLCPP_INFO(rclcpp::get_logger("controller"), "ang_err:%f", ang_err);
//    }
    double cmd_vel_linear, cmd_vel_angular;
    controller.get_cmd(cmd_vel_linear, cmd_vel_angular);
    {
        geometry_msgs::msg::Vector3 msg;
        msg.x = pos_d(0);
        msg.y = pos_d(1);
        monitor_pub->publish(msg);
    }
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = cmd_vel_linear;
        msg.angular.z = cmd_vel_angular;
        cmd_pub->publish(msg);
    }
    t += period;
}

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

    auto pos_bias1_B = Vector2d(-2, 2);
    auto pos_bias2_B = Vector2d(-2, -2);
    auto rot_WB = q.toRotationMatrix().block(0, 0, 2, 2);
    auto pos_bias1_W = rot_WB * pos_bias1_B;
    auto pos_bias2_W = rot_WB * pos_bias2_B;
    auto pos_db1 = pos_d + pos_bias1_W;
    auto pos_db2 = pos_d + pos_bias2_W;
    pos_d = pos_db2;
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

void state_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    pos << msg->pose.pose.position.x, msg->pose.pose.position.y;
    q = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
}

void on_shutdown_cb() {
    RCLCPP_INFO(rclcpp::get_logger("controller"), "end");
    std::cout << "end";
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    cmd_pub->publish(msg);
}