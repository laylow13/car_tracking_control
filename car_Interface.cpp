#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <chrono>
#include <string>
#include <thread>
#include "gtec_msgs/msg/linktrack_nodeframe3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "gtec_msgs/msg/linktrack_node2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"
#include "exlcm/statemsg.hpp"
#include "exlcm/datafreq.hpp"
#include "exlcm/uwbmsg.hpp"
#include "exlcm/imumsg.hpp"
#include "exlcm/imagemsg.hpp"
#include "exlcm/pointcloud.hpp"

#include "gtec_msgs/msg/states.hpp"
#include "gtec_msgs/msg/sensor_combined.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/path.hpp"

#include "gtec_msgs/msg/formation.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <unistd.h>
#include <iostream>
#include <fstream>
// using namespace std;

using namespace std::chrono;
using namespace std::chrono_literals;

using std::placeholders::_1;

const double pi = 3.141592653589793;

double limvar(double x, double xmin, double xmax) {
    if (x < xmin) {
        return xmin;
    } else if (x > xmax) {
        return xmax;
    } else {
        return x;
    }
}

void getdisangle2(double x0, double y0, double thet0, double x1, double y1, double &d, double &thet) {
    double dx = -(x0 - x1);
    double dy = -(y0 - y1);
    double xp = dx * cos(thet0) + dy * sin(thet0);
    double yp = -dx * sin(thet0) + dy * cos(thet0);
    d = sqrt(dx * dx + dy * dy);
    if (d > 0.1)
        thet = atan2(yp, xp);
    else
        thet = 0.0;
}

void quaternion_from_euler(double r, double p, double y, double &qw, double &qx, double &qy, double &qz) {
    // q = [0, 0, 0, 0]
    qw = cos(r / 2) * cos(p / 2) * cos(y / 2) + sin(r / 2) * sin(p / 2) * sin(y / 2);
    qx = sin(r / 2) * cos(p / 2) * cos(y / 2) - cos(r / 2) * sin(p / 2) * sin(y / 2);
    qy = cos(r / 2) * sin(p / 2) * cos(y / 2) + sin(r / 2) * cos(p / 2) * sin(y / 2);
    qz = cos(r / 2) * cos(p / 2) * sin(y / 2) - sin(r / 2) * sin(p / 2) * cos(y / 2);
}

void euler_from_quaternion(double qx, double qy, double qz, double qw, double &r, double &p, double &y) {

    double Epsilon = 0.0009765625;
    double Threshold = 0.5 - Epsilon;
    double TEST = qw * qy - qx * qz;
    double sign = 1;
    if (TEST < -Threshold || TEST > Threshold) {
        if (TEST > 0) {
            sign = 1;
        } else if (TEST < 0) {
            sign = -1;
        }

        y = -2 * sign * atan2(qx, qw);
        p = sign * pi / 2.0;
        r = 0;
    } else {
        r = atan2(2 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
        p = asin(-2 * (qx * qz - qw * qy));
        y = atan2(2 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
    }
}

std::vector<std::string> stringSplit(const std::string &str, char delim) {
    std::stringstream ss(str);
    std::string item;
    std::vector<std::string> elems;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
            : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), previous_error_(0.0) {}

    double calculate(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;
        integral_ += error * dt;
        double derivative = (error - previous_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        previous_error_ = error;
        return output;
    }

    double calculate(double error, double dt) {
        integral_ += error * dt;
        double derivative = (error - previous_error_) / dt;
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        previous_error_ = error;
        return output;
    }

private:
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain

    double integral_;         // Integral term
    double previous_error_;   // Previous error
};

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("car_control") {
        // namespece_ = "/scit_drone";
        // node_id_ = 0;
        // //
        // pn0_ = 0;
        // pe0_ = 3 * (node_id_ + 1);
        // pd0_ = -5;
        // yaw0_ = 0;

        this->declare_parameter<std::string>("namespece", "/scout");
        this->declare_parameter<int>("node_id", 1);
        this->declare_parameter<float>("pn0", 0);
        this->declare_parameter<float>("pe0", 0);
        this->declare_parameter<float>("pd0", 0.5);
        this->declare_parameter<float>("yaw0", 0);

        this->get_parameter("namespece", namespece_);
        this->get_parameter("node_id", node_id_);
        this->get_parameter("pn0", pn0_);
        this->get_parameter("pe0", pe0_);
        this->get_parameter("pd0", pd0_);
        this->get_parameter("yaw0", yaw0_);

        suffix_ = namespece_ + std::to_string(node_id_);

        // std::cout << "pid="<<getpid()<<std::endl;

        std::ofstream ofs;
        ofs.open("/home/com/YP/ros_ws/src/NodePID/husky" + std::to_string(node_id_) + ".txt", std::ios::out);
        ofs << getpid() << std::endl;
        ofs.close();

        //===================参数初始化======================
        imu_freq = 5;
        gps_freq = 5;
        camera_color_freq = 5;
        camera_depth_freq = 5;
        camera_infra1_freq = 5;
        camera_infra2_freq = 5;
        lidar_freq = 5;
        state_freq = 100;
        uwb_freq = 5;
        sensor_freq = 100;

        send_seq = 0;
        offboard_setpoint_counter_ = 0;
        state_seq = 0;
        imu_seq = 0;
        color_image_seq = 0;
        depth_image_seq = 0;
        infra1_image_seq = 0;
        infra2_image_seq = 0;
        lidar_seq = 0;
        sensor_seq = 0;

        mission_flag_ = 0;
        path_ini_ID_ = 0;
        path_other_ID_ = 0;

        isFmtSet = false;
        isPathSet = false;

        leader_id = node_id_;
        rx_l = 0;
        ry_l = 0;
        rz_l = 0;

        isposUpdate = false;
        isLeaderPosUpdate = false;
        last_timestamp = 0;
        distController = PIDController(1, 0, 0);
        angleController = PIDController(1, 0, 0);
        //预设路径（地面站设置）
        path_len_ = 6;
        for (int i = 0; i < path_len_; i++) {
            // path_n_[i] = 1 + 5 * i;
            // path_e_[i] = 3 * (node_id_ + 1);
            path_d_[i] = 0.3;
            path_yaw_[i] = 0;

            if (node_id_ < 9 + 1) {
                path_n_[i] = 0.0 + i * 100.0;
                path_e_[i] = 62.5 * (node_id_ - 1);
            } else if (node_id_ < 18 + 1) {
                path_n_[i] = 10.0 + i * 100.0;
                path_e_[i] = 62.5 * (node_id_ - 9 - 1);
            } else if (node_id_ < 27 + 1) {
                path_n_[i] = 20.0 + i * 100.0;
                path_e_[i] = 62.5 * (node_id_ - 18 - 1);
            } else if (node_id_ < 36 + 1) {
                path_n_[i] = 30.0 + i * 100.0;
                path_e_[i] = 62.5 * (node_id_ - 27 - 1);
            } else if (node_id_ < 45 + 1) {
                path_n_[i] = 40.0 + i * 100.0;
                path_e_[i] = 62.5 * (node_id_ - 36 - 1);
            }
        }

        //============Publisher========================
        std::string topic_name1 = suffix_ + "/cmd_vel";
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name1, 10);

        topic_name1 = suffix_ + "/states";
        pub_states_ = this->create_publisher<gtec_msgs::msg::States>(topic_name1, 10);

        topic_name1 = suffix_ + "/sensor_combined";
        pub_sensor_combined_ = this->create_publisher<gtec_msgs::msg::SensorCombined>(topic_name1, 10);

        //============Subscription========================
        // get common timestamp

        std::string topic_name = suffix_ + "/goal";
        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                topic_name, 10, std::bind(&OffboardControl::goal_callback, this, _1));

        topic_name = "/path";
        sub_goal_array_ = this->create_subscription<nav_msgs::msg::Path>(
                topic_name, 10, std::bind(&OffboardControl::goal_array_callback, this, _1));

        topic_name = suffix_ + "/state_ground_truth";
        sub_ground_truth_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_name, 10, std::bind(&OffboardControl::vehicle_ground_truth_callback, this, _1));

        topic_name = suffix_ + "/odom";
        sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
                topic_name, 10, std::bind(&OffboardControl::vehicle_odometry_callback, this, _1));

        topic_name = suffix_ + "/uwb/gtec/toa/ranging";
        sub_uwb_ = this->create_subscription<gtec_msgs::msg::LinktrackNodeframe3>(
                topic_name, 10, std::bind(&OffboardControl::uwb_callback, this, _1));

        topic_name = suffix_ + "/imu/data";
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
                topic_name, 10, std::bind(&OffboardControl::imu_callback, this, _1));

        topic_name = suffix_ + "/gps/data";
        sub_gps_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                topic_name, 10, std::bind(&OffboardControl::gps_callback, this, _1));

        topic_name = "/formation";
        sub_fmt_ = this->create_subscription<gtec_msgs::msg::Formation>(
                topic_name, 10, std::bind(&OffboardControl::fmt_callback, this, _1));

        // topic_name = suffix_ + "/color/image_raw";
        // sub_color_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     topic_name, 10, std::bind(&OffboardControl::color_image_callback, this, _1));

        // topic_name = suffix_ + "/depth/image_raw";
        // sub_depth_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     topic_name, 10, std::bind(&OffboardControl::depth_image_callback, this, _1));

        // topic_name = suffix_ + "/infra1/image_raw";
        // sub_infra1_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     topic_name, 10, std::bind(&OffboardControl::infra1_image_callback, this, _1));

        // topic_name = suffix_ + "/infra2/image_raw";
        // sub_infra2_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        //     topic_name, 10, std::bind(&OffboardControl::infra2_image_callback, this, _1));

        // topic_name = suffix_ + "/velodyne_points";
        // sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     topic_name, 10, std::bind(&OffboardControl::lidar_callback, this, _1));

        // std::vector<rclcpp::TopicEndpointInfo> test;

        // test = this->get_publishers_info_by_topic("/scout1/imu/data");

        // std::cout <<"=========================="<<test[0].topic_type() << std::endl;

        // usleep(200000);


        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


        timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

        // //lcm=====================================
        // if (!lcm.good())
        //     RCLCPP_ERROR(this->get_logger(), "Lcm bad!!!");

        //定义lcm数据发送线程
        //此处改为发送ros2 topic
        std::thread lcmmsg_send(&OffboardControl::lcmmsg_send_callback, this);
        lcmmsg_send.detach();

        //定义lcm数据接受线程
        std::thread lcmmsg_recieve(&OffboardControl::lcmmsg_recieve_callback, this);
        lcmmsg_recieve.detach();
    }

private:
    void timer_callback() {
        // std::cout <<node_id_<< "    " << mission_flag_<< "    " << isPathSet << std::endl;

        double d = sqrt((pn0_ - pn_) * (pn0_ - pn_) + (pe0_ - pe_) * (pe0_ - pe_));
        if (d < 2.0 && isposUpdate) {
            mission_flag_ = 1;
        }

        if (mission_flag_ == 0 && isposUpdate) {
            publish_trajectory_setpoint(pn0_, pe0_, pd_, yaw0_);
        }
        // isPathSet = true;
        if (mission_flag_ == 1 && isLeaderPosUpdate) {
//            double pn_g = path_n_[path_ini_ID_];
//            double pe_g = path_e_[path_ini_ID_];
//            double pd_g = path_d_[path_ini_ID_];
//            double yaw_g = path_yaw_[path_ini_ID_];

            // publish_trajectory_setpoint(pn_g, pe_g, pd_g, yaw_g);

            // std::cout << "pn_g = " << pn_g << std::endl;
            // std::cout << "pe_g = " << pe_g << std::endl;

//            double d = sqrt((pn_g - pn_) * (pn_g - pn_) + (pe_g - pe_) * (pe_g - pe_));
//            if (d < 2.0) {
//                path_ini_ID_ = path_ini_ID_ + 1;
//            }
//            if (path_ini_ID_ > path_len_ - 1) {
//                path_ini_ID_ = 0;
//            }

//            pn_g = path_n_[path_ini_ID_];
//            pe_g = path_e_[path_ini_ID_];
//            pd_g = path_d_[path_ini_ID_];
//            yaw_g = path_yaw_[path_ini_ID_];

            publish_trajectory_setpoint(pn_d, pe_d, pd_d, yaw_d);

            // std::cout << node_id_ << "  " << mission_flag_ << "  " << path_ini_ID_ << "  " << path_n_[path_ini_ID_] << "  " << path_e_[path_ini_ID_] << std::endl;
        }

        // if (node_id_==1 ||node_id_==10 ||node_id_==19 ||node_id_==28 ||node_id_==37)
        // {
        //    std::cout << node_id_ << "  " << mission_flag_ << "  " << path_ini_ID_ << "  " << path_n_[path_ini_ID_] << "  " << path_e_[path_ini_ID_] << std::endl;

        // }

        // // ===================LCM=================
        // ros2::statemsg state_msg;
        // state_msg.seq = state_seq;
        // state_msg.frame_id = "/world";
        // state_msg.RobotType = 3;
        // state_msg.RobotReal = 0;
        // state_msg.RobotID = node_id_;
        // state_msg.time = state_ground_truth.header.stamp.sec + state_ground_truth.header.stamp.sec * 1e9;
        // state_msg.x = state_ground_truth.pose.pose.position.x;
        // state_msg.y = state_ground_truth.pose.pose.position.y;
        // state_msg.z = state_ground_truth.pose.pose.position.z;
        // state_msg.v_x = state_ground_truth.twist.twist.linear.x;
        // state_msg.v_y = state_ground_truth.twist.twist.linear.y;
        // state_msg.v_z = state_ground_truth.twist.twist.linear.z;
        // state_msg.a_x = 0;
        // state_msg.a_y = 0;
        // state_msg.a_z = 0;
        // state_msg.q_x = state_ground_truth.pose.pose.orientation.x;
        // state_msg.q_y = state_ground_truth.pose.pose.orientation.y;
        // state_msg.q_z = state_ground_truth.pose.pose.orientation.z;
        // state_msg.q_w = state_ground_truth.pose.pose.orientation.w;
        // state_msg.w_x = state_ground_truth.twist.twist.angular.x;
        // state_msg.w_y = state_ground_truth.twist.twist.angular.y;
        // state_msg.w_z = state_ground_truth.twist.twist.angular.z;

        // this->lcm.publish("state_msg", &state_msg);
        // // std::cout << "pn = "<< my_data.position[0] << std::endl;
        // // std::cout << "pe = "<< my_data.position[1] << std::endl;
        // // std::cout << "pd = "<< my_data.position[2] << std::endl;
        // // ===================LCM=================

        // state_seq++;
    }

    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.);

        mission_flag_ = 2;
        double pn_g = msg->pose.position.x;
        double pe_g = msg->pose.position.y;
        double pd_g = msg->pose.position.z;

        double roll;
        double pitch;
        double yaw;

        euler_from_quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                              msg->pose.orientation.z, msg->pose.orientation.w,
                              roll, pitch, yaw);

        publish_trajectory_setpoint(pn_g, pe_g, pd_g, yaw);
    }

    // void goal_array_callback(const nav_msgs::msg::Path::SharedPtr msg)
    // {
    //     //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.);
    //     mission_flag_ = 2;
    //     double pn_g = msg->poses[path_other_ID_].pose.position.x;
    //     double pe_g = msg->poses[path_other_ID_].pose.position.y;
    //     double pd_g = msg->poses[path_other_ID_].pose.position.z;

    //     double roll;
    //     double pitch;
    //     double yaw;

    //     euler_from_quaternion(msg->poses[path_other_ID_].pose.orientation.x, msg->poses[path_other_ID_].pose.orientation.y,
    //                           msg->poses[path_other_ID_].pose.orientation.z, msg->poses[path_other_ID_].pose.orientation.w,
    //                           roll, pitch, yaw);

    //     if (abs(pn_g) + abs(pe_g) > 0.00001)
    //     {
    //         publish_trajectory_setpoint(pn_g, pe_g, pd_g, yaw);

    //         double d = sqrt((pn_g - pn_) * (pn_g - pn_) + (pe_g - pe_) * (pe_g - pe_));

    //         if (d < 1.0)
    //         {
    //             path_other_ID_ = path_other_ID_ + 1;
    //         }
    //         if (path_other_ID_ > path_len_ - 1)
    //         {
    //             path_other_ID_ = 0;
    //         }
    //     }
    // }

    void goal_array_callback(const nav_msgs::msg::Path::SharedPtr msg) {

        std::vector<std::string> header_path = stringSplit(msg->header.frame_id, '_');
        int robot_type = atoi(header_path[0].c_str());
        int robot_real = atoi(header_path[1].c_str());
        int robot_leader = atoi(header_path[2].c_str());

        // if (robot_type == 3 && robot_real == 0 && robot_leader == leader_id)
        if (robot_leader == leader_id) {
            for (size_t i = 0; i < msg->poses.size(); i++) {
                if (i < 255) {
                    path_t_[i] = msg->poses[i].header.stamp.sec + msg->poses[i].header.stamp.nanosec / 1e9;
                    path_n_[i] = msg->poses[i].pose.position.x + rx_l;
                    path_e_[i] = msg->poses[i].pose.position.y + ry_l;
                    path_d_[i] = msg->poses[i].pose.position.z + rz_l;

                    double roll;
                    double pitch;
                    double yaw;

                    euler_from_quaternion(msg->poses[i].pose.orientation.x, msg->poses[i].pose.orientation.y,
                                          msg->poses[i].pose.orientation.z, msg->poses[i].pose.orientation.w,
                                          roll, pitch, yaw);
                    path_yaw_[i] = yaw;

                    path_len_ = i + 1;
                }
            }

            isPathSet = true;

        }

        // std::cout << node_id_<<" : "<<robot_type<<"  "<<robot_real<<"  "<<robot_leader<<"  "<<path_t_[0]<<"  "<<path_n_[0]<<"  "<<path_e_[0]<< std::endl;
    }

    void leader_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        pn_d = msg->pose.pose.position.x + rx_l;
        pe_d = msg->pose.pose.position.y + ry_l;
        pd_d = msg->pose.pose.position.z + rz_l;

        double roll;
        double pitch;
        double yaw;

        euler_from_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                              roll, pitch, yaw);
        yaw_d = yaw;
        isLeaderPosUpdate = true;
    }

    void vehicle_ground_truth_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        isposUpdate = true;

        pn_ = msg->pose.pose.position.x;
        pe_ = msg->pose.pose.position.y;
        pd_ = msg->pose.pose.position.z;
        yaw_speed_ = msg->twist.twist.angular.z;

        double roll;
        double pitch;
        double yaw;

        euler_from_quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z, msg->pose.pose.orientation.w,
                              roll, pitch, yaw);
        yaw_ = yaw;
        // std::cout << "pn_ = " << pn_ << std::endl;
        // std::cout << "pe_ = " << pe_ << std::endl;

        state_ground_truth = *msg;

        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = msg->header.stamp;
        // t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "/scout" + std::to_string(node_id_) + "/base_link";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        // tf2::Quaternion q;
        // q.setRPY(0, 0, msg->theta);
        t.transform.rotation.x = msg->pose.pose.orientation.x;
        t.transform.rotation.y = msg->pose.pose.orientation.y;
        t.transform.rotation.z = msg->pose.pose.orientation.z;
        t.transform.rotation.w = msg->pose.pose.orientation.w;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        // std::cout << "pn_ = " << state_ground_truth.pose.pose.position.x << std::endl;
    }

    void vehicle_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        state_odometry = *msg;
        // std::cout << "pn_ = " << state_ground_truth.pose.pose.position.x << std::endl;
    }

    void uwb_callback(const gtec_msgs::msg::LinktrackNodeframe3::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "uwb_msg.id = " << (int)uwb_msg.id << std::endl;
        // std::cout << "uwb_msg.size = " << (int)uwb_msg.nodes.size() << std::endl;

        // lcm msg================================
        uwb_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        uwb_msg_lcm.frame_id = msg->header.frame_id;
        uwb_msg_lcm.RobotType = 3;
        uwb_msg_lcm.RobotReal = 0;
        uwb_msg_lcm.RobotID = node_id_;
        uwb_msg_lcm.sec = msg->header.stamp.sec;
        uwb_msg_lcm.nanosec = msg->header.stamp.nanosec;
        uwb_msg_lcm.role = msg->role;
        uwb_msg_lcm.id = msg->id;
        uwb_msg_lcm.local_time = msg->local_time;
        uwb_msg_lcm.system_time = msg->system_time;
        uwb_msg_lcm.voltage = msg->voltage;
        uwb_msg_lcm.num_nodes = msg->nodes.size();
        uwb_msg_lcm.uwb_id.clear();
        uwb_msg_lcm.dis.clear();
        uwb_msg_lcm.fp_rssi.clear();
        uwb_msg_lcm.rx_rssi.clear();
        for (size_t i = 0; i < msg->nodes.size(); i++) {
            uwb_msg_lcm.uwb_id.push_back(msg->nodes[i].id);
            uwb_msg_lcm.dis.push_back(msg->nodes[i].dis);
            uwb_msg_lcm.fp_rssi.push_back(msg->nodes[i].fp_rssi);
            uwb_msg_lcm.rx_rssi.push_back(msg->nodes[i].rx_rssi);
        }
    }

    void fmt_callback(const gtec_msgs::msg::Formation::SharedPtr msg) {
        // if (msg->robottype == 3 && msg->robotreal == 0)
        {
            for (size_t i = 0; i < msg->num_fmt; i++) {
                if (msg->robotid_list[i] == node_id_) {
                    leader_id = msg->robotid_list[0];
                    rx_l = msg->x[i];
                    ry_l = msg->y[i];
                    rz_l = msg->z[i];
                    if (!isFmtSet) {
                        auto leader_suffix_ = namespece_ + std::to_string(leader_id);
                        auto leader_topic_name = leader_suffix_ + "/state_ground_truth";
                        sub_leader_ground_truth_ = this->create_subscription<nav_msgs::msg::Odometry>(
                                leader_topic_name, 10,
                                std::bind(&OffboardControl::leader_ground_truth_callback, this, _1));
                    }
                    isFmtSet = true;
                    break;
                }
            }
        }

        // std::cout << node_id_<<" : "<<msg->robottype<<"  "<<msg->robotreal<<"  "<<msg->num_fmt<<"  "<<rx_l<<"  "<<ry_l<<"  "<<rz_l<<"  "<<leader_id<<std::endl;

        // fmt_msg = *msg;
        // std::cout << "uwb_msg.id = " << (int)uwb_msg.id << std::endl;
        // std::cout << "uwb_msg.size = " << (int)uwb_msg.nodes.size() << std::endl;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "imu_msg.linear_acceleration.z = " << (float)msg->linear_acceleration.z << std::endl;

        // lcm msg================================
        imu_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        imu_msg_lcm.frame_id = msg->header.frame_id;
        imu_msg_lcm.RobotType = 3;
        imu_msg_lcm.RobotReal = 0;
        imu_msg_lcm.RobotID = node_id_;

        imu_msg_lcm.q_x = msg->orientation.x;
        imu_msg_lcm.q_y = msg->orientation.y;
        imu_msg_lcm.q_z = msg->orientation.z;
        imu_msg_lcm.q_w = msg->orientation.w;

        imu_msg_lcm.v_x = msg->angular_velocity.x;
        imu_msg_lcm.v_y = msg->angular_velocity.y;
        imu_msg_lcm.v_z = msg->angular_velocity.z;

        imu_msg_lcm.a_x = msg->linear_acceleration.x;
        imu_msg_lcm.a_y = msg->linear_acceleration.y;
        imu_msg_lcm.a_z = msg->linear_acceleration.z;

        for (size_t i = 0; i < 9; i++) {
            imu_msg_lcm.orientation_covariance[i] = msg->orientation_covariance[i];
            imu_msg_lcm.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
            imu_msg_lcm.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
        }
        imu_msg_lcm.seq = imu_seq;
        imu_seq++;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {

        gps_data_ = *msg;
    }

    void color_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "msg->data[0] = " << (int)msg->data[0] << std::endl;

        // lcm msg================================

        color_image_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        color_image_msg_lcm.frame_id = msg->header.frame_id;
        color_image_msg_lcm.RobotType = 3;
        color_image_msg_lcm.RobotReal = 0;
        color_image_msg_lcm.RobotID = node_id_;

        color_image_msg_lcm.camera_name = "color";
        color_image_msg_lcm.height = msg->height;
        color_image_msg_lcm.width = msg->width;

        color_image_msg_lcm.encoding = msg->encoding;

        color_image_msg_lcm.is_bigendian = msg->is_bigendian;

        color_image_msg_lcm.step = msg->step;
        /// channel*width
        color_image_msg_lcm.imagesize = msg->data.size();
        // std::cout << "msg->data.size() = " << (int)msg->data.size() << std::endl;
        color_image_msg_lcm.data.clear();
        for (size_t i = 0; i < msg->data.size(); i++) {
            color_image_msg_lcm.data.push_back(msg->data[i]);
        }
        color_image_msg_lcm.seq = color_image_seq;
        color_image_seq++;
    }

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "msg->data[0] = " << (int)msg->data[0] << std::endl;

        // lcm msg================================

        depth_image_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        depth_image_msg_lcm.frame_id = msg->header.frame_id;
        depth_image_msg_lcm.RobotType = 3;
        depth_image_msg_lcm.RobotReal = 0;
        depth_image_msg_lcm.RobotID = node_id_;

        depth_image_msg_lcm.camera_name = "depth";
        depth_image_msg_lcm.height = msg->height;
        depth_image_msg_lcm.width = msg->width;

        depth_image_msg_lcm.encoding = msg->encoding;

        depth_image_msg_lcm.is_bigendian = msg->is_bigendian;

        depth_image_msg_lcm.step = msg->step;
        /// channel*width
        depth_image_msg_lcm.imagesize = msg->data.size();
        // std::cout << "msg->data.size() = " << (int)msg->data.size() << std::endl;
        depth_image_msg_lcm.data.clear();
        for (size_t i = 0; i < msg->data.size(); i++) {
            depth_image_msg_lcm.data.push_back(msg->data[i]);
        }
        depth_image_msg_lcm.seq = depth_image_seq;
        depth_image_seq++;
    }

    void infra1_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "msg->data[0] = " << (int)msg->data[0] << std::endl;

        // lcm msg================================

        infra1_image_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        infra1_image_msg_lcm.frame_id = msg->header.frame_id;
        infra1_image_msg_lcm.RobotType = 3;
        infra1_image_msg_lcm.RobotReal = 0;
        infra1_image_msg_lcm.RobotID = node_id_;

        infra1_image_msg_lcm.camera_name = "infra1";
        infra1_image_msg_lcm.height = msg->height;
        infra1_image_msg_lcm.width = msg->width;

        infra1_image_msg_lcm.encoding = msg->encoding;

        infra1_image_msg_lcm.is_bigendian = msg->is_bigendian;

        infra1_image_msg_lcm.step = msg->step;
        /// channel*width
        infra1_image_msg_lcm.imagesize = msg->data.size();
        // std::cout << "msg->data.size() = " << (int)msg->data.size() << std::endl;
        infra1_image_msg_lcm.data.clear();
        for (size_t i = 0; i < msg->data.size(); i++) {
            infra1_image_msg_lcm.data.push_back(msg->data[i]);
        }
        infra1_image_msg_lcm.seq = infra1_image_seq;
        infra1_image_seq++;
    }

    void infra2_image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "msg->data[0] = " << (int)msg->data[0] << std::endl;

        // lcm msg================================

        infra2_image_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        infra2_image_msg_lcm.frame_id = msg->header.frame_id;
        infra2_image_msg_lcm.RobotType = 3;
        infra2_image_msg_lcm.RobotReal = 0;
        infra2_image_msg_lcm.RobotID = node_id_;

        infra2_image_msg_lcm.camera_name = "infra2";
        infra2_image_msg_lcm.height = msg->height;
        infra2_image_msg_lcm.width = msg->width;

        infra2_image_msg_lcm.encoding = msg->encoding;

        infra2_image_msg_lcm.is_bigendian = msg->is_bigendian;

        infra2_image_msg_lcm.step = msg->step;
        /// channel*width
        infra2_image_msg_lcm.imagesize = msg->data.size();
        // std::cout << "msg->data.size() = " << (int)msg->data.size() << std::endl;
        infra2_image_msg_lcm.data.clear();
        for (size_t i = 0; i < msg->data.size(); i++) {
            infra2_image_msg_lcm.data.push_back(msg->data[i]);
        }
        infra2_image_msg_lcm.seq = infra2_image_seq;
        infra2_image_seq++;
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        // uwb_msg = *msg;

        // std::cout << "imu_msg.linear_acceleration.z = " << (float)msg->linear_acceleration.z << std::endl;

        // lcm msg================================

        lidar_msg_lcm.time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e9;
        lidar_msg_lcm.frame_id = msg->header.frame_id;
        lidar_msg_lcm.RobotType = 3;
        lidar_msg_lcm.RobotReal = 0;
        lidar_msg_lcm.RobotID = node_id_;

        lidar_msg_lcm.height = msg->height;
        lidar_msg_lcm.width = msg->width;

        lidar_msg_lcm.is_bigendian = msg->is_bigendian;

        lidar_msg_lcm.point_step = msg->point_step;
        lidar_msg_lcm.row_step = msg->row_step;
        lidar_msg_lcm.data_size = msg->data.size();

        lidar_msg_lcm.is_dense = msg->is_dense;

        for (size_t i = 0; i < msg->data.size(); i++) {
            lidar_msg_lcm.data.push_back(msg->data[i]);
        }

        lidar_msg_lcm.num_field = msg->fields.size();

        for (size_t i = 0; i < msg->fields.size(); i++) {
            lidar_msg_lcm.datatype.push_back(msg->fields[i].datatype);
            lidar_msg_lcm.name.push_back(msg->fields[i].name);
            lidar_msg_lcm.offset.push_back(msg->fields[i].offset);
            lidar_msg_lcm.count.push_back(msg->fields[i].count);
        }
    }

    //数据发送线程=======================================
    void lcmmsg_send_callback() {
        while (1) {
            // ===================LCM=================
            //自身状态包
            int r_freq = (int) (100 / state_freq);
            if (send_seq % r_freq == 0) {
                // ros2::statemsg state_msg;
                // state_msg.seq = state_seq;
                // state_msg.frame_id = "/world";
                // state_msg.RobotType = 3;
                // state_msg.RobotReal = 0;
                // state_msg.RobotID = node_id_;
                // state_msg.time = state_ground_truth.header.stamp.sec + state_ground_truth.header.stamp.nanosec * 1e9;
                // state_msg.x = state_ground_truth.pose.pose.position.x;
                // state_msg.y = state_ground_truth.pose.pose.position.y;
                // state_msg.z = state_ground_truth.pose.pose.position.z;
                // state_msg.v_x = state_ground_truth.twist.twist.linear.x;
                // state_msg.v_y = state_ground_truth.twist.twist.linear.y;
                // state_msg.v_z = state_ground_truth.twist.twist.linear.z;
                // state_msg.a_x = 0;
                // state_msg.a_y = 0;
                // state_msg.a_z = 0;
                // state_msg.q_x = state_ground_truth.pose.pose.orientation.x;
                // state_msg.q_y = state_ground_truth.pose.pose.orientation.y;
                // state_msg.q_z = state_ground_truth.pose.pose.orientation.z;
                // state_msg.q_w = state_ground_truth.pose.pose.orientation.w;
                // state_msg.w_x = state_ground_truth.twist.twist.angular.x;
                // state_msg.w_y = state_ground_truth.twist.twist.angular.y;
                // state_msg.w_z = state_ground_truth.twist.twist.angular.z;

                // state_msg.voltage = 100.0;
                // state_msg.health = 1;
                // this->lcm.publish("state_msg", &state_msg);
                // std::cout << "state_seq = " << state_seq << std::endl;
                // std::cout << "pe = "<< my_data.position[1] << std::endl;
                // std::cout << "pd = "<< my_data.position[2] << std::endl;
                // ===================LCM=================

                gtec_msgs::msg::States states;
                states.seq = state_seq;
                states.frame_id = "/map";
                states.robottype = 3;
                states.robotreal = 0;
                states.robotid = node_id_;
                states.time = state_ground_truth.header.stamp.sec + state_ground_truth.header.stamp.nanosec / 1e9;
                states.x = state_ground_truth.pose.pose.position.x;
                states.y = state_ground_truth.pose.pose.position.y;
                states.z = state_ground_truth.pose.pose.position.z;
                states.v_x = state_ground_truth.twist.twist.linear.x;
                states.v_y = state_ground_truth.twist.twist.linear.y;
                states.v_z = state_ground_truth.twist.twist.linear.z;
                states.q_x = state_ground_truth.pose.pose.orientation.x;
                states.q_y = state_ground_truth.pose.pose.orientation.y;
                states.q_z = state_ground_truth.pose.pose.orientation.z;
                states.q_w = state_ground_truth.pose.pose.orientation.w;
                states.w_x = state_ground_truth.twist.twist.angular.x;
                states.w_y = state_ground_truth.twist.twist.angular.y;
                states.w_z = state_ground_truth.twist.twist.angular.z;

                states.voltage = 100.0;
                states.health = 1;

                states.fdata1 = 0.0;
                states.fdata2 = 0.0;
                states.fdata3 = 0.0;
                states.fdata4 = 0.0;

                state_seq++;
                // unsigned int sleep_utime = 1000000/state_freq;
                // usleep(sleep_utime);

                pub_states_->publish(states);
            }

            // sensor包
            r_freq = (int) (100 / sensor_freq);
            if (send_seq % r_freq == 0) {
                gtec_msgs::msg::SensorCombined sensor_states;

                sensor_states.seq = sensor_seq;
                sensor_states.frame_id = "/map";
                sensor_states.robottype = 3;
                sensor_states.robotreal = 0;
                sensor_states.robotid = node_id_;
                sensor_states.time =
                        state_ground_truth.header.stamp.sec + state_ground_truth.header.stamp.nanosec / 1e9;

                sensor_states.imu_q_x = imu_msg_lcm.q_x;
                sensor_states.imu_q_y = imu_msg_lcm.q_y;
                sensor_states.imu_q_z = imu_msg_lcm.q_z;
                sensor_states.imu_q_w = imu_msg_lcm.q_w;
                sensor_states.imu_w_x = imu_msg_lcm.v_x;
                sensor_states.imu_w_y = imu_msg_lcm.v_y;
                sensor_states.imu_w_z = imu_msg_lcm.v_z;
                sensor_states.imu_a_x = imu_msg_lcm.a_x;
                sensor_states.imu_a_y = imu_msg_lcm.a_y;
                sensor_states.imu_a_z = imu_msg_lcm.a_z;

                sensor_states.gps_latitude = gps_data_.latitude;
                sensor_states.gps_longitude = gps_data_.longitude;
                sensor_states.gps_altitude = gps_data_.altitude;

                sensor_states.uwb_id = uwb_msg_lcm.id;
                for (size_t i = 0; i < uwb_msg_lcm.num_nodes; i++) {
                    sensor_states.uwb_id_nodes.push_back(uwb_msg_lcm.uwb_id[i]);
                    sensor_states.uwb_dis_nodes.push_back(uwb_msg_lcm.dis[i]);
                }

                sensor_states.magnetic_field_x = 0.0;
                sensor_states.magnetic_field_y = 0.0;
                sensor_states.magnetic_field_z = 0.0;

                sensor_states.fluid_pressure = 0.0;

                sensor_states.mc_x = state_ground_truth.pose.pose.position.x;
                sensor_states.mc_y = state_ground_truth.pose.pose.position.y;
                sensor_states.mc_z = state_ground_truth.pose.pose.position.z;
                sensor_states.mc_q_x = state_ground_truth.pose.pose.orientation.x;
                sensor_states.mc_q_y = state_ground_truth.pose.pose.orientation.y;
                sensor_states.mc_q_z = state_ground_truth.pose.pose.orientation.z;
                sensor_states.mc_q_w = state_ground_truth.pose.pose.orientation.w;
                sensor_states.mc_vx = state_ground_truth.twist.twist.linear.x;
                sensor_states.mc_vy = state_ground_truth.twist.twist.linear.y;
                sensor_states.mc_vz = state_ground_truth.twist.twist.linear.z;
                sensor_states.mc_wx = state_ground_truth.twist.twist.angular.x;
                sensor_states.mc_wy = state_ground_truth.twist.twist.angular.y;
                sensor_states.mc_wz = state_ground_truth.twist.twist.angular.z;
                sensor_states.mc_a_x = imu_msg_lcm.a_x;
                sensor_states.mc_a_y = imu_msg_lcm.a_y;
                sensor_states.mc_a_z = imu_msg_lcm.a_z;
                sensor_states.mc_aw_x = 0;
                sensor_states.mc_aw_y = 0;
                sensor_states.mc_aw_z = 0;

                sensor_states.enc_x = state_odometry.pose.pose.position.x;
                sensor_states.enc_y = state_odometry.pose.pose.position.y;
                sensor_states.enc_z = state_odometry.pose.pose.position.z;
                sensor_states.enc_q_x = state_odometry.pose.pose.orientation.x;
                sensor_states.enc_q_y = state_odometry.pose.pose.orientation.y;
                sensor_states.enc_q_z = state_odometry.pose.pose.orientation.z;
                sensor_states.enc_q_w = state_odometry.pose.pose.orientation.w;
                sensor_states.enc_vx = state_odometry.twist.twist.linear.x;
                sensor_states.enc_vy = state_odometry.twist.twist.linear.y;
                sensor_states.enc_vz = state_odometry.twist.twist.linear.z;
                sensor_states.enc_wx = state_odometry.twist.twist.angular.x;
                sensor_states.enc_wy = state_odometry.twist.twist.angular.y;
                sensor_states.enc_wz = state_odometry.twist.twist.angular.z;

                sensor_seq++;
                // unsigned int sleep_utime = 1000000/state_freq;
                // usleep(sleep_utime);

                pub_sensor_combined_->publish(sensor_states);
            }

            // //----包
            // r_freq = (int)(100/imu_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     this->lcm.publish("imu_msg", &imu_msg_lcm);
            // }

            // r_freq = (int)(100/camera_color_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     if (color_image_msg_lcm.data.size() >=1)
            //     {
            //        std::cout << "color_image_msg_lcm->data[0] = " << (int)color_image_msg_lcm.data[0] << std::endl;
            //     }

            //     this->lcm.publish("color_image_msg", &color_image_msg_lcm);
            // }

            // r_freq = (int)(100/camera_depth_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     if (depth_image_msg_lcm.data.size() >=1)
            //     {
            //        std::cout << "depth_image_msg_lcm->data[0] = " << (int)depth_image_msg_lcm.data[0] << std::endl;
            //     }

            //     this->lcm.publish("depth_image_msg", &depth_image_msg_lcm);
            // }

            // r_freq = (int)(100/camera_infra1_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     if (infra1_image_msg_lcm.data.size() >=1)
            //     {
            //        std::cout << "infra1_image_msg_lcm->data[0] = " << (int)infra1_image_msg_lcm.data[0] << std::endl;
            //     }

            //     this->lcm.publish("infra1_image_msg", &infra1_image_msg_lcm);
            // }

            // r_freq = (int)(100/camera_infra2_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     if (infra2_image_msg_lcm.data.size() >=1)
            //     {
            //        std::cout << "infra2_image_msg_lcm->data[0] = " << (int)infra2_image_msg_lcm.data[0] << std::endl;
            //     }

            //     this->lcm.publish("infra2_image_msg", &infra2_image_msg_lcm);
            // }

            // r_freq = (int)(100/lidar_freq);
            // if (send_seq % r_freq == 0)
            // {
            //     if (lidar_msg_lcm.data.size() >=1)
            //     {
            //        std::cout << "lidar_msg_lcm->data[0] = " << (int)lidar_msg_lcm.data[0] << std::endl;
            //     }

            //     this->lcm.publish("lidar_msg", &lidar_msg_lcm);
            // }

            //----包

            //固定频率100Hz，10ms， 10000us
            usleep(10000);

            send_seq++;

            if (send_seq >= 100) {
                send_seq = 0;
            }
        }
    }

    //数据接收线程===========================================

    void lcmmsg_recieve_callback() {
        //数据频率接收
        this->lcm.subscribe("data_freq", &OffboardControl::recieve_msg_freq, this);

        // test
        // this->lcm.subscribe("state_msg2", &OffboardControl::state_msg_example, this);

        //

        //

        while (0 == lcm.handle()) {
            // Do nothing
        }
    }

    void state_msg_example(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                           const ros2::statemsg *msg) {
        // std::cout << "stat_seq = " << (int)msg->seq << std::endl;
    }

    void recieve_msg_freq(const lcm::ReceiveBuffer *rbuf, const std::string &chan,
                          const ros2::datafreq *msg) {
        // std::cout << "data_freq = " << (int)msg->freq << std::endl;

        if (msg->RobotType == 3 && msg->RobotReal == 0 && msg->RobotID == node_id_) {
            if (msg->data_name == "state") {
                state_freq = msg->freq;
            } else if (msg->data_name == "sensor") {
                sensor_freq = msg->freq;
            }
            // else if (msg->data_name == "camera_color")
            // {
            //     camera_color_freq = msg->freq;
            // }
            // else if (msg->data_name == "camera_depth")
            // {
            //     camera_depth_freq = msg->freq;
            // }
            // else if (msg->data_name == "camera_infra1")
            // {
            //     camera_infra1_freq = msg->freq;
            // }
            // else if (msg->data_name == "camera_infra2")
            // {
            //     camera_infra2_freq = msg->freq;
            // }
            // else if (msg->data_name == "lidar")
            // {
            //     lidar_freq = msg->freq;
            // }
            // else if (msg->data_name == "gps")
            // {
            //     gps_freq = msg->freq;
            // }
            // else if (msg->data_name == "uwb")
            // {
            //     uwb_freq = msg->freq;
            // }
        }
    }

public:
    //命名空间
    std::string namespece_;

    //无人车ID
    int node_id_;
    std::string suffix_;
    rclcpp::TimerBase::SharedPtr timer_;

    //初始位置
    float pn0_;
    float pe0_;
    float pd0_;
    float yaw0_;

    //无人机预设轨迹
    float path_n_[255];
    float path_e_[255];
    float path_d_[255];
    float path_yaw_[255];
    float path_t_[255];
    int path_len_;

    //主机ID
    int leader_id;

    //相对主机的位置
    float rx_l;
    float ry_l;
    float rz_l;

    //无人机位置
    float pn_;
    float pe_;
    float pd_;
    float yaw_;

    float yaw_speed_;

    //位置目标点
    double pn_d;
    double pe_d;
    double pd_d;
    double yaw_d;

    //任务模式 （0，未起飞；1，起飞（跟踪地面站设定轨迹）;2,跟踪其他指令）
    int mission_flag_;

    //地面站路径ID
    int path_ini_ID_;

    //其他路径ID
    int path_other_ID_;

    //无人车状态序列号
    int state_seq;
    int sensor_seq;
    int imu_seq;
    int color_image_seq;
    int depth_image_seq;
    int infra1_image_seq;
    int infra2_image_seq;
    int lidar_seq;

    // lcm msg 发送序列
    int send_seq;

    bool isFmtSet;
    bool isPathSet;

    //位置是否更新
    bool isposUpdate;
    bool isLeaderPosUpdate;

private:
    float last_timestamp;
    PIDController distController;
    PIDController angleController;
    //无人车真值
    nav_msgs::msg::Odometry state_ground_truth;

    //里程计
    nav_msgs::msg::Odometry state_odometry;

    // gps data
    sensor_msgs::msg::NavSatFix gps_data_;

    // uwb
    gtec_msgs::msg::LinktrackNodeframe3 uwb_msg;

    // formation
    gtec_msgs::msg::Formation fmt_msg;

    // uwb lcm
    ros2::uwbmsg uwb_msg_lcm;
    // imu
    sensor_msgs::msg::Imu imu_msg;
    // imu lcm
    ros2::imumsg imu_msg_lcm;

    // image
    sensor_msgs::msg::Image color_image_msg;
    sensor_msgs::msg::Image depth_image_msg;
    sensor_msgs::msg::Image infra1_image_msg;
    sensor_msgs::msg::Image infra2_image_msg;
    // image lcm
    ros2::imagemsg color_image_msg_lcm;
    ros2::imagemsg depth_image_msg_lcm;
    ros2::imagemsg infra1_image_msg_lcm;
    ros2::imagemsg infra2_image_msg_lcm;

    sensor_msgs::msg::PointCloud2 lidar_msg;
    ros2::pointcloud lidar_msg_lcm;

private:
    //============Publisher========================
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

    rclcpp::Publisher<gtec_msgs::msg::States>::SharedPtr pub_states_;

    rclcpp::Publisher<gtec_msgs::msg::SensorCombined>::SharedPtr pub_sensor_combined_;

    //============Subscription========================
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_goal_array_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ground_truth_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_leader_ground_truth_; //
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

    rclcpp::Subscription<gtec_msgs::msg::LinktrackNodeframe3>::SharedPtr sub_uwb_;

    rclcpp::Subscription<gtec_msgs::msg::Formation>::SharedPtr sub_fmt_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infra1_image_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_infra2_image_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

    //============tf========================
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::atomic<uint64_t> timestamp_; //!< common synced timestamped

    uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

    void publish_trajectory_setpoint(float pn, float pe, float pd, float yaw) const;

    void publish_velocity_setpoint(float vx, float wz) const;


public:
    lcm::LCM lcm;

    // LCM数据转发频率
    /// 数据名称，包括：imu,gps,camera_color,camera_depth,camera_infra1,camera_infra2,lidar,state,uwb

    int imu_freq;
    int gps_freq;
    int camera_color_freq;
    int camera_depth_freq;
    int camera_infra1_freq;
    int camera_infra2_freq;
    int lidar_freq;
    int uwb_freq;

    int state_freq;
    int sensor_freq;
};

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint(float pn, float pe, float pd, float yaw) const {
    //和当前目标点的距离
    // double d = sqrt((pn - pn_) * (pn - pn_) + (pe - pe_) * (pe - pe_));

    // double qx = (pn - pn_) / (d + 0.00001);
    // double qy = (pe - pe_) / (d + 0.00001);
    // double vx = qx * 1;
    // double vy = qy * 1;
    // double vcmd = 1.0 * d;

    // vcmd = limvar(vcmd, -2, 2);

    // double yawcmd = atan2(vy, vx);
    // if (yawcmd - yaw_ > pi)
    // {
    //     yawcmd = yawcmd - 2 * pi;
    // }
    // else if (yawcmd - yaw_ < -pi)
    // {
    //     yawcmd = yawcmd + 2 * pi;
    // }
    // if (fabs(yawcmd - yaw_) > pi / 10)
    // {
    //     vcmd = 0;
    // }

    // std::cout << "yaw_ = " << yaw_ << std::endl;
    // std::cout << "yawcmd = " << yawcmd << std::endl;
    // double wz = 0.5 * (yawcmd - yaw_) - 0.1 * yaw_speed_;
    /////////////////////////////////////////////PID Control
    double d;
    double thet;
    auto timestamp = this->get_clock()->now().seconds();
    auto dt = timestamp - last_timestamp;
    last_timestamp = timestamp;
    getdisangle2(pn_, pe_, yaw_, pn, pe, d, thet);
    double vcmd = distController.calculate(d, dt);
    double wz = angleController.calculate(thet, dt);
    /////////////////////////////////////////////
//    double vcmd = 1.0 * d;
    vcmd = limvar(vcmd, -2, 2);
//
//    // std::cout << "d = " << d << std::endl;
//    // std::cout << "thet = " << thet << std::endl;
//
//    double wz = 1.0 * thet - 0.0 * yaw_speed_;
//
    wz = limvar(wz, -1, 1);
//
//    // wz = 1.0;
//    // std::cout << "wz = " << wz << std::endl;
//
//    // if (fabs(thet) > pi / 6)
//    // {
//    //     vcmd = 0.0;
//    // }
//
//    // wz = 1.0;
//    // vcmd = 0.2;
//
//    if (node_id_ < 6) {
//        std::cout << node_id_ << "    " << vcmd << "    " << wz << std::endl;
//    }


    publish_velocity_setpoint(vcmd, wz);

    // std::cout << "vcmd = " << vcmd << std::endl;
    // std::cout << "wz = " << wz << std::endl;
}

void OffboardControl::publish_velocity_setpoint(float vx, float wz) const {

    geometry_msgs::msg::Twist msg;

    msg.linear.x = vx;
    msg.angular.z = wz;

    pub_cmd_vel_->publish(msg);

    // if (node_id_ == 2)
    // {
    //     std::cout << "vx = " << vx << std::endl;
    //     std::cout << "wz = " << wz << std::endl;
    // }
}


int main(int argc, char *argv[]) {
    // std::cout << "Starting offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}
