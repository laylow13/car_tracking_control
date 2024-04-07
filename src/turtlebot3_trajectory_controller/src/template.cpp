//
// Created by lay on 23-9-4.
//
#include "rclcpp/rclcpp.hpp"

class Temp : public rclcpp::Node {
    Temp() : Node("test") {
        this->declare_parameter("test_param", 1);

    }
};



int main(int argc, char *argv[]) {
}