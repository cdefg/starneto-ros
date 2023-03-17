#include "starneto_node.hpp"
#include "starneto_mems.hpp"
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using std::chrono_literals::ms

Starneto::Starneto(){


    pub_imu = this->create_publisher<starneto_msgs::msg::Gtimu>("GTIMU", 10);
    pub_imu = this->create_publisher<starneto_msgs::msg::Gpfpd>("GPFPD", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&StarnetoNode::timer_callback, this));
}

~Starneto::Starneto(){
    
}

void Starneto::timer_callback(){
    //create msg struct
    struct GPSmsg gps;
    struct IMUmsg imu;
    //publish msg

}