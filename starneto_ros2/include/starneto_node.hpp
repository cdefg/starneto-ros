#ifndef STARNETO_NODE_HPP_
#define STARNETO_NODE_HPP_

#include "starneto_mems.hpp"
#include "rclcpp/rclcpp.hpp"
#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gtimu.hpp"

class StarnetoNode : public rclcpp::Node{
    public:
        StarnetoNode();
        ~StarnetoNode();


    private:
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<starneto_msgs::msg::Gtimu> pub_imu;
        rclcpp::Publisher<starneto_msgs::msg::Gpfpd> pub_gnss;


}


#endif //STARNETO_NODE_HPP_