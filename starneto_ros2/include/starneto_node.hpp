#ifndef STARNETO_NODE_HPP_
#define STARNETO_NODE_HPP_

#include "starneto_mems.hpp"

#include "rclcpp/rclcpp.hpp"
#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gtimu.hpp"

class StarnetoNode : public rclcpp::Node{
    public:
        StarnetoNode();


    private:
        void timer_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<starneto_msgs::msg::Gtimu>::SharedPtr pub_imu_;
        rclcpp::Publisher<starneto_msgs::msg::Gpfpd>::SharedPtr pub_gnss_;

        std::shared_ptr<Starneto> nav_;
};

starneto_msgs::msg::Gtimu getIMU(struct IMUMsg imu);
starneto_msgs::msg::Gpfpd getGPS(struct GPSMsg gps);


#endif //STARNETO_NODE_HPP_