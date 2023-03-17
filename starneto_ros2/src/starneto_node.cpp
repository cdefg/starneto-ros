#include "starneto_node.hpp"
#include "starneto_mems.hpp"
#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/gtimu.hpp"
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

StarnetoNode::StarnetoNode():Node("starneto_node"){


    pub_imu = this->create_publisher<starneto_msgs::msg::Gtimu>("GTIMU", 10);
    pub_imu = this->create_publisher<starneto_msgs::msg::Gpfpd>("GPFPD", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&StarnetoNode::timer_callback, this));
}

~StarnetoNode::StarnetoNode(){
    
}

void StarnetoNode::timer_callback(){
    //create msg struct
    struct GPSmsg gps;
    struct IMUmsg imu;
    //publish msg
    starneto_msgs::msg::Gtimu msg_imu;
    starneto_msgs::msg::Gpfpd msg_gps;
    msg_imu = getIMU(imu);
    msg_gps = getGPS(gps);

    auto now = chrono::system_clock::now();
    msg_imu.header.stamp = now;
    msg_gps.header.stamp = now;

    pub_imu->publish(msg_imu);
    pub_gps->publish(msg_gps);
}


starneto_msgs::msg::Gtimu getIMU(struct IMUmsg imu){
    starneto_msgs::msg::Gtimu ros_imu;
    ros_imu.gpsweek = imu.gpsweek;
    ros_imu.gpstime = imu.gpstime;
    ros_imu.gx = imu.gx;
    ros_imu.gy = imu.gy;
    ros_imu.gz = imu.gz;
    ros_imu.ax = imu.ax;
    ros_imu.ay = imu.ay;
    ros_imu.az = imu.az;
    ros_imu.tpr = imu.tpr;
    return ros_imu;
}

starneto_msgs::msg::Gpfpd getGPS(struct GPSmsg gps){
    starneto_msgs::msg::Gpfpd ros_gps;
    ros_gps.gpsweek = gps.gpsweek;
    ros_gps.gpstime = gps.gpstime;
    ros_gps.heading = gps.heading;
    ros_gps.pitch = gps.pitch;
    ros_gps.roll = gps.roll;
    ros_gps.lat = gps.lat;
    ros_gps.lon = gps.lon;
    ros_gps.alt = gps.alt;
    ros_gps.vn = gps.vn;
    ros_gps.ve = gps.ve;
    ros_gps.vu = gps.vu;
    ros_gps.baseline = gps.baseline;
    ros_gps.nsv1 = gps.nsv1;
    ros_gps.nsv2 = gps.nsv2;
    ros_gps.status = gps.status;
    return ros_gps;
}

