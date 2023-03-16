#include "rclcpp/rclcpp.hpp"

#include <slcurses.h>
#include "starneto_mems.hpp"
#include <iostream>
#include <memory>
#include <functional>
#include <vector>

#include <chrono>

using namespace std::chrono_literals;

class NavPublisher : public rclcpp::Node
{
  public:
    NavPublisher()
    : Node("nav_publisher")
    {
      system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

      // this->set_all_parameters();
      serial_port = "/dev/ttyUSB0";
      navigator = std::make_shared<Starneto>();
      navigator->initSerial(serial_port);

      std::cout << "[+] setting up publishers" << std::endl;
      pub_gpfpd  = this->create_publisher<starneto_msgs::msg::Gpfpd>("GPFPD", 10);
      pub_gtimu  = this->create_publisher<starneto_msgs::msg::Gtimu>("GTIMU", 10);
      // pub_pos320 = this->create_publisher<starneto_msgs::msg::Pos320Nav>("Pos320Nav", 10);
      // auto freq = navigator->getNodeRate();
      // hardcoding now
      timer_ = this->create_wall_timer(
        10ms, std::bind(&NavPublisher::timer_callback, this));
    }

  private:
    // ROS node items
    std::shared_ptr<Starneto> navigator;

    rclcpp::Publisher<starneto_msgs::msg::Gpfpd>::SharedPtr pub_gpfpd;
    rclcpp::Publisher<starneto_msgs::msg::Gtimu>::SharedPtr pub_gtimu;
    // rclcpp::Publisher<starneto_msgs::msg::Pos320Nav>::SharedPtr pub_pos320;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();
    // void set_all_parameters();

    std::string serial_port;
    std::string GPFPD_output_topic;
    std::string GTIMU_output_topic;
    int node_rate;

    std::shared_ptr<rclcpp::Clock> system_clock;
};

void NavPublisher::timer_callback(){
  this->navigator->run();
  rclcpp::Time now = system_clock->now();
  this->navigator->fillMsgHead(now);
  auto gps = this->navigator->getGpfpd();
  auto imu = this->navigator->getGtimu();

  pub_gpfpd->publish(gps);
  pub_gtimu->publish(imu);
}

// void NavPublisher::set_all_parameters(){
//   std::vector<rclcpp::Parameter> all_new_parameters{
//       rclcpp::Parameter("node_rate", 100),
//       rclcpp::Parameter("serial_port", "/dev/ttyUSB0"),
//       rclcpp::Parameter("GPFPD_output_topic", "/GPFPD"),
//       rclcpp::Parameter("GTIMU_output_topic", "/GTIMU"),
//   };
//   this->set_parameters(all_new_parameters);
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavPublisher>());
  rclcpp::shutdown();
  return 0;
}