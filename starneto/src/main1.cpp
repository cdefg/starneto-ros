#include "rclcpp/rclcpp.hpp"

#include <slcurses.h>
#include "starneto_mems.hpp"

typedef ns_starneto_mems::Starneto Starneto;

class NavPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("nav_publisher"), count_(0)
    {
      navigator = std::make_unique<Starneto>();
      navigator->loadParameters();
      navigator->initSerial();
      pub_gpfpd  = this->create_publisher<starneto_msgs::msg::Gpfpd>("GPFPD", 10);
      pub_gtimu  = this->create_publisher<starneto_msgs::msg::Gtimu>("GTIMU", 10);
      // pub_pos320 = this->create_publisher<starneto_msgs::msg::Pos320Nav>("Pos320Nav", 10);
    }

  private:
    // ROS node items
    rclcpp::Publisher<starneto_msgs::msg::Gpfpd>::SharedPtr pub_gpfpd;
    rclcpp::Publisher<starneto_msgs::msg::Gtimu>::SharedPtr pub_gtimu;
    // rclcpp::Publisher<starneto_msgs::msg::Pos320Nav>::SharedPtr pub_pos320;

    // serial class instance
    Starneto* navigator;
};

void Starneto::loadParameters() {
    std::cout << ("loading handle parameters") << std::endl;
    if (!nodeHandle.param<int>("node_rate", node_rate, 1)) {
        std::cout << ("Did not load node_rate.") << std::endl;
    }
    if (!nodeHandle.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0")) {
        std::cout << ("Did not load serial_port.") << std::endl;
    }
    if (!nodeHandle.param<std::string>("GPFPD_output_topic", GPFPD_output_topic, "/GPFPD")) {
        std::cout << ("Did not load GPFPD_output_topic.") << std::endl;
    }
    if (!nodeHandle.param<std::string>("GTIMU_output_topic", GTIMU_output_topic, "/GTIMU")) {
        std::cout << ("Did not load GTIMU_output_topic.") << std::endl;
    }
}

Starneto::Starneto(){
    loadParameters();
    publishToTopics();
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Rate rate(navigator.getNodeRate());

  NavPublisher nav_publisher();

  while (rclcpp::ok()){
    navigator.run();
    navigator.sendMsg();
    rclcpp::spinOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}