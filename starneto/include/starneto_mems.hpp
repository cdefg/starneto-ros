#ifndef STARNETO_MEMS_HPP
#define STARNETO_MEMS_HPP

#include "starneto_msgs/msg/gtimu.hpp"
#include "starneto_msgs/msg/gpfpd.hpp"
#include "starneto_msgs/msg/pos320_nav.hpp"

#include "serial/serial.h"

#include <rclcpp/rclcpp.hpp>

class Starneto {

 public:

  // Constructor
  Starneto();
  int getNodeRate() const;

  // Method
  //// ROS Method
  void initSerial(const std::string);

  //// Serial Method
  void run();
  void initState();
  void readSerial();
  int  analyzeProtocol();
  void analyzeGpfpd();
  void analyzeGtimu();
  void runAlgorithm();
  void fillMsgHead(rclcpp::Time);

  starneto_msgs::msg::Gpfpd getGpfpd();
  starneto_msgs::msg::Gtimu getGtimu();
 
 private:
  char OneFrame[200];   // one frame data
  unsigned char rbuf[1000];  // 接收缓冲区
  int numinbuf;     // buffer num
  int numgetted;    // num get
  int protocolFlag; // 0:Unknown 1:GPFPD 2:GTIMU
  int CntDelimiter;//分隔符计数
  int PosDelimiter[15];//用于记录分隔符位置
  int field_len[15];//字符串长度
  char temp_field[30];
  char str[3];
  unsigned int tmpint;
  int cscomputed;//计算得到的校验，除去$*hh<CR><LF>共6个字符
  int csreceived;//接收到的校验
  char strtemp[3];
  double delta_t;
  double gpstime_pre;//上一个gps时间

  int GPFPD_STATE_PARSER;
  int GTIMU_STATE_PARSER;

  const int UNKNOWN_PROTOCOL = 0;
  const int GPFPD_ENABLE = 1;
  const int GTIMU_ENABLE = 2;

  std::string serial_port;

  serial::Serial ser;
  starneto_msgs::msg::Gpfpd gnss;
  starneto_msgs::msg::Gtimu imu;
};
  


  


#endif //STARNETO_MEMS_HPP
