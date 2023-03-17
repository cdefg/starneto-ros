#ifndef STARNETO_MEMS_HPP_
#define STARNETO_MEMS_HPP_

#include "serial/serial.h"

// definitions for int32, float64, float32, int16, uint8
typedef int int32;
typedef double float64;
typedef float float32;
typedef short int16;
typedef unsigned char uint8;

struct GPSMsg{
    int32   gpsweek;
    float64 gpstime;
    float32 heading;
    float32 pitch;
    float32 roll;
    float64 lat;
    float64 lon;
    float64 alt;
    float32 ve;
    float32 vn;
    float32 vu;
    float32 baseline;
    int16   nsv1;
    int16   nsv2;
    uint8   status;
};

struct IMUMsg{
    int32   gpsweek;
    float64 gpstime;
    float64 gx;
    float64 gy;
    float64 gz;
    float64 ax;
    float64 ay;
    float64 az;
    float32 tpr; 
};

class Starneto
{
    private:
        std::string serial_port;

        char OneFrame[200];
        unsigned char rbuf[1000];
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

        serial::Serial ser;

        GPSMsg gnss;
        IMUMsg imu;

    public:
        // initialize functions
        Starneto(std::string serial_port_name);
        ~Starneto();
        void initSerial(const std::string serial_port);

        // run functions
        void run();
        void initState();
        void readSerial();
        void runAlgorithm();

        // util functions
        unsigned int GetXorChecksum(const char *pch, int len);
        double DegMin2Deg(double dddmmpmmmm);

        // output functions
        void printGPSMsg();
        void printIMUMsg();
};


#endif // STARNETO_MEMS_HPP_

