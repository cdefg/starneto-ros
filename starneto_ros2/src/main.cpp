#include "starneto_mems.hpp"
#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(void){
    Starneto starneto("/dev/ttyUSB0");

    int count = 0;
    while (true){
        starneto.run();
        count++;
        if (count >= 100){
            count = 0;
            starneto.printGPSMsg();
            starneto.printGPSMsg();
        }

    }
}