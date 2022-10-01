#include <ros/ros.h>
#include "actuator_remote/FiveFloats.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuator_remote");

  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<actuator_remote::FiveFloats> ("remote_controller", 1);
  while(true)
  {
    actuator_remote::FiveFloats msg;

    float input_data;
    std::cout << "Give input (0): ";
    std::cin >> input_data;
    msg.data0 = input_data;

    std::cout << "Give input (1): ";
    std::cin >> input_data;
    msg.data1 = input_data;

    std::cout << "Give input (2): ";
    std::cin >> input_data;
    msg.data2 = input_data;

    std::cout << "Give input (3): ";
    std::cin >> input_data;
    msg.data3 = input_data;

    std::cout << "Give input (4): ";
    std::cin >> input_data;
    msg.data4 = input_data;


    p.publish(msg);

    ros::spinOnce();
  }

  return 0;
}