#include <ros/ros.h>
#include "actuator_remote/SeveralDatas.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "actuator_remote");

  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<actuator_remote::SeveralDatas> ("remote_controller", 1);
  while(true)
  {
    actuator_remote::SeveralDatas msg;

    float f_input_data;
    int i_input_data;
    
    std::cout << "Give input (0): ";
    std::cin >> f_input_data;
    msg.data0 = f_input_data;

    std::cout << "Give input (1): ";
    std::cin >> f_input_data;
    msg.data1 = f_input_data;

    std::cout << "Give input (2): ";
    std::cin >> f_input_data;
    msg.data2 = f_input_data;

    std::cout << "Give input (3): ";
    std::cin >> f_input_data;
    msg.data3 = f_input_data;

    std::cout << "Give input (4): ";
    std::cin >> f_input_data;
    msg.data4 = f_input_data;
    
    std::cout << "Give int data : ";
    std::cin >> i_input_data;
    msg.int_data = i_input_data;


    p.publish(msg);

    ros::spinOnce();
  }

  return 0;
}