#include <ros/ros.h>
#include <simple_layers/InfraredData.h>
#include "simple_layers/UltrasoundData.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "public_sencor");
  ros::NodeHandle nh;
  

  ros::Publisher ul_pub = nh.advertise<simple_layers::UltrasoundData>("ultrasound",10);
 
  while(nh.ok())
  {
      ros::Time tmp_t = ros::Time::now();
      
      simple_layers::UltrasoundData tmp;
      tmp.header.frame_id = "ultra_centor";
      tmp.header.stamp = tmp_t;
      
      tmp.frame_id.push_back("ultra_left_1");
      tmp.frame_id.push_back("ultra_before_1");
      tmp.frame_id.push_back("ultra_before_2");
      tmp.frame_id.push_back("ultra_before_3");
      tmp.frame_id.push_back("ultra_before_4");
      tmp.frame_id.push_back("ultra_right_1");
      
      geometry_msgs::Point32 a;
      for(int i = 0; i < 6; i++)
      {
	a.x += 0.1;
	a.y = 0.00;
	a.z = 0.00;
	tmp.value.push_back(a);
      }
      ul_pub.publish(tmp);
      sleep(1);
      
      std::cout << "public sensor" << std::endl;
  }
  
  
  return 0;
}
