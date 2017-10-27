#include <ros/ros.h> 
#include <string>
#include "simple_layers/UltrasoundData.h"
#include "simple_layers/InfraredData.h"
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h>

#define NUMBER_OF_ULTRASONIC 6
#define CENTER_OF_MUL_SENSOR "ultra_centor"

std::string ultrasonic_id[NUMBER_OF_ULTRASONIC] = {
  "ultra_left_1", 
  "ultra_before_1",
  "ultra_before_2",
  "ultra_before_3",
  "ultra_before_4",
  "ultra_right_1"
};

//协议解析
bool protocolParsing(std::string& data, std::vector<double>& result)
{
  const char* data_c = data.c_str();
  uint32_t len = data.length();
  
  
  if(*data_c == '\0')
    return false;
    //goto ERR0;
  
  
  //判断头尾标志位
  if( !((*data_c=='#') && (*(data_c+1)=='#') && (*(data_c+len-1)=='*') && (*(data_c+len-2)=='*')) )
    return false;
  
  ROS_INFO("protocolParsing::received data--> %s",data_c); 
  
  data_c += 2;//从实际数据开始
  std::string tmp;
  while('*' != *data_c)
  {
   //按照逗号为间隔截断数据，最后一个数据通过第二个条件执行stod函数
   if(',' == *data_c) 
   {
     //将字符转换成double并插入到vector中
     result.push_back( std::stod(tmp.c_str()) );
     tmp.clear();
     data_c += 1;
     continue;
   }
   tmp += *data_c;
   data_c += 1;
  }
  
  //最后一个数据
  result.push_back( std::stod(tmp.c_str()) );
  
  return true;
//ERR0:
//  return false;
}

//在进行转换格式之前，确保ros_data数据为空
void rosMessagePacket(std::vector<double>& data, const std::string& frame_id, simple_layers::UltrasoundData& ros_data)
{
    //确保ros_data数据大小为0
    ros_data.frame_id.clear();
    ros_data.value.clear();
    
    geometry_msgs::Point32 format_data;
    ros_data.header.frame_id = frame_id;
    ros_data.header.stamp = ros::Time::now();
    
    uint8_t a = 0;
    std::cout << "data is size: " << data.size() << std::endl;
    for(auto i = data.begin(); i !=  data.end(); i++,a++)
    {
      
      //std::cout << "调试---" << a << "----------------" << std::endl;
      //将厘米转换成米
      format_data.x = data[a] / 100;
      format_data.y = 0.0;
      format_data.z = 0.0;
      ros_data.frame_id.push_back(ultrasonic_id[a]); 
      ros_data.value.push_back(format_data);
    }
}
  
  
int main (int argc, char** argv) 
{ 
  //初始化节点 
  ros::init(argc, argv, "serial_node"); 
  ros::NodeHandle nh; 
  ros::Publisher ultra_pb_g = nh.advertise<simple_layers::UltrasoundData>("ultrasound",1000);
  
  std::vector<double> ultrao_data;
  simple_layers::UltrasoundData ros_data;
  
 
  serial::Serial ser; //声明串口对象 
  try 
  { 
    //设置串口属性，并打开串口 
    ser.setPort("/dev/ttyUSB0"); 
    ser.setBaudrate(9600); 
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
    ser.setTimeout(to); 
    ser.open(); 
  } 
  catch (serial::IOException& e) 
  { 
    ROS_ERROR_STREAM("Unable to open port "); 
    return -1; 
  } 
  
  //检测串口是否已经打开，并给出提示信息 
  if(ser.isOpen()) 
  { 
    ROS_INFO_STREAM("Serial Port initialized"); 
  } 
  else 
  { 
    return -1; 
  } 
  
  //指定循环的频率 
  ros::Rate loop_rate(5); 
  while(ros::ok()) 
  { 
    
    if(ser.available()){ 
      //ROS_INFO_STREAM("Reading from serial port\n"); 
 
      std::string result = ser.read(ser.available()).data(); 
      //ROS_INFO_STREAM("Read: " << result); 
      
      //解析字符串
      protocolParsing(result, ultrao_data);
      //将距离数据转换ros
      rosMessagePacket(ultrao_data,CENTER_OF_MUL_SENSOR,ros_data); 
      
      ultrao_data.clear(); //转换完成后确保数据缓冲为空
      
      ultra_pb_g.publish(ros_data);
      
    } 
    //处理ROS的信息，比如订阅消息,并调用回调函数 
    ros::spinOnce(); 
    loop_rate.sleep(); 
    
  } 
} 
