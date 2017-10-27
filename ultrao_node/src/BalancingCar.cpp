#include "ros/ros.h"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include "serial/serial.h"
#include "balancing_car_com/Angles.h"
#include "balancing_car_com/Pid.h"
#include <vector>


struct PIDdata
{
float kp;
float ki;
float kd;
};

using namespace std;

balancing_car_com::Angles fit;

float kp_temp=0,ki_temp=0,kd_temp=0;
float fit_array[1000];
float fit_aim = 0.0;
string head="##",tail="**";
string recv_buf;

vector<PIDdata> pidFIFO;

string ftos(float &temp)
{
  string s,s_temp;
  stringstream ss;
  ss << temp;
  s_temp = ss.str();
  ss.str("");
  if(temp < 10.0 && (s_temp.length()==1))
    {
      s = "00" + s_temp + ".00";
      return s;
    }
  if(temp < 10.0 && (s_temp.length()==3))
    {
      s = "00" + s_temp + "0";
      return s;
    }
  else if(temp < 10.0 && (s_temp.length()==4))
    {
      s = "00" + s_temp;
      return s;
    }
  else if(temp>=10.0 && temp<100.0 && (s_temp.length()==2))
    {
      s = "0" + s_temp +".00";
      return s;
    }
  else if(temp>=10.0 && temp<100.0 && (s_temp.length()==4))
    {
      s = "0" + s_temp +"0";
      return s;
    }
  else if(temp>=10.0 && temp<100.0 && (s_temp.length()==5))
    {
      s = "0" + s_temp;
      return s;
    }
  else if(temp>=100.0 && (s_temp.length()==3))
    {
      s = s_temp + ".00";
      return s;
    }
  else if(temp>=100.0 && (s_temp.length()==5))
    {
      s = s_temp + "0";
      return s;
    }
  else
      return s_temp;
}


void callback(const balancing_car_com::Pid &pid_in)
{
    PIDdata data;
    data.kp = pid_in.kp;
    data.ki = pid_in.ki;
    data.kd = pid_in.kd;
    pidFIFO.push_back(data);
  /*配置串口*/
  /*
  string port("/dev/ttyUSB0");
  unsigned long baud = 38400;
  serial::Serial my_serial(port,baud,serial::Timeout::simpleTimeout(1000));

  stringstream oss;
  string send_buf;
  string kp_str,ki_str,kd_str;
  string start = "##01**";
*/
  /*接收到PID发送开始训练命令*/
  //ROS_INFO("str:%s",send_buf.data());
/*
  oss << start;
  my_serial.write(oss.str());
  ROS_INFO("start:%s",start.c_str());
  oss.str("");

  usleep(100*1000);
  */
  /*浮点数转字符串*/
/*
  kp_temp = pid_in.kp;
  ki_temp = pid_in.ki;
  kd_temp = pid_in.kd;
  kp_str  = ftos(kp_temp);
  ki_str  = ftos(ki_temp);
  kd_str  = ftos(kd_temp);
*/
  /*组帧*/
  //send_buf = head+"02"+"|"+kp_str+"|"+ki_str+"|"+kd_str+tail;
  //ROS_INFO("str:%s",send_buf.data());
  //oss << send_buf;
  /*串口发送*/
  //my_serial.write(oss.str());
  //oss.str("");
  //ROS_INFO("send_buf[%s]",send_buf.c_str());
}


int main(int argc, char **argv)
{
  /*配置串口*/
  string port("/dev/ttyUSB0");
  unsigned long baud = 38400;
  serial::Serial my_serial(port,baud,serial::Timeout::simpleTimeout(1000));

  ////
  bool end_flag = true;
  ////

  int count = 0;
  string num = "-0123456789";

  ////
  PIDdata data;
  pidFIFO.reserve(200);
  ////

  //my_serial.write("##01**");

  //sleep(1);

  //my_serial.write("##02|295.00|000.00|001.48**");


  ros::init(argc, argv, "BalancingCar");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/BalancingCar/pid",100,callback);
  ros::Publisher angle_pub = n.advertise<balancing_car_com::Angles>("/BalancingCar/angles",100);


  ros::Rate loop(10);

  while(ros::ok())
  {
    recv_buf = my_serial.readline(50, "\n");
    stringstream ss;
    const char *ptr = recv_buf.data();
    const int len = recv_buf.length();
    //if(*ptr=='\0')
    //  goto recNULL;
    //ROS_INFO("Rceived :[%s]",ptr);
	//recNULL:
	if((!pidFIFO.empty()) && end_flag)
	{
		end_flag = false;
		data = pidFIFO.front();
		pidFIFO.erase(pidFIFO.begin());
		stringstream oss;
  		string send_buf;
  		string kp_str,ki_str,kd_str;
  		string start = "##01**";

		oss << start;
  		my_serial.write(oss.str());
  		ROS_INFO("start:%s",start.c_str());
  		oss.str("");

		usleep(100*1000);

		kp_temp = data.kp;
  		ki_temp = data.ki;
  		kd_temp = data.kd;
  		kp_str  = ftos(kp_temp);
  		ki_str  = ftos(ki_temp);
  		kd_str  = ftos(kd_temp);

  		send_buf = head+"02"+"|"+kp_str+"|"+ki_str+"|"+kd_str+tail;
  		oss << send_buf;
  		/*串口发送*/
  		my_serial.write(oss.str());
  		oss.str("");
  		ROS_INFO("update_pid[%s]",send_buf.c_str());

	}
#if 0
    else
    {
      if( (len==6) && (*ptr=='#') && (*(ptr+1)=='#') && (*(ptr+len-1)=='*') && (*(ptr+len-2)=='*') )
        {
	  float sum_error = 0.0;
	  for(int i=0; i<count ;i++)
	    {
	      sum_error += fit_array[i]-fit_aim;
	    }
          fit.fitness = sum_error/count;
	  count=0;
	  //memset();
	  angle_pub.publish(fit);
        }
      else if( (*ptr=='#') && (*(ptr+1)=='#') && (*(ptr+len-1)=='*') && (*(ptr+len-2)=='*') )
        {
	  string str_temp;
	  float x;
          str_temp = recv_buf.substr(recv_buf.find_first_of(num),recv_buf.find_last_of(num)-recv_buf.find_first_of(num)+1);
          ss << str_temp;
          ss >> x;
          ss.str("");
          fit_array[count] = x;
          ++count;
        }
      else
	goto recNULL;
    }
#endif

#if 1
    else
    {
          if(*ptr=='\0')
        	goto recNULL;
          ROS_INFO("Rceived :[%s]",ptr);
          if((*ptr=='#') && (*(ptr+1)=='#') && (*(ptr+len-1)=='*') && (*(ptr+len-2)=='*'))
	      {
			  //ROS_INFO("Rceived :[%s]",ptr);
              string str_temp;
		      str_temp = recv_buf.substr(recv_buf.find_first_of(num),recv_buf.find_last_of(num)-recv_buf.find_first_of(num)+1);
              if(str_temp.compare("03") == 0)
              {

		            ROS_INFO("rec end:%s",str_temp.c_str());
		            float sum_error = 0.0;
		            for(int i=0; i<count ;i++)
		            {
		              sum_error += fit_array[i]-fit_aim;
		            }
		            fit.fitness = sum_error/count;
		            if(count<45)
		                fit.fitness=100;
		            count=0;
	          	    //memset();
                    //print("Testing the fit is:"+fit);
		            angle_pub.publish(fit);
		            end_flag = true;
		            usleep(100*1000);
               }
      	  }
          else if( (ptr[0]=='#') && (ptr[1]=='#') && (ptr[len-2]=='*') && (ptr[len-3]=='*') )
          {
            //ROS_INFO("Rceived success!");
		    string str_temp;
		    str_temp = recv_buf.substr(recv_buf.find_first_of(num),recv_buf.find_last_of(num)-recv_buf.find_first_of(num)+1);




                    float x;
		            ss << str_temp;
		            ss >> x;
                    if(x>200.0)
                      x = x-360.0;
                    ROS_INFO("%f",x);
		            ss.str("");
		            fit_array[count] = x;
		            ROS_INFO("fit_array[%d]:%.2f",count,x);
	                ++count;


	      }

         //else
	            //goto recNULL;
    }
#endif

    //angles.Angles.resize(10);
    //for(int i=0;i<10;i++)
    //  angles.Angles[i]=(char)i;
    //angle_pub.publish(angles);
    recNULL:
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
