#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_datatypes.h>
#include <iostream>

using namespace std;
string rec_buffer;


int main(int argc, char** argv)
{
  std::string port("/dev/ttyACM0");
  unsigned long baud = 115200;
  serial::Serial ser(port,baud,serial::Timeout::simpleTimeout(1000));

  ros::init(argc, argv, "mpu6050_serial_to_imu_node");

 /* ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyACM0");
*/
  ros::NodeHandle nh("yaw");
  ros::Publisher imu_pub = nh.advertise<geometry_msgs::Vector3>("yaw_data", 5);

  ros::Rate r(50); // 200 hz

  geometry_msgs::Vector3 imu_yaw;

  std::string yaw_str;
  float yaw;

  while(ros::ok())
  {
    
     rec_buffer = ser.readline(26,"\n");
     const char *receive_data = rec_buffer.data();
     if(rec_buffer.length()>=22)
       {
          yaw_str.assign(rec_buffer, 4,8);
          const char *receive_yaw = yaw_str.data();
          ROS_INFO("receive data: %s", receive_data);
          
          yaw = atof(receive_yaw);
          imu_yaw.z = yaw * 3.1415926 /180;
          ROS_INFO("Yaw: %f", yaw);

          imu_pub.publish(imu_yaw);
       }
    ros::spinOnce();
    r.sleep();
  }
}
