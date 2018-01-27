#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
using namespace std;

bool zero_orientation_set = false;

bool set_zero_orientation(std_srvs::Empty::Request&,
                          std_srvs::Empty::Response&)
{
  ROS_INFO("Zero Orientation Set.");
  zero_orientation_set = false;
  return true;
}

int main(int argc, char** argv)
{
  std::string port("/dev/ttyUSB0");
  unsigned long baud = 115200;
  serial::Serial ser(port,baud,serial::Timeout::simpleTimeout(1000));
  std::string tf_parent_frame_id;
  std::string tf_frame_id;
  std::string frame_id;
  double time_offset_in_seconds;
  bool broadcast_tf;
  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;
  int data_start;


  ros::init(argc, argv, "mpu6050_serial_to_imu_node");

  ros::NodeHandle private_node_handle("~");
  private_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
  private_node_handle.param<std::string>("tf_parent_frame_id", tf_parent_frame_id, "imu_base");
  private_node_handle.param<std::string>("tf_frame_id", tf_frame_id, "imu_link");
  private_node_handle.param<std::string>("frame_id", frame_id, "imu_link");
  private_node_handle.param<double>("time_offset_in_seconds", time_offset_in_seconds, 0.0);
  private_node_handle.param<bool>("broadcast_tf", broadcast_tf, true);
  private_node_handle.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.01);
  private_node_handle.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.01);
  private_node_handle.param<double>("orientation_stddev", orientation_stddev, 0.01);

  ros::NodeHandle nh("imu");
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("data", 50);


  ros::Rate r(20); // 20 hz

  sensor_msgs::Imu imu;

  imu.linear_acceleration_covariance[0] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[4] = linear_acceleration_stddev;
  imu.linear_acceleration_covariance[8] = linear_acceleration_stddev;

  imu.angular_velocity_covariance[0] = angular_velocity_stddev;
  imu.angular_velocity_covariance[4] = angular_velocity_stddev;
  imu.angular_velocity_covariance[8] = angular_velocity_stddev;

  imu.orientation_covariance[0] = orientation_stddev;
  imu.orientation_covariance[4] = orientation_stddev;
  imu.orientation_covariance[8] = orientation_stddev;


  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0,0,0));

  std::string input;
  std::string read;

  //ofstream outfile;
  //std::string rec_buffer;

  while(ros::ok())
  {

    if(ser.isOpen())
       {
           if(ser.available())
               {
                   read = ser.read(ser.available());
                   input += read;
                   while (input.length() >= 59)
                       {
                       data_start = input.find(':');
                       if (data_start != std::string::npos)
                           {
                           if ((input.length() >= data_start + 59)  && (input.compare(data_start + 57, 2, "\r\n")  == 0))
                              {
                              int16_t w = ((0xff &(char)input[data_start + 36]) << 8) | 0xff &(char) input[data_start + 35];
                              int16_t x =  ((0xff &(char)input[data_start + 38]) << 8) | 0xff &(char) input[data_start + 37];
                              int16_t y =  ((0xff &(char)input[data_start + 40]) << 8) | 0xff &(char) input[data_start + 39];
                              int16_t z =  ((0xff &(char)input[data_start + 42]) << 8) | 0xff &(char) input[data_start + 41];
                ROS_INFO("quaternion data: %x, %c ", input[data_start + 36], input[data_start + 35]);
                             double wf = w * 0.001;
                             double xf = x * 0.001;
                             double yf = y * 0.001;
                             double zf = z * 0.001;

                tf::Quaternion orientation(xf, yf, zf, wf);


                // get gyro values
                int16_t gx =  ((0xff &(char)input[data_start + 12]) << 8) | 0xff & (char)input[data_start + 11];
                int16_t gy =  ((0xff &(char)input[data_start + 14]) << 8) | 0xff & (char)input[data_start + 13];
                int16_t gz =  ((0xff &(char)input[data_start + 16]) << 8) | 0xff & (char)input[data_start + 15];
                double gxf = gx * 0.001;
                double gyf = gy * 0.001;
                double gzf = gz * 0.001;

                // get acelerometer values
                int16_t ax =  ((0xff &(char)input[data_start + 18]) << 8) | 0xff &(char)input[data_start + 17];
                int16_t ay =  ((0xff &(char)input[data_start + 20]) << 8) |0xff &(char) input[data_start + 19];
                int16_t az =  ((0xff &(char)input[data_start + 22]) << 8) | 0xff &(char)input[data_start + 21];
                // calculate accelerations in m/s²
                double axf = ax * 0.001 * 9.81;
                double ayf = ay * 0.001 * 9.81;
                double azf = az * 0.001 * 9.81;

                int16_t yaw = ((0xff &(char)input[data_start + 46]) << 8) | 0xff &(char)input[data_start + 45];
                ROS_INFO("yaw: %d", yaw);
              
                // calculate measurement time
                ros::Time measurement_time = ros::Time::now() ;

                // publish imu message
                imu.header.stamp = measurement_time;
                imu.header.frame_id = frame_id;


                imu.angular_velocity.x = gxf;
                imu.angular_velocity.y = gyf;
                imu.angular_velocity.z = gzf;

                imu.linear_acceleration.x = axf;
                imu.linear_acceleration.y = ayf;
                imu.linear_acceleration.z = azf;

                imu.orientation.x = orientation[0];
                imu.orientation.y = orientation[1];
                imu.orientation.z = orientation[2];
                imu.orientation.w = orientation[3];

                imu_pub.publish(imu);
    //outfile << gxf<<' '<<gyf<<' '<<gzf<<' '<<endl;
    }
    }
    }
    }
    }


    //outfile.close();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
