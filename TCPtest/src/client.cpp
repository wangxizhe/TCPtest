/****************************
 * 文件名：client.cpp
 * 创建人：东北大学-王希哲
 * 描 述：东北大学无人驾驶实验室、TCP/IP、客户端
 * 日 期：2020-11-5
 * 版 本：1.0.0
 ***************************/
#include "stdlib.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <pcl/filters/passthrough.h>

#include <math.h>
#include "std_msgs/String.h"            //ros定义的String数据类型
#include <stdlib.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>

////TCP
// ROS
#include <ros/ros.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
// ROS messages
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include<string>
// Tf
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
// UTM conversion

// Ethernet
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iconv.h>
#include "dispatch.h"
// UINT16_MAX is not defined by default in Ubuntu Saucy
#ifndef UINT16_MAX
#define UINT16_MAX (65535)
#endif
using namespace Eigen;
using namespace std;
static double pubHz = 1.0;

using namespace std;

class MMW_lidar
{
public:
    MMW_lidar()
    {
        socket_fd = socket(AF_INET, SOCK_STREAM,0);
           if(socket_fd == -1)
           {
               cout<<"socket 创建失败："<<endl;
               exit(-1);
           }

           struct sockaddr_in addr;
           addr.sin_family = AF_INET;
           addr.sin_port = htons(65500);
           addr.sin_addr.s_addr = inet_addr("192.168.3.100");

           int res = connect(socket_fd,(struct sockaddr*)&addr,sizeof(addr));
           if(res == -1)
           {
               cout<<"bind 链接失败："<<endl;
               exit(-1);
           }
           cout<<"bind 链接成功："<<endl;
        
    }

    void recive_LL();
    void send_LL();
    bool openTCPServerSocket(const std::string &interface, const std::string &ip_addr, uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr);
private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;
  int client_sockfd;
  int count2 = 0;
  int  countSet = 500;
  int good = 0;
int socket_fd;
};
void MMW_lidar::recive_LL()
{
   write(socket_fd, "OK", 2);
   char buffer[1024]={};
   int size = read(socket_fd, buffer, sizeof(buffer));//通过fd与客户端联系在一起,返回接收到的字节数
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MMW_lidar");
  MMW_lidar start_detec;
  while(ros::ok())
  {
     start_detec.recive_LL();
  }
}
