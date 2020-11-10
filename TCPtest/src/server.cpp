/****************************
 * 文件名：server.cpp
 * 创建人：东北大学-王希哲
 * 描 述：东北大学无人驾驶实验室、TCP/IP、服务端
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
//#include <gps_common/conversions.h>
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
        sub_Yolo = nh.subscribe("/class_and_area", 1, &MMW_lidar::send_LL, this);

        //Set TCP
        std::string interface_tcp_server = "";
        priv_nh.getParam("interface_tcp_server", interface_tcp_server);

        std::string ip_addr_tcp_server = "192.168.8.100";
        priv_nh.getParam("ip_addr_tcp_server", ip_addr_tcp_server);

        int port_tcp_server = 65500;
        priv_nh.getParam("port_tcp_server", port_tcp_server);

        std::string interface_tcp_client = "";
        priv_nh.getParam("interface_tcp_client", interface_tcp_client);

        std::string ip_addr_tcp_client = "192.168.8.101";
        priv_nh.getParam("ip_addr_tcp_client", ip_addr_tcp_client);

        int port_tcp_client = 65500;
        priv_nh.getParam("port_tcp_client", port_tcp_client);

        priv_nh.getParam("pubHz", pubHz);
        countSet = (size_t)(100.0 / pubHz);

        // Variables
        Packet packet;
        sockaddr source;
        bool first = true;

        int fd;
        sockaddr_in sock;
        //Set TCP
        int server_sockfd;
        sockaddr_in server_sock;
        sockaddr_in client_sock;
        bool sendTCPPacket = false;
          char buf[BUFSIZ]; //BUFSIZ system defalut cache size.
        if(openTCPServerSocket(interface_tcp_server,ip_addr_tcp_server,
                               port_tcp_server,&server_sockfd,&server_sock)){
            if(listen(server_sockfd,5) < 0){
                ROS_FATAL("Listen error");
            }else{
                socklen_t sin_size = sizeof (client_sock);

                if((client_sockfd = accept(server_sockfd,(struct sockaddr*)&client_sock, &sin_size)) < 0)
                {
                    ROS_FATAL("Accept error");
                }else{
                    sendTCPPacket= true;
                }
            }
        }else{
            ROS_FATAL("Failed to open socketTCP");
        }
        ROS_INFO("TCP link has been setup!");
    }

    int recive_LL();
    void send_LL(const std_msgs::String::ConstPtr& msg);
    bool openTCPServerSocket(const std::string &interface, const std::string &ip_addr, uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr);
private:
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh;

  ros::Subscriber sub_Yolo ;
  int client_sockfd;
  int count2 = 0;
  int  countSet = 500;
  int good = 0;
};
bool MMW_lidar::openTCPServerSocket(const std::string &interface, const std::string &ip_addr,
                                       uint16_t port, int *fd_ptr, sockaddr_in *sock_ptr)
{
    int fd;
    fd = socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);

    if(fd != -1){
        if (interface.length()) {
          if (!setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.length()) == 0) {
            close(fd);
            return false;
          }
        }

        memset(sock_ptr, 0, sizeof(sockaddr_in));
        sock_ptr->sin_family = AF_INET;
        sock_ptr->sin_port = htons(port);
        if (!inet_aton(ip_addr.c_str(), &sock_ptr->sin_addr)) {
          sock_ptr->sin_addr.s_addr = INADDR_ANY; // Invalid address, use ANY
        }

        if (bind(fd, (sockaddr*)sock_ptr, sizeof(sockaddr)) == 0) {
          *fd_ptr = fd;
          return true;
        }
    }
    return  false;
}
void MMW_lidar::send_LL(const std_msgs::String::ConstPtr& msg)
{
    if(recive_LL()>0)
    {
	std::string result;
	result = msg->data.c_str();
	const char* buffer = result.data();
	if(send(client_sockfd, buffer,strlen(buffer), 0) < 0)//(char*)&buffer, sizeof(buffer)
	{
	    ROS_FATAL("Write error");
	}
    }
}

int MMW_lidar::recive_LL()
{
   char recvBuf[1024];
   memset(recvBuf, 0, sizeof(recvBuf));
   recv(client_sockfd, recvBuf, sizeof(recvBuf), 0);
   return strlen(recvBuf);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MMW_lidar");
  MMW_lidar start_detec;
  ros::spin();
}
