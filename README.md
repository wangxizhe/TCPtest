# TCPtest
Writing method of TCP / IP communication on ROS platform  
## installation method
mkdir -p ~/TCPtest/src  
cd TCPtest/src  
git clone https://github.com/wangxizhe/TCPtest.git  
cd ..  
catkin_make
## operation method
source devel/setup.bash  
### server.cpp  
roslaunch TCPtest server.launch  
### client.cpp  
roslaunch TCPtest client.launch
