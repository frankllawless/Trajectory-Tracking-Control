########Optitrack trajectory tracking commands###########
roslaunch vrpn_OptiTrack vrpn_start.launch
#adjust in launch file:
	"server" default="{IP_OF_SERVER}"


rosrun optitrack_trajectory_tracking experiment_{NUMBER OF EXPERIMENT}

press space 
adjust velocity with up and down arrow keys
press space

adjust velocity 
press space

#completed

#################Some basic ROS commands#################
#create a ROS package 
catkin_create_pkg "package name" std_msgs rospy
catkin_make 
source devel/setup.bash

#make a launch file
<launch>
  <node name="" pkg="" type=".py" output="screen"/>
  <node name="" pkg="" type=".py" output="screen"/>
</launch>

#Connect to a TurtleBot3
on Raspberry Pi:
	username = ubuntu
	password = turtlebot
	
	#update Host and Raspberry Pi IP 
	nano ~/.bashrc
	
	#update wifi connection
	cd /etc/netplan
	sudo nano 50-cloud-init.yaml
	#set static IP
	dhcp4: no
	addresses: [xxx.xxx.xxx.xxx/24] #in 50-cloud-init.yaml add this line where xxx.xxx.xxx.xxx is your static IP (follow the IP labeling convention in the Lab)
	
on Host Machine:
	#connect to TurtleBot and initialize subscribers
	ssh ubuntu@{Raspberry Pi IP}
	roslaunch turtlebot3_bringup turtlebot3.robot.launch
	
	#scan for all wifi connections
	sudo arp-scan -l
