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
