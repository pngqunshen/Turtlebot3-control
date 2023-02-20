#export ROS_MASTER_URI=http://$1:11311
#export ROS_HOSTNAME=$2

export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

source devel/setup.bash
roslaunch ee4308_bringup robot.launch
