# ABB_IRB120_Robot_ROS
To interface with the MATLAB program, you need to run the following launch file:

 `roslaunch abb_irb120_support robot_MATLAB_interface_irb120.launch robot_ip:=192.168.125.1`
 
The robot ip is specific to the robot controller. If you are running the simulation in robot studio, you will enter the ip adress of the computer running robt studio. If you are conencting to the real robot, you need to connect to the service port (192.168.125.1).

The folder 120-800267_BACKUP_2017-12-11 under Robot Studio contains the controller that interacts with ROS. 
It is important that you chnage the ip adress in the ROS_sockets.sys file to match the ip adress used above. The file is located in Home/ROS in the controller folder. 

The ROS_20171103-125232.rsstn file under Robot Studio contains the workstation with the ABB IRB 120 robot.
