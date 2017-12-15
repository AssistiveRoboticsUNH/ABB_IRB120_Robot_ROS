# ABB_IRB120_Robot_ROS

## Step 1: Downlaod Robot Studio
http://cdn.robotstudio.com/install/RobotStudio_6.06_SP1.zip

## Step 2: Set up robot controller for ROS

Open the  workstation file (ROS_20171103-125232.rsstn)

Under the controller tab, click Backup, Restore Backup, then open the controller folder  (120-800267_BACKUP_2017-12-11)

You will need to change the ip adress in the ROS_sockets.sys file to match the ip address of the robot. If you are running the simulation in robot studio, you will enter the ip adress of the computer running robt studio. If you are conencting to the real robot, you need to connect to the service port (192.168.125.1). The file is located in Home/ROS in the controller folder.

Finally, you need to restart the controller. Go to controller, restart, restart RAPID. If everything was doen correctly, the robot pendant should say it is waiting for connection.

## Step 3: Connect to ROS

Move the abb, abb_experimental, and industrial_core-indigo-devel, folders into your ROS workspace and build it. Then, run the following command:

`roslaunch abb_irb120_support robot_MATLAB_interface_irb120.launch robot_ip:=192.168.125.1`

(Note: the ip address used here needs to match the adress set in the ROS_sockets.sys file).

the pendant should display that a connection was made. 

To interface with the MATLAB program, you need to run the following launch file:

 `roslaunch abb_irb120_support robot_MATLAB_interface_irb120.launch robot_ip:=192.168.125.1`

## Step 4: Run the app1.mlapp file in MATLAB

The MATLAB files aere currently not located on this repository. If you need them, you can request them by email. You may need to change the ip address in the ABB_Robot.m file and the Teleoperator.m file to match that of the computer running ROS.

