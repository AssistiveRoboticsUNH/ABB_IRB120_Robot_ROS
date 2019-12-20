classdef Teleoperator < handle
    properties (Constant)
        % ROBOT PHYSICAL PROPERTIES (SI units)
        Name = 'IMUs'
    end
    
    properties(SetAccess = public)
        % CLASS PUBLIC PROPERTIES
        % messages
        datavec 
        % placeholder variables
        Position
        PositionL
        PositionU
        L1
        L2
        calibrate
        Quat
        Listening
        % subscriber
        listen_sub
        upper_arm_sub
        lower_arm_sub
        hand_sub
        %IP adresses
        robot_ip 
        my_ip 
    end
    
    methods(Static)
        function ROS_CONNECT(~,robot_ip, my_ip)
            % initiates connection between MATLAB and ROS
        rosinit(robot_ip,'NodeHost', my_ip,'NodeName','/MATLAB_node')
        end
        
    end
    
    methods

        function obj = Teleoperator()
        % Define a teleoperator object with the following:
        % telop = Teleoperator()
        % This will try to connect to ROS if it is not already connected. 
        % It will subscribe to /r_upper_arm_data_vec, /r_lower_arm_data_vec, and /r_hand_data_vec.
        % You will likely need to add these as custom messages in MATLAB.
        % Use the following tutorial:
        % https://www.mathworks.com/help/robotics/ug/create-custom-messages-from-ros-package.html
        % or run the following command         
        %           rosgenmsg(pwd)
        % edit the .txt as directed, then add and save the path with the following:         
        %           addpath([pwd '\matlab_gen\msggen'])
        %           savepath
        % If those topics are not published yet, you will get an error. 
        % When calibration is called, the user needs to hold their arm in
        % the postion shown in the example. 
        % Run the following: imshow('calibration.jpg')
        % teleop.Position is updated with the current postion of the user's
        % hand in the ROS callback functions for the subscribers. The reference frame is assumed to be at the user's hip.
        % It is assumed that the offset from the hip to the shoulder is
        % equal tothe length of the upper arm. It is also assumed that the
        % upper arm and lower arm are both 0.3 meters long. Finally, the orientation of the end
        % effector is defined by the relative angle between the hand IMU
        % and the forearm IMU. 
        % To calibrate call the following:
        % teleop.Calibrate()
        

        %Initialize Properties
        obj.robot_ip='192.168.119.128';
        obj.my_ip = '192.168.1.6';
        %Connect to ROS
        try
        ABB_Robot.ROS_CONNECT(obj,obj.robot_ip,obj.my_ip)
        catch
            'ROS Already Connected'
        end
        obj.calibrate = [1 0 0 0];
        % Messages
        obj.datavec = rosmessage('threespace/dataVec');      
        % Subscribers
        obj.upper_arm_sub = rossubscriber('/r_upper_arm_data_vec',@obj.callBackU);
        obj.lower_arm_sub = rossubscriber('/r_lower_arm_data_vec',@obj.callBackL);
        obj.hand_sub = rossubscriber('/r_hand_data_vec',@obj.callBackH);
        obj.L1=.3; % length of upper arm
        obj.L2=.3;  % length of lower arm
        obj.PositionU = [1 0 0 0 0 0 0];
        obj.PositionL = [1 0 0 0 0 0 0];
        obj.Position = [0 0 0 0 0 0]; 
        obj.Quat = {[1 0 0 0],[1 0 0 0]};
        obj.Calibrate()
        end
        
        function Calibrate(obj)
            obj.calibrate = {obj.Quat{1}, obj.Quat{2}};
        end
              
        function callBackH(obj,~, message)
            quat=message.Quat.Quaternion;
            q=quatdivide([quat.W quat.X quat.Y quat.Z],obj.Quat{2});
            euler= flip(quat2eul(q,'ZYX'))*180/pi;
            obj.Position(1:3) = euler+[0 90 0];
        end
        
        function callBackL(obj,~, message)
            quat=message.Quat.Quaternion;
            obj.Quat{2} = [quat.W quat.X quat.Y quat.Z];
            q =quatmultiply(quatdivide([quat.W quat.X quat.Y quat.Z],obj.calibrate{2}),[-.7071,.7071,0,0]);
            R = quat2rotm(q);
            YZX = [R*[obj.L1;0;0]]';
            obj.PositionL = [q,[YZX(1) YZX(2) YZX(3)]+obj.PositionU(5:7)];
            euler= flip(quat2eul(q,'ZYX'));
            obj.Position(4:6) = [obj.PositionL(5)-(obj.L1)/2,obj.PositionL(6),obj.PositionL(7)+(obj.L1)];
        end
        
         function callBackU(obj,~, message)
            quat=message.Quat.Quaternion;
            obj.Quat{1} = [quat.W quat.X quat.Y quat.Z];
            q =quatmultiply(quatdivide([quat.W quat.X quat.Y quat.Z],obj.calibrate{1}),[-.7071,.7071,0,0]);
            R = quat2rotm(q);
            YZX = [R*[obj.L1;0;0]]';
            obj.PositionU = [q,YZX(1), YZX(2), YZX(3)];
         end       
    end
end

