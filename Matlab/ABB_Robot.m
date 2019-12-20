classdef ABB_Robot < handle
    properties (Constant)
        % ROBOT PHYSICAL PROPERTIES (SI units)
        Name = 'ABB IRB 120'
    end
    
    properties(SetAccess = public)
        % CLASS PUBLIC PROPERTIES
        robot
        ik
        % messages
        JointTraj_msg
        Point_msg 
        % placeholder variables
        Moving=0;
        Listening
        Configuration
        Teleop
        Position
        Goal
        GoalT
        Demo
        Tasks
        % publisher
        joint_command_pub
        % subscriber
        joint_states_sub
        listen_sub
        %IP adresses
        robot_ip 
        my_ip 
    end
    
    methods(Static)
        function ROS_CONNECT(~,robot_ip, my_ip)
            rosinit(robot_ip)
        end
        
    end
    
    methods

        function obj = ABB_Robot()
         % Create a robot object by calling the following:
         % robot = ABB_Robot()
            
        % MAIN FUNCTION
            % build robot model
            obj.robot = robotics.RigidBodyTree;
            obj.Configuration=[];
            body1 = robotics.RigidBody('body1');
            jnt1 = robotics.Joint('jnt1','revolute');
            body1.Joint = jnt1;
            jnt1.PositionLimits = [-165 165]*pi/180;
            body2 = robotics.RigidBody('body2');
            jnt2 = robotics.Joint('jnt2','revolute');
            jnt2.PositionLimits = [-110 110]*pi/180;
            body3 = robotics.RigidBody('body3');
            jnt3 = robotics.Joint('jnt3','revolute');
            jnt3.PositionLimits = [-110 70]*pi/180;
            body4 = robotics.RigidBody('body4');
            jnt4 = robotics.Joint('jnt4','revolute');
            jnt4.PositionLimits = [-160 160]*pi/180;
            body5 = robotics.RigidBody('body5');
            jnt5 = robotics.Joint('jnt5','revolute');
            jnt5.PositionLimits = [-120 120]*pi/180;
            body6 = robotics.RigidBody('body6');
            jnt6 = robotics.Joint('jnt6','revolute');
            jnt6.PositionLimits = [-180 180]*pi/180;
            
            Transform = {};
            Transform{1} = obj.trans(0,0,0,0,0,290/1000);
            Transform{2} = obj.trans(90,0,180,0,0,103/1000);
            Transform{3} = obj.trans(0,0,0,0,270/1000,0);
            Transform{4} = obj.trans(0,270,0,-134/1000,70/1000,0);
            Transform{5} = obj.trans(0,90,0,0,0,168/1000);
            Transform{6} = obj.trans(0,-90,0,-72/1000,0,0);
            setFixedTransform(jnt1,Transform{1});
            setFixedTransform(jnt2,Transform{2});
            setFixedTransform(jnt3,Transform{3});
            setFixedTransform(jnt4,Transform{4});
            setFixedTransform(jnt5,Transform{5});
            setFixedTransform(jnt6,Transform{6});
            body2.Joint = jnt2;
            body3.Joint = jnt3;
            body4.Joint = jnt4;
            body5.Joint = jnt5;
            body6.Joint = jnt6;
            addBody(obj.robot,body1,'base')
            addBody(obj.robot,body2,'body1')
            addBody(obj.robot,body3,'body2')
            addBody(obj.robot,body4,'body3')
            addBody(obj.robot,body5,'body4')
            addBody(obj.robot,body6,'body5')
            obj.Configuration = randomConfiguration(obj.robot);    
            % IK solver
            obj.ik = robotics.InverseKinematics('RigidBodyTree',obj.robot);
            ik.SolverParameters.AllowRandomRestart = false;
            obj.ik.SolverAlgorithm = 'LevenbergMarquardt';
            %Initialize Properties
            obj.Tasks = struct('Task1',struct('X',PSM(),'Y',PSM(),'Z',PSM(),'Roll',PSM(),'Pitch',PSM(),'Yaw',PSM()));
            obj.Position = [0 0 0 0 0 0];
            obj.Goal = [0 0 0 0 0 0];
            obj.GoalT =[0,0]; 
            obj.robot_ip='192.168.119.128';
            obj.my_ip = '192.168.1.6';
            % Connect to ROS
            try
            ABB_Robot.ROS_CONNECT(obj,obj.robot_ip,obj.my_ip)
            catch
                'ROS Already Connected'
            end
                    % Messages
            obj.JointTraj_msg = rosmessage('trajectory_msgs/JointTrajectory');
            obj.JointTraj_msg.JointNames = {'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'};
            obj.Point_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            % Publishers
            obj.joint_command_pub = rospublisher('/joint_path_command');
            % Subscribers
            obj.joint_states_sub = rossubscriber('/joint_states',@obj.UpdateModel);
            obj.Listening = 0;
            obj.listen_sub = rossubscriber('/recognizer/output',@obj.VoiceCmd); 
            
        end
        
        function UpdateModel(obj,~,message)
            for i =1:6 
                obj.Configuration(i).JointPosition = message.Position(i);
                obj.Position(i) =  message.Position(i);
            end
        end
        
        function StartListening(obj)
            obj.Listening = 1;
            if isempty(obj.Teleop)==0
            obj.Teleop.Listening = 1;
            end
        end
        function StopListening(obj)
            obj.Listening = 0;
            if isempty(obj.Teleop)==0
            obj.Teleop.Listening = 0;
            end
        end
        
        function VoiceCmd(obj,~, message)
            if obj.Listening == 1
           if strcmp(message.Data,'home configuration')
               obj.Home
           elseif strcmp(message.Data,'send last')
               obj.SendLast
           elseif strcmp(message.Data,'go here')
               obj.RealTime
           elseif strcmp(message.Data,'start calibrate')
                    obj.Teleop.Calibrate
           end
            end
        end
        
        function Home(obj)
            obj.Point_msg.Positions = [0,0,0,0,0,0];
            obj.Point_msg.Velocities = [1,1,1,1,1,1];
            obj.JointTraj_msg.Points = obj.Point_msg;
            send(obj.joint_command_pub,obj.JointTraj_msg)
        end
        
        function GoTo(obj,Position)
            % This method will command the robot to move to Position. The
            % Postion variable needs to be defined as follows:
            % Position = [alpha,beta,gamma,x,y,z]
            T1 = getTransform(obj.robot,obj.Configuration,'body6');
            Positions = zeros(1,6);
            configSoln = obj.Configuration;
            tform = obj.trans(Position(1),Position(2),Position(3),Position(4),Position(5),Position(6));
            initialguess = configSoln; % use current configuration as initial guess
            weights = [1 1 1 1 1 1];
            [configSoln,solnInfo] = obj.ik('body6',tform,weights,initialguess);
            T2 = getTransform(obj.robot,configSoln,'body6');
            D = sqrt(abs(sum((T1(1:3,4)-T2(1:3,4)).^2))); % distance from current configuration to new configuration
            for j = 1:6
               Positions(j) = configSoln(j).JointPosition;
            end
            obj.Goal = Positions;
            obj.Point_msg.Positions = Positions;
            obj.JointTraj_msg.Points = obj.Point_msg;
            obj.GoalT = obj.GoalT+[6,0]*D+[.4 0]; % The goal time is proportional to the distance plus a constant. These values can be changed. 
            obj.GoalT(2) = obj.GoalT(2)+round(1e9*(obj.GoalT(1)-floor(obj.GoalT(1))));
            if obj.GoalT(2)>1e9
                obj.GoalT(2) = obj.GoalT(2)-1e9;   % GoalT(1) is seconds and GoalT(2) is nano-seconds
                obj.GoalT(1) = obj.GoalT(1)+1;
            end
            obj.GoalT(1) = floor(obj.GoalT(1));
            obj.JointTraj_msg.Points.TimeFromStart.Nsec = obj.GoalT(2);
            obj.JointTraj_msg.Points.TimeFromStart.Sec = obj.GoalT(1);
            send(obj.joint_command_pub,obj.JointTraj_msg)
        end
        
        function SendLast(obj)
            % Resend the last trajecory command to the robot.
            send(obj.joint_command_pub,obj.JointTraj_msg)
        end
        function Execute_Task(obj,str,T,S)
            % execute learned tasks with time scaling T and goal scaling S.
            roll = obj.Tasks.(str).Roll.Generate_Traj(T(1),S(1));
            pitch = obj.Tasks.(str).Pitch.Generate_Traj(T(2),S(2));
            yaw = obj.Tasks.(str).Yaw.Generate_Traj(T(3),S(3));
            x = obj.Tasks.(str).X.Generate_Traj(T(4),S(4));
            y = obj.Tasks.(str).Y.Generate_Traj(T(5),S(5));
            z = obj.Tasks.(str).Z.Generate_Traj(T(6),S(6));  
            t = obj.Tasks.(str).X.t*T(1);
            figure(3)
            subplot(3,2,1)
            plot(t,roll)
            subplot(3,2,1)
            plot(t,roll)
            subplot(3,2,2)
            plot(t,pitch)
            subplot(3,2,3)
            plot(t,yaw)
            subplot(3,2,4)
            plot(t,x)
            subplot(3,2,5)
            plot(t,y)
            subplot(3,2,6)
            plot(t,z)
            figure(2)
            obj.Execute_Traj(t,[roll',pitch',yaw',x',y',z'])
        end
        
        function Learn_Task(obj,str)
            % Input the name of the new task as a string. This method will
            % add a tasks to robot.Tasks with PSM models trained for the
            % data contrained in robot.Demo. 
            obj.Tasks.(str) = struct('X',PSM(),'Y',PSM(),'Z',PSM(),'Roll',PSM(),'Pitch',PSM(),'Yaw',PSM());
            t = obj.Demo(:,1);
            n=1;
            for i = {'Roll','Pitch','Yaw','X','Y','Z'}
                n=n+1;
                x = obj.Demo(:,n);
                obj.Tasks.(str).(i{1}).Learn_Traj(t,x,.01);                
            end
        end
        function Execute_Traj(obj,t,x)
            % this function takes a [nx1] time vector (t) and a [nx6] position
            % matrix (x); the order of the postion input for x i the following:
            %  x = [alpha,beta,gamma,x,y,z].
            % If the trajecory is too large, the ABB robot controller will say too many points where sent. To deal with this, 
            % this method breaks the trajectory into segmetns of length 80.
            % It is best to send trejcectories with less than 80 points.
            t = t(1:15:end);
            x = x(1:15:end,:);
            dt = t(2)-t(1);
            n=0;
            X={};
            T={};
            L = length(t);
            p = 0;           
            while length(x(:,1)) >80
               n=n+1;
               X{n} = x(1:80,:);
               T{n} = t(1:80);
               x(1:80,:)=[];
               t(1:80)=[];
            end
            n=n+1;
            X{n} = x;
            T{n} = t;
            for k = 1:length(X)
                x = X{k};
                t = T{k};
                Positions = zeros(length(x(:,1)),6);
                Velocities = Positions; 
                configSoln = obj.robot.homeConfiguration;
                for i = 1:length(x(:,1))
                    tform = obj.trans(x(i,1),x(i,2),x(i,3),x(i,4),x(i,5),x(i,6));
                    initialguess = configSoln;
                    weights = [2 2 2 1 1 1];
                    [configSoln,solnInfo] = obj.ik('body6',tform,weights,initialguess);
                    p = p+1;
                    p/L*100
                    for j = 1:6
                       Positions(i,j) = configSoln(j).JointPosition;
                    end
                end
                for i = 1:6
                    Velocities(:,i) = obj.d_dt(Positions(:,i),dt);
                end
                Sec = 0;
                Nsec = 0;
                obj.JointTraj_msg.Points = [];
                for i = 1:length(Positions(:,1)) 
                    obj.Point_msg = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                    obj.Point_msg.Positions = Positions(i,:);
                    obj.Point_msg.Velocities = Velocities(i,:);
                    if Nsec+round(dt*1e9)>=1e9
                        Nsec = Nsec+round(dt*1e9)-1e9;
                        Sec=Sec+1;
                    else
                        Nsec = Nsec+round(dt*1e9);
                    end
                    obj.Point_msg.TimeFromStart.Sec= Sec;
                    obj.Point_msg.TimeFromStart.Nsec= Nsec;
                    obj.JointTraj_msg.Points = [obj.JointTraj_msg.Points;obj.Point_msg];
                end
                send(obj.joint_command_pub,obj.JointTraj_msg)
                while sum(abs(obj.Position - Positions(end,:))>0.0001)>0
                    abs(obj.Position - Positions(end,:))
                    receive(obj.joint_states_sub)
                end
            end
        end
        
        function RealTime(obj)
            % goes to current postion of teleoperator
            obj.GoTo(obj.Teleop.Position)
        end
        
        function xd = d_dt(obj,x,dt)
            % take numerical derivative. To aviod the length of the array
            % changing, the function interpolates the value of the array to the second derivative. 
            s=size(x);
            if s(2)>s(1)
            x = [x x(end)+(x(end)-x(end-1))+((x(end)-x(end-1))-(x(end-1)-x(end-2)))];
            xd = diff(x)./dt;
            elseif s(1)>s(2)
            x = [x;x(end)+(x(end)-x(end-1))+((x(end)-x(end-1))-(x(end-1)-x(end-2)))];
            xd = diff(x)./dt;
            end
        end
        
        function Transformation = trans(obj,alpha,beta,gamma,x,y,z)
            % calculate tranformation matrix to end effector goal. 
            Rx = [1 0 0; 0 cosd(alpha) -sind(alpha); 0 sind(alpha) cosd(alpha)];
            Ry = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
            Rz = [cosd(gamma) -sind(gamma) 0; sind(gamma) cosd(gamma) 0; 0    0   1];
            rotationMatrix = Rz*Ry*Rx;
            Transformation = [rotationMatrix [x;y;z]; 0 0 0 1];
        end        
    end
end

