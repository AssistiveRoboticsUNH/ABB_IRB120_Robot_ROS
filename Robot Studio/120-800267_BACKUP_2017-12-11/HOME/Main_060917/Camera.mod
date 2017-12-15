MODULE Camera
    !Change the job name
    CONST string job_name := "fuse.job";
    
    ! wobj0 target
    CONST robtarget scan_fuse0:=[[218.35,162.94,61.09],[0.000765337,0.382108,0.924117,0.000953337],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget grip_position:=[[134.79,154.34,-50.69],[0.0003238,-0.972488,-0.23295,-0.00116017],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget grip_position1:=[[65.59,22.99,-49.50],[0.000112717,0.295115,-0.955462,0.000224792],[-1,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC MoveToDetectedObject(VAR cameradev cam0)
        VAR cameratarget cam_object;
        VAR Pose cam_frame;
        VAR num cam_frame_angle;
        
        ! Reset object frame after task
        wobj_Partsfeeder.oframe := [[0,0,0],[1,0,0,0]];
        
        !Change the camera name
        !CONST cameradev cam0 := is7200C_472158;
        CamSetProgramMode cam0;
        CamLoadJob cam0, job_name;
        CamSetRunMode cam0;
        
        !If the camera is mounted on the robot, store this position during setup
        !so that the robot may always return to this position before requesting an image.
        MoveL scan_fuse0, v100, fine, QC5_Gripper \WObj:=wobj_Partsfeeder;
        
        CamReqImage cam0;
        CamGetResult cam0, cam_object;
        
        ! We zero the object rotation because it rotates around the work object definition
        ! Instead we'll later use RelTool to rotate the tool.
        cam_frame := cam_object.cframe;
        cam_frame_angle := -EulerZYX(\Z, cam_frame.rot); ! Only need Z. negative because camera Z is opposite work object
        cam_frame.rot := [1, 0, 0, 0]; ! Zero out the rotation
        
        wobj_Partsfeeder.oframe := cam_frame;
        
        
        !During the first cycle, run the program until this point,
        !then jog the tool to the desired grip position and modpos grip_position
        

        ! Z is negative here because grip_position target points down.
        GripperOpen;
        MoveL RelTool(grip_position, 0, 0, -80, \Rz:=cam_frame_angle), v30, fine, QC5_Gripper\WObj:=wobj_Partsfeeder;
        MoveL RelTool(grip_position, 0, 0, 0, \Rz:=cam_frame_angle), v30, fine, QC5_Gripper\WObj:=wobj_Partsfeeder;
        GripperClose;
        
        ! Reset object frame after task
        wobj_Partsfeeder.oframe := [[0,0,0],[1,0,0,0]];
    ENDPROC
ENDMODULE