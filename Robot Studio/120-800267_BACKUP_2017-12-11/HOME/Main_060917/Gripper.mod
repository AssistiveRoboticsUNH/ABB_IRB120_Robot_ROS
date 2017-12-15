MODULE Gripper
    VAR num wait_time:=0.7;
    
    ! Clear IO
    PROC GripperInit()
        SetDO doEX600_Gripper_Open, 0;
        SetDO doEX600_Gripper_Closed, 0;
    ENDPROC
    
    ! In theory: maintain pneumatic pressure when closed, but not when open.
    PROC GripperClose()
        SetDO doEX600_Gripper_Closed, 1;
        WaitTime wait_time;
    ENDPROC
    
    PROC GripperOpen()
        SetDO doEX600_Gripper_Closed, 0;
        SetDO doEX600_Gripper_Open, 1;
        WaitTime wait_time;
        SetDO doEX600_Gripper_Open, 0;
    ENDPROC
ENDMODULE