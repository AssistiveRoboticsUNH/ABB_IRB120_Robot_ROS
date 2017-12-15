MODULE Gripper
    !***********************************************************
    !
    ! Module:  Gripper
    !
    ! Description: 14 June 2017, Gripper Commands
    !
    ! Author: Mike Locke
    !
    ! Version: 1.0
    !
    !*************************************************************************************************************************************************  
    
    VAR num wait_time:=0.5;
    
    !*************************************************************************************************************************************************  
    
    !INITIALIZE GRIPPERS (Both Valves Closed)
    PROC Gripper_Initialize()
        SetDO doEX600_Gripper_Open,0;
        SetDO doEX600_Gripper_Closed,0;
    ENDPROC
    
    !CLOSE GRIPPERS
    PROC Gripper_Close()
        SetDO doEX600_Gripper_Open,0;
        SetDO doEX600_Gripper_Closed,1;
        WaitTime wait_time;
    ENDPROC
    
    !OPEN GRIPPERS
    PROC Gripper_Open()
        SetDO doEX600_Gripper_Closed,0;   
        SetDO doEX600_Gripper_Open,1;
        WaitTime wait_time;
    ENDPROC
    
ENDMODULE