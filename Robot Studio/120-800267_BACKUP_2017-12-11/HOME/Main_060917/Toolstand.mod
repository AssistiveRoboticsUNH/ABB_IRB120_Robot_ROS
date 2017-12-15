MODULE Toolstand
    ! PCB grip position
	CONST robtarget tool_pickup0:=[[-633.79,325.11,-0.34],[0.812064,2.63764E-05,-0.000369742,-0.583568],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	! Gripper grip position
    CONST robtarget tool_pickup1:=[[-849.48,109.66,2.72],[0.816236,0.000247775,-0.00013021,-0.577719],[1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC AttachPCB()
        ! If we had a sensor to detect a tool on the QC5-M it, we would check it here.
        ! Instead we use pneumatics signal
        IF DOutput(doEX600_QCT) = 0 THEN
            TPWrite "Attempted to attach tool while tool was attached.";
            Stop;
        ENDIF
        
        IF diEX600_mount0 = 1 THEN
            MoveJ RelTool(tool_pickup0, 0, 0, -40), v100, fine, QC5_M\WObj:=wobj_Toolstand;
            MoveL tool_pickup0, v30, fine, QC5_M\WObj:=wobj_Toolstand;
            SetDO doEX600_QCT, 0;
        ELSE
            TPWrite "No tool in mount 0.";
        ENDIF
    ENDPROC
    
    PROC AttachGripper()
        ! If we had a sensor to detect a tool on the QC5-M it, we would check it here.
        ! Instead we use pneumatics signal
        IF DOutput(doEX600_QCT) = 0 THEN
            TPWrite "Attempted to attach tool while tool was attached.";
            Stop;
        ENDIF
        
        IF diEX600_mount1 = 1 THEN
            MoveJ RelTool(tool_pickup1, 0, 0, -40), v100, fine, QC5_M\WObj:=wobj_Toolstand;
            MoveL tool_pickup1, v30, fine, QC5_M\WObj:=wobj_Toolstand;
            SetDO doEX600_QCT, 0;
            WaitTime 0.5;
            MoveL tool_pickup1, v30, fine, QC5_Gripper\WObj:=wobj_Toolstand;
        ELSE
            TPWrite "No tool in mount 1.";
        ENDIF
    ENDPROC
ENDMODULE