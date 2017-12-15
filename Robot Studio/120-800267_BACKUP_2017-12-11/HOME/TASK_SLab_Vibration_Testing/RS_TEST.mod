MODULE RS_TEST
    CONST jointtarget HOME_POSITION:=[[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PEG_PICK_OFF:=[[-0.003820414,-0.002415011,204.661489181],[0.000000008,0.000000327,1,-0.000000037],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PEG_PICK:=[[-0.003821036,-0.002403827,50.45415513],[0.000000007,0.00000027,1,-0.000000007],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PEG_PLACE_OFF:=[[0.000002509,-0.000003548,182.733480027],[0.000000006,0.707107068,0.707106494,-0.000000101],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget PEG_PLACE:=[[0.000002055,-0.000004251,32.978060504],[0.000000018,0.707107059,0.707106503,-0.000000042],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    PROC PATH_PEG_PLACE_()
        MoveAbsJ HOME_POSITION,v100,fine,tool_PEG_GRIPPER;
        Gripper_Open;
        
        FOR Y FROM 0 TO 1 DO
            FOR X FROM 0 TO 3 DO
                
        MoveJ offs(PEG_PICK_OFF,50.09*X,35.07*Y,0),v500,z20,tool_PEG_GRIPPER\WObj:=Peg_Holder;
        MoveL offs(PEG_PICK,50.09*X,35.07*Y,0),v500,fine,tool_PEG_GRIPPER\WObj:=Peg_Holder;
        Gripper_Close;
        !WaitTime 1;
        MoveL offs(PEG_PICK_OFF,50.09*X,35.07*Y,0),v500,fine,tool_PEG_GRIPPER\WObj:=Peg_Holder;
        
        MoveL offs(PEG_PLACE_OFF,50.09*X,35.07*Y,0),v500,z20,tool_PEG_GRIPPER\WObj:=Toy_Stand;
        MoveL offs(PEG_PLACE,50.09*X,35.07*Y,0),v500,fine,tool_PEG_GRIPPER\WObj:=Toy_Stand;
        Gripper_Open;
        !WaitTime 1;
        MoveL offs(PEG_PLACE_OFF,50.09*X,35.07*Y,0),v500,fine,tool_PEG_GRIPPER\WObj:=Toy_Stand;
        
        ENDFOR
        ENDFOR
        
    ENDPROC
ENDMODULE