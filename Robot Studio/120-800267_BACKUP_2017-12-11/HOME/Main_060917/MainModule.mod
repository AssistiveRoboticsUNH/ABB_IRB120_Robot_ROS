MODULE MainModule
   
	TASK PERS wobjdata wobj_PCB:=[FALSE,TRUE,"",[[188.001,291.997,238.216],[0.883504,-6.05471E-05,-1.7545E-05,-0.468424]],[[0,0,0],[1,0,0,0]]];
	TASK PERS wobjdata wobj_Toolstand:=[FALSE,TRUE,"",[[307.076,-362.851,323.412],[0.000219385,0.925757,-0.378118,6.41316E-05]],[[0,0,0],[1,0,0,0]]];
	TASK PERS wobjdata wobj_Partsfeeder:=[FALSE,TRUE,"",[[360.438,-299.009,214.863],[0.697158,-2.80285E-05,4.02439E-05,0.716918]],[[-19.1258,9.48129,0],[1,0,0,0]]];
	TASK PERS tooldata QC5_M:=[TRUE,[[0,0,92.838],[1,0,0,0]],[1,[97.5,0,39.1],[1,0,0,0],0.018,0.006,0.008]];
	TASK PERS tooldata QC5_Gripper:=[TRUE,[[0,0,177.46],[1,0,0,0]],[1.2,[79.1,0,50],[1,0,0,0],0.018,0.01,0.009]];
	TASK PERS tooldata QCT_PCB:=[TRUE,[[0,0,250],[1,0,0,0]],[1.4,[65.2,0,91.5],[1,0,0,0],0.031,0.025,0.011]];
    TASK PERS wobjdata wobj_Station:=[FALSE,TRUE,"",[[184.579,352.245,150.735],[0.0147377,-0.479585,-0.877348,-0.00647388]],[[0,0,0],[1,0,0,0]]];
	CONST robtarget zeroes:=[[0,0,0],[1.37903E-05,0.371639,-0.928378,6.44596E-06],[0,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget Home:=[[384.05,5.09,609.09],[0.694633,-0.000243783,0.719314,0.00846163],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget TEST1:=[[-507.34,371.03,-53.43],[0.792222,0.00023708,2.20902E-06,-0.610233],[-2,-1,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget Test1_obj0:=[[-376.27,153.55,320.49],[4.73344E-05,-0.974081,-0.2262,9.20199E-05],[1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget Test0_obj0:=[[-373.08,-151.29,323.68],[5.58358E-05,0.972432,0.233185,-0.00039062],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	
	PROC main()
        !AttachGripper;
        GripperInit;
        MoveToDetectedObject is7200C_472158;
        Stop;
	ENDPROC
ENDMODULE