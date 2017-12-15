MODULE Misc
    
    PROC print_Euler(VAR Orient rot)
        VAR num angleX;
        VAR num angleY;
        VAR num angleZ;
        
        angleX := EulerZYX(\X, rot);
        TPWrite "EulerX: " \Num:=angleX;
        angleY := EulerZYX(\Y, rot);
        TPWrite "EulerY: " \Num:=angleY;
        angleZ := EulerZYX(\Z, rot);
        TPWrite "EulerZ: " \Num:=angleZ;
    ENDPROC
    
    PROC explicit_ethernet_ip()
        VAR iodev dev;
        VAR rawbytes rawdata_out;
        VAR rawbytes rawdata_in;
        VAR num input_int;
        VAR byte return_status;
        VAR byte return_errcodecnt;
        VAR num return_errcode;
        VAR byte value;
        
        ! Empty contents of rawdata_out and rawdata_in
        ClearRawBytes rawdata_out;
        ClearRawBytes rawdata_in;
        
        ! Add Fieldbus command header to rawdata_out with service "GET_ATTRIBUTE_SINGLE" and path to QuickConnect attribute on I/0 unit.
        PackDNHeader "0E","6,20 F5 24 01 30 0C",rawdata_out;
        
        ! Open FCI device
        Open "/FCI1:"\File:="TheUnit",dev\Bin;
        
        ! Write the contents of rawdata_out to dev
        WriteRawBytes dev,rawdata_out\NoOfBytes:=RawBytesLen(rawdata_out);
        
        ! Read the answer from dev
        ReadRawBytes dev,rawdata_in;
        
        ! Close FCI device
        Close dev;
        
        ! Unpack rawdata_in to the variable return_status
        UnpackRawBytes rawdata_in,1,return_status\Hex1;
        
        ! The first byte is always the general status byte. 0 means success, see the CIP standard error codes.
        IF return_status=0 THEN
            TPWrite "Status OK from device. Status code: "\Num:=return_status;
            
            ! Unpack the read data value that follows the status byte. 
            UnpackRawBytes rawdata_in,2,value\Hex1;
            TPWrite "Read value: "\Num:=value;
        ELSE
            ! If the general status was not ok there is extended error information that can be retreived. First byte, after the general status byte, tells how many extended error words can be found.
            UnpackRawBytes rawdata_in,2,return_errcodecnt\Hex1;
            
            ! Unpack the number of extended status words. In this example only the first one is unpacked.
            UnpackRawBytes rawdata_in,3,return_errcode\IntX:=UINT;
            TPWrite "Error code from device: "\Num:=return_status;
            TPWrite "Additional error code count from device: "\Num:=return_errcodecnt;
            TPWrite "Additional error code from device: "\Num:=return_errcode;
        ENDIF
    ENDPROC
ENDMODULE