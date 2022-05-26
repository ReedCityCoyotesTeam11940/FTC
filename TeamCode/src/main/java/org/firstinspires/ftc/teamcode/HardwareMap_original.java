package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMap_original {
    //create motors
    public DcMotor Right_Front = null;
    public DcMotor Left_Front = null;
    public DcMotor Right_Back = null;
    public DcMotor Left_Back = null;

    //additional variables
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    public HardwareMap(HardwareMap hwMap){
        initialize(hwMap);
    }

    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        frontRightMotor = hardwareMap.get(DcMotor.class
    }
}
