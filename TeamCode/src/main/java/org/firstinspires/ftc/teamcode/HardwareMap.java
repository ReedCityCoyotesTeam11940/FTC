package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareMap {
    //create motors
    public DcMotor frontRightMotor = null;
    public DcMotor frontLeftMotor = null;
    public DcMotor backRightMotor = null;
    public DcMotor backLeftMotor = null;

    // create servo
    public Servo randomServo = null;

    //additional variables
    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime;

    public Hardware(HardwareMap hwMap){
        intitalize(hwMap);
    }

    private void initialize(HardwareMap hwMap){
        hardwareMap = hwMap;

        frontRightMotor = hardwareMap.get(DcMotor.class,  "frontRightMotor");
    }
}
