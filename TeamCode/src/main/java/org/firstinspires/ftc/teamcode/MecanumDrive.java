package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumDrive (Blocks to Java)")
public abstract class MecanumDrive extends LinearOpMode {
    private DcMotor Right_Front;
    private DcMotor Left_Front;
    private DcMotor Right_Back;
    private DcMotor Left_Back;

    //this function is started when this Op Mode is selected from the drives station

    public void runOpMode() {
        float Vertical;
        float Horizontal;
        float Pivot;

        Right_Front = hardwareMap.get(DcMotor.class,"Right_Front");
        Right_Back = hardwareMap.get(DcMotor.class,"Right_Back");
        Left_Front = hardwareMap.get(DcMotor.class,"Left_Front");
        Left_Back = hardwareMap.get(DcMotor.class,"Left_Back");

        Right_Front.setDirection(DcMotorSimple.Direction.REVERSE);
        Right_Back.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if(opModeIsActive()){
            Vertical = -gamepad1.left_stick_y;
            Horizontal = gamepad1.left_stick_x;
            Pivot = gamepad1.right_stick_x;
            Right_Front.setPower(-Pivot + (Vertical - Horizontal));
            Right_Back.setPower(-Pivot + Vertical + Horizontal);
            Left_Front.setPower(Pivot + Vertical + Horizontal);
            Left_Back.setPower(Pivot + (Vertical - Horizontal));
        }
    }
}


