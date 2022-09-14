package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Gyro")
public class Gyro extends LinearOpMode {
    /*
    the keyword 'extends' means to create a child class of LinearOpMode, or to let Gyro
    inherit traits of LinearOpMode, it's parent class, like you might inherit your parents' things
     */

    // creates variables we'll use later
    private DcMotor left;
    private DcMotor right;
    private GyroSensor gyro;
    private DcMotor BackLeft;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor FrontRight;

    boolean onTarget;
    ElapsedTime _7BholdTimeVariable_7D;
    double steer;
    double error;
    double GYRO_ADJUSTMENT;
    double COUNTS_PER_INCH;
    double P_TURN_COEFF;
    int HEADING_THRESHOLD;
    double leftSpeed;
    double rightSpeed;
    double P_DRIVE_COEFF;

    // this function is executed when this Op Mode is selected from the driver station
    // the override keyword is used to override a parent class's traits inside a child class
    @Override
    public void runOpMode(){
        int COUNTS_PER_MOTOR_REV;
        int DRIVE_GEAR_REDUCTION;
        int WHEEL_DIAMETER_INCHES;
        double DRIVE_SPEED;
        double TURN_SPEED;

        // assigns those variables we set up to certain values, like a name
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        gyro = hardwareMap.get(GyroSensor.class, "gyro");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        //initialization session, initializing certain values that tells the robot how it should run

        left.setPower(0);
        right.setPower(0);
        COUNTS_PER_MOTOR_REV = 1440;
        DRIVE_GEAR_REDUCTION = 1;
        WHEEL_DIAMETER_INCHES = 4;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /(WHEEL_DIAMETER_INCHES * 3.14159);
        GYRO_ADJUSTMENT = 1.0765;
        DRIVE_SPEED = 0.7;
        TURN_SPEED = 0.4;
        HEADING_THRESHOLD = 1;
        P_DRIVE_COEFF = 0.15;
        P_TURN_COEFF = 0.1;
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro.calibrate();

        /*
        while the robot is stopped and the gyro is not done calibrating, this message will
        display, telling the drivers to wait for the gyro to finish calibrating
        */
        while (!isStopRequested() && gyro.isCalibrating()){
            telemetry.addData("gyro calibrating", "wait to press start");
            telemetry.update();
        }

        telemetry.addData("gyro calibrated", "ready to run");
        telemetry.addData("robot heading = ", ((ModernRoboticsI2cGyro)gyro).getIntegratedZValue());
        telemetry.update();
        waitForStart();
        //start button pressed
        gyro.resetZAxisIntegrator();
        gyroDrive(DRIVE_SPEED, 12, 0);
        gyroDrive(DRIVE_SPEED, -12, 0);
        gyroTurn(TURN_SPEED, 90);
        gyroHold(TURN_SPEED, 90, 0.5);
        gyroTurn(TURN_SPEED, 180);
        gyroHold(TURN_SPEED, 180, 0.5);
        gyroTurn(TURN_SPEED, 270);
        gyroHold(TURN_SPEED, 270, 0.5);
        gyroTurn(TURN_SPEED, 0);
        gyroHold(TURN_SPEED, 0, 0.5);
    }

    // sets up some variables in telemetry, then connects the correct motors to the correct variables
    private boolean onHeading(double p_speed, double p_angle, double PCoeff) {
        onTarget = false;
        telemetry.addData("speed", p_speed);
        telemetry.addData("angle", p_angle);
        telemetry.addData("pCoeff", PCoeff);
        telemetry.addData("error", error);
        telemetry.update();
        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0;
            rightSpeed = 0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = steer * p_speed;
            leftSpeed = rightSpeed * 1;
        }

        BackLeft.setPower(leftSpeed);
        FrontLeft.setPower(leftSpeed);
        BackRight.setPower(rightSpeed);
        FrontRight.setPower(rightSpeed);

        telemetry.addData("target", p_angle);
        telemetry.addData("error", error);
        telemetry.addData("steer", steer);
        telemetry.addData("leftSpeed", leftSpeed);
        telemetry.addData("rightSpeed", rightSpeed);
        return onTarget;
    }

    // establishes the variables targetAngle and robotError, then never uses them again
    private double getError(double targetAngle){
        double robotError;
        robotError = targetAngle - ((ModernRoboticsI2cGyro)gyro).getIntegratedZValue()* GYRO_ADJUSTMENT;
        while(robotError > 180){
            robotError = robotError - 360;
        }
        while (robotError <= -180){
            robotError = robotError + 360;
        }
        return robotError;
    }

    // sets up the code to let the gyro turn
    private void gyroTurn(double p_speed, int p_angle){
        _7BholdTimeVariable_7D = new ElapsedTime();
        while(opModeIsActive() && !onHeading(p_speed, p_angle, P_TURN_COEFF)){
            telemetry.update();
        }
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        FrontRight.setPower(0);
    }

    // establishes the code to let the gyro hold this; not the code telling the gyro HOW to hold
    private void gyroHold (double p_speed, int p_angle, double p_holdTime){
        _7BholdTimeVariable_7D = new ElapsedTime();
        while (opModeIsActive() && _7BholdTimeVariable_7D.time() < p_holdTime){
            onTarget = onHeading(p_speed,p_angle,P_TURN_COEFF);
            telemetry.update();
        }
        BackLeft.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        FrontRight.setPower(0);
    }

    // makes it so that the steer variables never exceeds the integers 1 and -1
private double getSteer(double p_error, double PCoeff){
        steer = p_error * PCoeff;
        if (steer > 1){
            steer = -1;
        }
        return steer;
        /* the y axis of a joystick ranges from -1 in its topmost position to +1 in its
        bottommost position. we nullify this value so that the topmost position corresponds
        to the maximum forward power.
        */
}

// sets up the variables that let the robot drive
 private void gyroDrive(double p_speed, int p_distance, int p_angle){
        double MoveCounts;
        double newLeftBackTarget;
        double newRightBackTarget;
        double newLeftFrontTarget;
        double newRightFrontTarget;
        double max;

        if (opModeIsActive()){
            MoveCounts = p_distance * COUNTS_PER_INCH;

            //newTarget = current position + how far away the driver wants to go
            newLeftBackTarget = BackLeft.getCurrentPosition() + MoveCounts;
            newRightBackTarget = BackRight.getCurrentPosition() + MoveCounts;
            newLeftFrontTarget = FrontLeft.getCurrentPosition() + MoveCounts;
            newRightFrontTarget = FrontRight.getCurrentPosition() + MoveCounts;


            //sets the target position to where the driver wants to go
            BackLeft.setTargetPosition((int) newLeftBackTarget);
            BackRight.setTargetPosition((int) newRightBackTarget);
            FrontLeft.setTargetPosition((int) newLeftFrontTarget);
            FrontRight.setTargetPosition((int) newRightFrontTarget);

            //tells the motors to go to the position the driver wants
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // sets the motors to the speed the driver wants
            FrontLeft.setPower(p_speed);
            FrontRight.setPower(p_speed);
            BackLeft.setPower(p_speed);
            BackRight.setPower(p_speed);

            // while all the motors are busy, execute the following code
            // the following code is making sure that the robot is ok while all motors are busy
            while (opModeIsActive() && BackLeft.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && FrontRight.isBusy()){
                error = getError(p_angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                if (p_distance < 0){
                    steer = steer * -1;
                }

                leftSpeed = p_speed - steer;
                rightSpeed = p_speed + steer;

                if (Math.abs(leftSpeed) > Math.abs(rightSpeed)){
                    max = leftSpeed;
                }else{
                    max = rightSpeed;
                }
                if (max > 1){
                  leftSpeed = leftSpeed / max;
                  rightSpeed = rightSpeed / max;

                  FrontLeft.setPower(leftSpeed);
                  FrontRight.setPower(rightSpeed);
                  BackLeft.setPower(leftSpeed);
                  BackRight.setPower(rightSpeed);
                telemetry.addData("BackLeft",BackLeft.getCurrentPosition());
                telemetry.addData("Back Right", BackRight.getCurrentPosition());
                telemetry.addData("Front Right", FrontRight.getCurrentPosition());
                telemetry.addData("Front Left", FrontLeft.getCurrentPosition());

                telemetry.addData(":LeftBackTarget", newLeftBackTarget);
                telemetry.addData("RightBackTarget", newRightBackTarget);
                telemetry.addData("RightFrontTarget", newRightFrontTarget);
                telemetry.addData("LeftFrontTarget", newLeftFrontTarget);

                telemetry.addData("Target", p_angle);
                telemetry.addData("Steer", steer);
                telemetry.addData("LeftSpeed", leftSpeed);
                telemetry.addData("RightSpeed", rightSpeed);

                telemetry.update();
                }
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);

                BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
 }
}