package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor, frontRightMotor, leftOdo, middleOdo, rightOdo;
    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    public double rotateMultiplier = 0.8;

    @Override
    public void init() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        //get motors from hardware map
        backLeftMotor = hMap.backRightMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;

        leftOdo = hMap.leftOdo;
        middleOdo = hMap.middleOdo;
        rightOdo = hMap.rightOdo;

        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        passedContactsRightOdo = rightOdo.getCurrentPosition();
        passedContactsLeftOdo = leftOdo.getCurrentPosition();
        passedContactsMiddleOdo = middleOdo.getCurrentPosition();

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //get gamepad input
        double joystickY = -gamepad1.left_stick_y;
        double joystickX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

//        //Get odo deltas
//        int deltaContactsLeftOdo = leftOdo.getCurrentPosition() - passedContactsLeftOdo;
//        int deltaContactsRightOdo = rightOdo.getCurrentPosition() - passedContactsRightOdo;
//        int deltaContactsMiddleOdo = middleOdo.getCurrentPosition() - passedContactsMiddleOdo;
//
//        //Update passed odo contacts
//        passedContactsLeftOdo += deltaContactsLeftOdo;
//        passedContactsRightOdo += deltaContactsRightOdo;
//        passedContactsMiddleOdo += deltaContactsMiddleOdo;
//
//        //Get position change
//        double[] positionChange = Odometry.getPositionChange(-deltaContactsRightOdo, deltaContactsLeftOdo, -deltaContactsMiddleOdo, robotRotation);
//
//        double deltaX = positionChange[0];
//        double deltaY = positionChange[1];
//        double deltaRotation = positionChange[2];
//
//        //Update position
//        posX += deltaX;
//        posY += deltaY;
//        robotRotation = ToolBox.scaleAngle(robotRotation + deltaRotation);

        //Move robot
        double deadzone = 0.05;
        if(Math.abs(joystickX) > deadzone || Math.abs(joystickY) > deadzone || Math.abs(rotate) > deadzone) {
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(joystickAngle, magnitude, rotate * rotateMultiplier);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        else {
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
        }



//        //odo test - drive back to zero on a
//        if(gamepad1.a){
//            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 0, robotRotation, 0, 0.5);
//
//            telemetry.addData("angle to 0 in PI radians", Math.atan2(-posX, -posY)/Math.PI);
//
//            backLeftMotor.setPower(motorPowers[0]);
//            backRightMotor.setPower(motorPowers[1]);
//            frontLeftMotor.setPower(motorPowers[2]);
//            frontRightMotor.setPower(motorPowers[3]);
//        }


        //output data
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Robot direction in PI radians", robotRotation/Math.PI);
        telemetry.addData("PassedContactsRightOdo", passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", passedContactsMiddleOdo);
        telemetry.addData("MotorPowerBackLeft", backLeftMotor.getPower());
        telemetry.addData("MotorPowerBackRight", backRightMotor.getPower());
        telemetry.addData("MotorPowerFrontLeft", frontLeftMotor.getPower());
        telemetry.addData("MotorPowerFrontRight", frontRightMotor.getPower());
    }
}
