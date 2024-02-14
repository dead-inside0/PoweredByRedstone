package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor linearMechanismMotor;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    double timeOfLastFrame = 0;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    @Override
    public void init() {
        //get motors from hardware map
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        backLeftMotor = hMap.backLeftMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;

        linearMechanismMotor = hMap.linearMechanismMotor;

        passedContactsRightOdo = backRightMotor.getCurrentPosition();
        passedContactsLeftOdo = backLeftMotor.getCurrentPosition();
        passedContactsMiddleOdo = frontLeftMotor.getCurrentPosition();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //get gamepad input
        double joystickY = -gamepad1.left_stick_y;
        double joystickX = gamepad1.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        double linearMechanismInput = gamepad2.left_stick_y;

        //calculate delta time
        double deltaTime = runtime.seconds() - timeOfLastFrame;

        //Update time of last frame
        timeOfLastFrame += deltaTime;

        //Get odo deltas
        int deltaContactsLeftOdo = backLeftMotor.getCurrentPosition() - passedContactsLeftOdo;
        int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - passedContactsRightOdo;
        int deltaContactsMiddleOdo = frontLeftMotor.getCurrentPosition() - passedContactsMiddleOdo;

        //Update passed odo contacts
        passedContactsLeftOdo += deltaContactsLeftOdo;
        passedContactsRightOdo += deltaContactsRightOdo;
        passedContactsMiddleOdo += deltaContactsMiddleOdo;

        //Get position change
        double[] positionChange = Odometry.getPositionChange(-deltaContactsRightOdo, deltaContactsLeftOdo, -deltaContactsMiddleOdo, robotRotation);
        double deltaX = positionChange[0];
        double deltaY = positionChange[1];
        double deltaRotation = positionChange[2];

        //Update position
        posX += deltaX;
        posY += deltaY;
        robotRotation += deltaRotation;
        robotRotation = ToolBox.scaleAngle(robotRotation);


        //Move robot
        double deadzone = 0.05;
        if(Math.abs(joystickX) > deadzone || Math.abs(joystickY) > deadzone || Math.abs(rotate) > deadzone) {
            double joystickAngle = Math.atan2(-joystickX, joystickY);
            double moveAngle = ToolBox.joystickToRobot(joystickAngle, robotRotation);
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle, magnitude, rotate);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        else {
            double breakingPower = 0.01;
            if(Math.abs(deltaX) > 0.5 || Math.abs(deltaY) > 0.5 || Math.abs(deltaRotation) > Math.PI/360){
                //break
                frontLeftMotor.setPower(breakingPower);
                frontRightMotor.setPower(breakingPower);
                backLeftMotor.setPower(breakingPower);
                backRightMotor.setPower(breakingPower);

                frontLeftMotor.setPower(-breakingPower);
                frontRightMotor.setPower(-breakingPower);
                backLeftMotor.setPower(-breakingPower);
                backRightMotor.setPower(-breakingPower);
            }
        }

        if(linearMechanismMotor.getCurrentPosition() > 100 && linearMechanismMotor.getCurrentPosition() < 1900) {
            linearMechanismMotor.setPower(linearMechanismInput);
        }


        //odo test - drive back to zero on y
        if(gamepad1.a){
            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 0, robotRotation, 0, 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }




        //output data
        //telemetry.addData("Joystick X", joystickX);
        //telemetry.addData("Joystick Y", joystickY);
        //telemetry.addData("Rotate joystick", rotate);
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Robot direction in PI radians", robotRotation/Math.PI);
        telemetry.addData("PassedContactsRightOdo", -passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", -passedContactsMiddleOdo);
    }
}
