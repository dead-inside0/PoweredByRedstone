package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor,frontRightMotor, linearMechanismMotor, pickUpMotor, hookMotor, leftOdo, rightOdo, middleOdo;
    public Servo droneServo, placeServo, hookServo;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    double timeOfLastFrame = 0;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    boolean useGlobalPos = true;
    double speedMultiplier = 0.8;

    @Override
    public void init() {
        //get motors from hardware map
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        backLeftMotor = hMap.backLeftMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;

        linearMechanismMotor = hMap.linearMechanismMotor;
        pickUpMotor = hMap.pickUpMotor;
        hookMotor = hMap.hookMotor;

        hookServo = hMap.hookServo;
        //droneServo = hMap.droneServo;
        placeServo = hMap.placeServo;

        leftOdo = hMap.leftOdo;
        middleOdo = hMap.middleOdo;
        rightOdo = hMap.rightOdo;

        passedContactsRightOdo = rightOdo.getCurrentPosition();
        passedContactsLeftOdo = leftOdo.getCurrentPosition();
        passedContactsMiddleOdo = middleOdo.getCurrentPosition();

        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearMechanismMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        double rotate = gamepad1.right_stick_x;
        double linearMechanismInput = gamepad2.right_stick_y;

        //calculate delta time
        double deltaTime = runtime.seconds() - timeOfLastFrame;

        //Update time of last frame
        timeOfLastFrame += deltaTime;

        //Get odo deltas
        int deltaContactsLeftOdo = leftOdo.getCurrentPosition() - passedContactsLeftOdo;
        int deltaContactsRightOdo = rightOdo.getCurrentPosition() - passedContactsRightOdo;
        int deltaContactsMiddleOdo = middleOdo.getCurrentPosition() - passedContactsMiddleOdo;

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
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double moveAngle = joystickAngle;
            if(useGlobalPos){
                moveAngle = ToolBox.globalToRobot(joystickAngle, robotRotation);

            }
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle, magnitude * speedMultiplier, rotate);

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




        //odo test - drive back to zero on a
        if(gamepad1.a){
            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 0, robotRotation, 0, 1);

            telemetry.addData("angle to 0 in PI radians", Math.atan2(-posX, -posY)/Math.PI);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }


        //GAMEPAD 1
        double triggerDeadzone = 0.8;

        //Reset position and rotation
        if(gamepad1.y && gamepad1.left_bumper && gamepad1.right_bumper){
            robotRotation = 0;
            posX = 0;
            posY = 0;
        }

        //Toggle global position use
        if(gamepad1.x && gamepad1.left_bumper && gamepad1.right_bumper){
            if(useGlobalPos){
                useGlobalPos = false;
            }
            else{
                useGlobalPos = true;
            }
        }
        
        //Change speed multiplier
        if(gamepad1.left_trigger > triggerDeadzone){
            speedMultiplier = 0.5;
        }
        else if (gamepad1.right_trigger > triggerDeadzone) {
            speedMultiplier = 1;
        }
        else{
            speedMultiplier = 0.8;
        }


        //GAMEPAD 2

        // Move arm
        linearMechanismMotor.setPower(linearMechanismInput);

        //Pick up pixel
        if(gamepad2.a){
            //spit out pixel
            if(gamepad2.left_trigger > triggerDeadzone) {
                pickUpMotor.setPower(-1);
            }
            //pick up pixel
            else {
                pickUpMotor.setPower(1);
            }
        }
        //Stop motor
        else{
            pickUpMotor.setPower(0);
        }


        //Pull hook
        if(gamepad2.dpad_up){
            hookMotor.setPower(1);
        }
        //Release hook
        else if (gamepad2.dpad_down) {
            hookMotor.setPower(-0.5);
        }
        //Stop motor
        else{
            hookMotor.setPower(0);
        }

        //Shoot drone
        if(gamepad2.b && gamepad2.left_bumper && gamepad2.right_bumper) {
            //hold
            if(gamepad2.left_trigger > triggerDeadzone) {
                droneServo.setPosition(1);
            }
            //let go
            else {
                droneServo.setPosition(0);
            }
        }

        //Place hook
        if(gamepad2.y){
            //hide hook
            if(gamepad2.left_trigger > triggerDeadzone) {
                hookServo.setPosition(1);
            }
            //place hook
            else{
                hookServo.setPosition(0.6);
            }
        }


        //Place pixel
        if(gamepad2.b){
            //hide placement mechanism
            if(gamepad2.left_trigger > triggerDeadzone) {
                placeServo.setPosition(0);
            }
            //place pixel
            else{
                placeServo.setPosition(1);
            }
        }


        //output data
        //telemetry.addData("Joystick X", joystickX);
        //telemetry.addData("Joystick Y", joystickY);
        //telemetry.addData("Rotate joystick", rotate);
        telemetry.addData("Using global position", useGlobalPos);
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Robot direction in PI radians", robotRotation/Math.PI);
        telemetry.addData("PassedContactsRightOdo", -passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", -passedContactsMiddleOdo);
    }
}
