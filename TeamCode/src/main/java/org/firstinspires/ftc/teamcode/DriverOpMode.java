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

    double linearMechanismStartPos;

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
        droneServo = hMap.droneServo;
        placeServo = hMap.placeServo;

        leftOdo = hMap.leftOdo;
        middleOdo = hMap.middleOdo;
        rightOdo = hMap.rightOdo;

        passedContactsRightOdo = rightOdo.getCurrentPosition();
        passedContactsLeftOdo = leftOdo.getCurrentPosition();
        passedContactsMiddleOdo = middleOdo.getCurrentPosition();

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hookMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearMechanismMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
        linearMechanismMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMechanismMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        robotRotation = ToolBox.scaleAngle(deltaRotation);


        //Move robot
        double deadzone = 0.05;
        if(Math.abs(joystickX) > deadzone || Math.abs(joystickY) > deadzone || Math.abs(rotate) > deadzone) {
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double moveAngle = joystickAngle;
            if(useGlobalPos){
                moveAngle = ToolBox.globalToRobot(joystickAngle, robotRotation);
            }
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle, magnitude * speedMultiplier, rotate * speedMultiplier);

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




        //odo test - drive back to zero on a
        if(gamepad1.a){
            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 0, robotRotation, 0, 0.5);

            telemetry.addData("angle to 0 in PI radians", Math.atan2(-posX, -posY)/Math.PI);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }


        //GAMEPAD 1
        double triggerDeadzone = 0.8;

        double linearMechanismMinPos = 0;
        double linearMechanismMaxPos = -2000;

        //Turn of all driver assist
        if(gamepad1.left_bumper && gamepad1.right_bumper) {
            if (gamepad1.y && gamepad1.x){
                linearMechanismMinPos = 1000000000;
                linearMechanismMaxPos = -100000000;
            }
            //Reset position and rotation
            else if (gamepad1.y) {
                robotRotation = 0;
                posX = 0;
                posY = 0;
            }
            //Toggle global position use
            else if (gamepad1.x) {
                if (useGlobalPos) {
                    useGlobalPos = false;
                } else {
                    useGlobalPos = true;
                }
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
        if(linearMechanismInput < -deadzone && linearMechanismMotor.getCurrentPosition() > linearMechanismMaxPos){
            //up - encoder subtracts
            linearMechanismMotor.setPower(linearMechanismInput);
        }
        else if (linearMechanismInput > deadzone && linearMechanismMotor.getCurrentPosition() < linearMechanismMinPos) {
            //down - encoder adds
            linearMechanismMotor.setPower(linearMechanismInput);
        }
        else{
            linearMechanismMotor.setPower(0);
        }

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
        if(gamepad2.x && gamepad2.left_bumper && gamepad2.right_bumper) {
            //hold
            if(gamepad2.left_trigger > triggerDeadzone) {
                droneServo.setPosition(1);
            }
            //let go
            else {
                droneServo.setPosition(0.55);
            }
        }

        //Place hook
        if(gamepad2.y && gamepad2.left_bumper && gamepad2.right_bumper){
            //og position
            if(gamepad2.left_trigger > triggerDeadzone) {
                hookServo.setPosition(1);
            }

            //hide hook
            else if(gamepad2.right_trigger > triggerDeadzone){
                hookServo.setPosition(0);
            }
            //place hook
            else{
                hookServo.setPosition(0.2);
            }
        }

        //Place pixel
        if(gamepad2.b){
            //go to og position
            if(gamepad2.left_trigger > triggerDeadzone) {
                placeServo.setPosition(1);
            }
            //place pixel
            else if(gamepad2.right_trigger > triggerDeadzone){
                placeServo.setPosition(0);
            }
            //lock pixel while travel
            else{
                placeServo.setPosition(0.65);
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
        telemetry.addData("PassedContactsRightOdo", passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", passedContactsMiddleOdo);
    }
}
