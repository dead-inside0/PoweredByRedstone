package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private MyHardwareMap hMap;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;


    double deadzone = 0.05;
    double posX = 0;
    double posY = 0;
    double robotAngle = 0;

    double timeToMaxSpeed = 1;

    double timeOfLastFrame = 0;

    double timeSinceBreak = 0;

    double maxPower = 0.75;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    @Override
    public void init() {
        //get motors from hardware map
        hMap = new MyHardwareMap(hardwareMap);
        backLeftMotor = hMap.motor1; //odo1
        backRightMotor = hMap.motor2; //odo2
        frontRightMotor = hMap.motor3; //odo3
        frontLeftMotor = hMap.motor4;
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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


        //move robot in local direction
        //backLeftMotor.setPower(Range.clip(joystickY - joystickX + rotate, -1.0, 1.0));
        //backRightMotor.setPower(Range.clip(joystickY + joystickX - rotate, -1.0, 1.0));
        //frontLeftMotor.setPower(Range.clip(joystickY + joystickX + rotate, -1.0, 1.0));
        //frontRightMotor.setPower(Range.clip(joystickY - joystickX - rotate, -1.0, 1.0));


        //update position
        int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - passedContactsRightOdo;
        int deltaContactsLeftOdo = backLeftMotor.getCurrentPosition() - passedContactsLeftOdo;
        int deltaContactsMiddleOdo = frontRightMotor.getCurrentPosition() - passedContactsMiddleOdo;
        passedContactsRightOdo += deltaContactsRightOdo;
        passedContactsLeftOdo += deltaContactsLeftOdo;
        passedContactsMiddleOdo += deltaContactsMiddleOdo;
        double[] positionChange = Odometry.getPositionChange(deltaContactsRightOdo, deltaContactsLeftOdo, deltaContactsMiddleOdo, robotAngle);
        posX += positionChange[0];
        posY += positionChange[1];
        robotAngle += positionChange[2];

        //calculate delta time
        double deltaTime = runtime.seconds() - timeOfLastFrame;
        timeOfLastFrame = runtime.seconds();


        //move robot
        if((joystickX <= -deadzone || joystickX >= deadzone) && (joystickY <= -deadzone || joystickY >= deadzone)) {
            timeSinceBreak += deltaTime;
            double accelerationMultiplier = ToolBox.acceleration(timeSinceBreak, timeToMaxSpeed);
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double moveAngle = ToolBox.joystickToRobot(joystickAngle, robotAngle);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle);
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);
            if (gamepad1.right_trigger > 0.9) {
                maxPower = 1.0;
            } else if (gamepad1.left_trigger > 0.9) {
                maxPower = 0.5;
            } else{
                maxPower = 0.75;
            }
            backLeftMotor.setPower(Range.clip((motorPowers[0] + rotate) * magnitude * accelerationMultiplier, -maxPower, maxPower));
            backRightMotor.setPower(Range.clip((motorPowers[1] + rotate) * magnitude * accelerationMultiplier, -maxPower, maxPower));
            frontLeftMotor.setPower(Range.clip((motorPowers[2] + rotate) * magnitude * accelerationMultiplier, -maxPower, maxPower));
            frontRightMotor.setPower(Range.clip((motorPowers[3] + rotate) * magnitude * accelerationMultiplier, -maxPower, maxPower));
        }
        else {
            timeSinceBreak = 0;

            //break
            double breakingPower = 0.05;
            backLeftMotor.setPower(breakingPower);
            backRightMotor.setPower(breakingPower);
            frontLeftMotor.setPower(breakingPower);
            frontRightMotor.setPower(breakingPower);
        }


        //odometry test
        int targetX = 0;
        int targetY = 0;
        double speed = 0.25;
        if(gamepad1.triangle) {
            if (Math.abs(posX - targetX) >= 0.01 && Math.abs(posY - targetY) >= 0.01) {
                double[] motorPowersToPoint = ToolBox.getMotorPowersToPoint(posX, posY, targetX, targetY);
                backLeftMotor.setPower(motorPowersToPoint[0] * speed);
                backRightMotor.setPower(motorPowersToPoint[1] * speed);
                frontLeftMotor.setPower(motorPowersToPoint[2] * speed);
                frontRightMotor.setPower(motorPowersToPoint[3] * speed);
            }
        }


        //output data
        //telemetry.addData("Joystick X", joystickX);
        //telemetry.addData("Joystick Y", joystickY);
        //telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("PassedContactsRightOdo", passedContactsRightOdo);
        telemetry.addData("PassedContactsLeftOdo", passedContactsLeftOdo);
        telemetry.addData("PassedContactsMiddleOdo", passedContactsMiddleOdo);
        telemetry.addData("Direction Angle", robotAngle);
    }
}
