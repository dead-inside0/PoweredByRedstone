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
    double robotRotation = 0;
    double deltaX = 0;
    double deltaY = 0;
    double deltaRotation = 0;

    double timeOfLastFrame = 0;
    double timeAccelerating = 0;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    @Override
    public void init() {
        //get motors from hardware map
        hMap = new MyHardwareMap(hardwareMap);
        backLeftMotor = hMap.backLeftMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

        //calculate delta time
        double deltaTime = runtime.seconds() - timeOfLastFrame;

        //Update time of last frame
        timeOfLastFrame = runtime.seconds();

        //Get odo deltas
        int deltaContactsLeftOdo = backLeftMotor.getCurrentPosition() - passedContactsLeftOdo;
        int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - passedContactsRightOdo;
        int deltaContactsMiddleOdo = frontLeftMotor.getCurrentPosition() - passedContactsMiddleOdo;

        //Update passed odo contacts
        passedContactsLeftOdo += deltaContactsLeftOdo;
        passedContactsRightOdo += deltaContactsRightOdo;
        passedContactsMiddleOdo += deltaContactsMiddleOdo;

        //Get position change
        double[] positionChange = Odometry.getPositionChange(deltaContactsRightOdo, deltaContactsLeftOdo, deltaContactsMiddleOdo, robotRotation);
        deltaX = positionChange[0];
        deltaY = positionChange[1];
        deltaRotation = positionChange[2];

        //Update position
        posX += deltaX;
        posY += deltaY;
        robotRotation += deltaRotation;

        //if input
        if((joystickX <= -deadzone || joystickX >= deadzone) && (joystickY <= -deadzone || joystickY >= deadzone)) {
            timeAccelerating += deltaTime;
            double joystickAngle = Math.atan2(joystickX, joystickY);
            double moveAngle = ToolBox.joystickToRobot(joystickAngle, robotRotation);
            double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle);
            double magnitude = ToolBox.pythagoras(joystickX, joystickY);

            backLeftMotor.setPower(Range.clip(motorPowers[0] * magnitude + rotate, -1, 1));
            backRightMotor.setPower(Range.clip(motorPowers[1] * magnitude + rotate, -1, 1));
            frontLeftMotor.setPower(Range.clip(motorPowers[2] * magnitude + rotate, -1, 1));
            frontRightMotor.setPower(Range.clip(motorPowers[3] * magnitude + rotate, -1, 1));
        }
        else {
            timeAccelerating = 0;

            //if moving break
            double breakingPower = 0.05;
            if(deltaX > 0 || deltaY > 0 || deltaRotation > 0) {
                backLeftMotor.setPower(breakingPower);
                backRightMotor.setPower(breakingPower);
                frontLeftMotor.setPower(breakingPower);
                frontRightMotor.setPower(breakingPower);
            }
        }




        //odometry test - move back to origin on Y
        int targetX = 0;
        int targetY = 0;
        double speed = 0.5;
        if(gamepad1.y) {
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
        telemetry.addData("Direction Angle", robotRotation);
    }
}
