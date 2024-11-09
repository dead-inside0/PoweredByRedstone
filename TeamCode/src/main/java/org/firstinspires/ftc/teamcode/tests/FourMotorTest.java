package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ToolBox;

@TeleOp(name="Four motor")
public class FourMotorTest extends OpMode {
    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor, frontRightMotor;

    public double rotateMultiplier = 0.8;

    @Override
    public void init() {
        //get motors from hardware map
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor2");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor3");

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

        //output data
        telemetry.addData("Joystick X", joystickX);
        telemetry.addData("Joystick Y", joystickY);
        telemetry.addData("Rotate joystick", rotate);
        telemetry.addData("Joystick angle", Math.atan2(joystickX, joystickY));

        telemetry.addData("MotorPowerBackLeft", backLeftMotor.getPower());
        telemetry.addData("MotorPowerBackRight", backRightMotor.getPower());
        telemetry.addData("MotorPowerFrontLeft", frontLeftMotor.getPower());
        telemetry.addData("MotorPowerFrontRight", frontRightMotor.getPower());

    }
}
