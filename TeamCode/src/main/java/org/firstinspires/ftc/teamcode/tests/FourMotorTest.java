package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Four-Motor joystick test")
public class FourMotorTest extends OpMode {

    private MyHardwareMap hMap = new MyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleftMotor = null;
    private DcMotor backrightMotor = null;

    private DcMotor frontleftMotor = null;

    private DcMotor frontrightMotor = null;

    @Override
    public void init() {
        //Left Back
        //leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backleftMotor = hMap.motor1;
        frontleftMotor = hMap.motor3;
        //Right Back
        //rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backrightMotor = hMap.motor2;
        frontrightMotor = hMap.motor4;

        backleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.left_stick_x;
        backleftMotor.setPower(forward-rotate);
        backrightMotor.setPower(forward+rotate);
        frontleftMotor.setPower(forward-rotate);
        frontrightMotor.setPower(forward+rotate);
        telemetry.addData("Joystick X", rotate);
        telemetry.addData("Joystick Y", forward);
        telemetry.addData("Runtime", runtime.toString());
    }
}

