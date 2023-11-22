package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Two-Motor joystick test")
public class TwoMotorTest extends OpMode {

    private MyHardwareMap hMap = new MyHardwareMap();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;

    @Override
    public void init() {
        //Left
        //leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        leftMotor = hMap.motor1;
        leftMotor = hardwareMap.get(DcMotor.class, "motor1");
        //Right
        rightMotor = hardwareMap.get(DcMotor.class, "motor2");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.left_stick_x;
        leftMotor.setPower(forward-rotate)  ;
        rightMotor.setPower(forward+rotate);
        telemetry.addData("Joystick X", rotate);
        telemetry.addData("Joystick Y", forward);
        telemetry.addData("Runtime", runtime.toString());
    }
}
