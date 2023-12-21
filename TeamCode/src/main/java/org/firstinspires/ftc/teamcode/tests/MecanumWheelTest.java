//note: Motors config works only for the test rig. As the naming refers to the positions of the motor on the rig.
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Mecanum wheel test")
public class MecanumWheelTest extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();

    private MyHardwareMap hMap;
    private DcMotor backleftMotor;
    private DcMotor backrightMotor;
    private DcMotor frontleftMotor;
    private DcMotor frontrightMotor;
    @Override
    public void init() {
        hMap = new MyHardwareMap(hardwareMap);
        backleftMotor = hMap.motor1;
        backrightMotor = hMap.motor2;
        frontleftMotor = hMap.motor4;
        frontrightMotor = hMap.motor3;
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
        double side = gamepad1.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        backleftMotor.setPower(Range.clip(forward-side+rotate,-1.0,1.0));
        backrightMotor.setPower(Range.clip(forward+side-rotate,-1.0,1.0));
        frontleftMotor.setPower(Range.clip(forward+side+rotate,-1.0,1.0));
        frontrightMotor.setPower(Range.clip(forward-side-rotate,-1.0,1.0));
        telemetry.addData("Joystick X", side);
        telemetry.addData("Joystick Y", forward);
        telemetry.addData("Runtime", runtime.toString());
    }
}

