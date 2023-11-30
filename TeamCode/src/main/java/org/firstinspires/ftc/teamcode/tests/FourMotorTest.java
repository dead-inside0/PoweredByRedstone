//note: Motors config works only for the test rig. As the naming refers to the positions of the motor on the rig.
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Four-Motor joystick test")
@Disabled
public class FourMotorTest extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();

    final private MyHardwareMap hMap = new MyHardwareMap(hardwareMap);
    final private DcMotor backleftMotor = hMap.motor1;
    final private DcMotor backrightMotor = hMap.motor2;
    final private DcMotor frontleftMotor = hMap.motor4;
    final private DcMotor frontrightMotor = hMap.motor3;

    @Override
    public void init() {
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
        final double forward = -gamepad1.left_stick_y;
        final double rotate = gamepad1.left_stick_x;
        backleftMotor.setPower(forward-rotate);
        backrightMotor.setPower(forward+rotate);
        frontleftMotor.setPower(forward-rotate);
        frontrightMotor.setPower(forward+rotate);
        telemetry.addData("Joystick X", rotate);
        telemetry.addData("Joystick Y", forward);
        telemetry.addData("Runtime", runtime.toString());
    }
}

