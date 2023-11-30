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
    private MyHardwareMap hMap = new MyHardwareMap(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backleftMotor = null;
    private DcMotor backrightMotor = null;
    private DcMotor frontleftMotor = null;
    private DcMotor frontrightMotor = null;

    @Override
    public void init() {
        //backleftMotor = hardwareMap.get(DcMotor.class, "motor1");
        backleftMotor = hMap.motor1;
        //frontleftMotor = hardwareMap.get(DcMotor.class, "motor4");
        frontleftMotor = hMap.motor4;
        //backrightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backrightMotor = hMap.motor2;
        //frontrightMotor = hardwareMap.get(DcMotor.class, "motor3");
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
        double rotate = gamepad1.right_stick_x;
        backleftMotor.setPower(Range.clip(forward-side+rotate,-1.0,1.0));
        backrightMotor.setPower(Range.clip(forward+side-rotate,-1.0,1.0));
        frontleftMotor.setPower(Range.clip(forward+side+rotate,-1.0,1.0));
        frontrightMotor.setPower(Range.clip(forward-side-rotate,-1.0,1.0));
        telemetry.addData("Joystick X", side);
        telemetry.addData("Joystick Y", forward);
        telemetry.addData("Runtime", runtime.toString());
    }

}

