//note: Motors config works only for the test rig. As the naming refers to the positions of the motor on the rig.
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Four-Motor joystick test")
public class FourMotorTest extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();

    private MyHardwareMap hMap;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;

    @Override
    public void init() {
        hMap = new MyHardwareMap(hardwareMap);
        backLeftMotor = hMap.motor1;
        backRightMotor = hMap.motor2;
        frontLeftMotor = hMap.motor4;
        frontRightMotor = hMap.motor3;
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double joystickY = -gamepad1.left_stick_y;
        double joystickX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double maxPower = 0.75;
        if(gamepad1.right_trigger>0.9){
            maxPower = 1.0;
        } else if (gamepad1.left_trigger > 0.9) {
            maxPower = 0.5;
        }
        //move robot in local direction
        backLeftMotor.setPower(Range.clip(joystickY - joystickX + rotate, -maxPower, maxPower));
        backRightMotor.setPower(Range.clip(joystickY + joystickX - rotate, -maxPower, maxPower));
        frontLeftMotor.setPower(Range.clip(joystickY + joystickX + rotate, -maxPower, maxPower));
        frontRightMotor.setPower(Range.clip(joystickY - joystickX - rotate, -maxPower, maxPower));
        telemetry.addData("Joystick X", joystickX);
        telemetry.addData("Joystick Y", joystickY);
        telemetry.addData("Runtime", runtime.toString());
    }
}

