package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Two motor")
public class TwoMotorTest extends OpMode {
    DcMotor motorRight;
    DcMotor motorLeft;

    @Override
    public void init() {
        motorRight = hardwareMap.get(DcMotor.class, "motor0");
        motorLeft = hardwareMap.get(DcMotor.class, "motor1");
    }

    @Override
    public void loop(){
        double joystickY = -gamepad1.left_stick_y;
        motorRight.setPower(joystickY);
        motorLeft.setPower(joystickY);

        telemetry.addData("Encoders passed motor right", motorRight.getCurrentPosition());
        telemetry.addData("Encoders passed motor 1", motorLeft.getCurrentPosition());
    }
}
