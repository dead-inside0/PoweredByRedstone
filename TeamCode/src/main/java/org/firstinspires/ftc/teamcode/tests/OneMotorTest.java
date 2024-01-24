package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="One motor")
public class OneMotorTest extends OpMode {
    DcMotor motor;
    MyHardwareMap hMap;

    @Override
    public void init() {
        hMap = new MyHardwareMap(hardwareMap);
        motor = hMap.motor1;
    }

    @Override
    public void loop(){
        double joystickY = -gamepad1.left_stick_y;
        motor.setPower(joystickY);
    }
}
