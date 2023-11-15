package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Test motor 1", group="Linear OpMode")
@Disabled
public class TestMotor1 extends LinearOpMode {
    MyHardwareMap map = new MyHardwareMap();

    private DcMotor motor = null;

    @Override
    public void runOpMode() {
        motor = map.motor1;
        motor.setPower(20);
        motor.setDirection((DcMotorSimple.Direction.FORWARD));
    }
}
