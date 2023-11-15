package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="Test motor 1")
public class TestMotor1 extends OpMode {
    //MyHardwareMap map = new MyHardwareMap();

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor = null;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor1");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        runtime.reset();
        motor.setPower(0.5);
    }

    @Override
    public void loop() {

    }
}
