package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Odometry test")
public class odoTest extends OpMode {

    public DcMotor odo;
    @Override
    public void init() {
        odo = hardwareMap.get(DcMotor.class, "odo enc");
        odo.setDirection(DcMotor.Direction.REVERSE);
        odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            odo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        telemetry.addData("Encoder", odo.getCurrentPosition());
    }
}
