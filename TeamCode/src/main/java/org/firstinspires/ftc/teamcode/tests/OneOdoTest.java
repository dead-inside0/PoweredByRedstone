package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="One Odo Test")
public class OneOdoTest extends OpMode {
    private MyHardwareMap hMap;
    private DcMotor odo;

    @Override
    public void init(){
        hMap = new MyHardwareMap(hardwareMap);
        odo = hMap.backLeftMotor;
    }

    @Override
    public void loop(){
        telemetry.addData("Motor Position", odo.getCurrentPosition());
    }
}
