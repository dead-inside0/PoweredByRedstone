package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MyHardwareMap {
    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor,frontRightMotor;

    public MyHardwareMap(HardwareMap map){
        backLeftMotor = map.get(DcMotor.class, "motor0");
        backRightMotor = map.get(DcMotor.class, "motor1");
        frontLeftMotor = map.get(DcMotor.class, "motor2");
        frontRightMotor = map.get(DcMotor.class, "motor3");
    }
}
