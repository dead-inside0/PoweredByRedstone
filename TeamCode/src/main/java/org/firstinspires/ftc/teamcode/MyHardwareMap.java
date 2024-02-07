package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyHardwareMap {
    HardwareMap hdwMap = null;

    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor,frontRightMotor;

    public MyHardwareMap(HardwareMap map){
        hdwMap = map;

        backLeftMotor = hdwMap.get(DcMotor.class, "motor0");
        backRightMotor = hdwMap.get(DcMotor.class, "motor1");
        frontLeftMotor = hdwMap.get(DcMotor.class, "motor2");
        frontRightMotor = hdwMap.get(DcMotor.class, "motor3");

    }
}
