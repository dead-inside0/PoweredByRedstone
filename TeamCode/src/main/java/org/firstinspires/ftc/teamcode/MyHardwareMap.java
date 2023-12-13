package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MyHardwareMap {
    HardwareMap hdwMap = null;

    public DcMotor motor1,motor2,motor3,motor4;

    public DcMotor encR,encL,encF;

    public MyHardwareMap(HardwareMap map){
        hdwMap = map;

        motor1 = hdwMap.get(DcMotor.class, "motor1");
        motor2 = hdwMap.get(DcMotor.class, "motor2");
        motor3 = hdwMap.get(DcMotor.class, "motor3");
        motor4 = hdwMap.get(DcMotor.class, "motor4");

    }
}
