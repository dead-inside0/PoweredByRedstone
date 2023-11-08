package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OgHardwareMap {
    HardwareMap hdwMap = null;

    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;
    public DcMotor motor4 = null;

    public void init(HardwareMap ahwMap){
        hdwMap = ahwMap;

        motor1 = hdwMap.get(DcMotor.class, "leftFront");
        motor2 = hdwMap.get(DcMotor.class, "rightFront");
        motor3 = hdwMap.get(DcMotor.class, "leftBack");
        motor4 = hdwMap.get(DcMotor.class, "rightBack");


    }
}
