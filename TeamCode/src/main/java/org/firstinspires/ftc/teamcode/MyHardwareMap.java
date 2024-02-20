package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MyHardwareMap {
    public DcMotor backLeftMotor,backRightMotor, frontLeftMotor,frontRightMotor, linearMechanismMotor, pickUpMotor, hookMotor, leftOdo, rightOdo, middleOdo;
    public Servo droneServo, placeServo, hookServo;

    public MyHardwareMap(HardwareMap map){
        backLeftMotor = map.get(DcMotor.class, "motor0");
        backRightMotor = map.get(DcMotor.class, "motor1");
        frontLeftMotor = map.get(DcMotor.class, "motor2");
        frontRightMotor = map.get(DcMotor.class, "motor3");

        linearMechanismMotor = map.get(DcMotor.class, "motor4");
        pickUpMotor = map.get(DcMotor.class, "motor5");
        hookMotor = map.get(DcMotor.class, "motor6");

        droneServo = map.get(Servo.class, "servo0");
        placeServo = map.get(Servo.class, "servo1");
        hookServo = map.get(Servo.class, "servo2");

        leftOdo = map.get(DcMotor.class, "motor0");
        middleOdo = map.get(DcMotor.class, "motor1");
        rightOdo = map.get(DcMotor.class, "motor4");
    }
}
