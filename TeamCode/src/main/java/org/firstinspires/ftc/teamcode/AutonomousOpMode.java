package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutonomousOpMode extends LinearOpMode {

    final private ElapsedTime runtime = new ElapsedTime();


    final private MyHardwareMap hMap = new MyHardwareMap(hardwareMap);
    final private DcMotor backLeftMotor = hMap.backLeftMotor;
    final private DcMotor backRightMotor = hMap.backRightMotor;
    final private DcMotor frontLeftMotor = hMap.frontLeftMotor;
    final private DcMotor frontRightMotor = hMap.frontRightMotor;

    /*
        top left=0
        bottom left=1
        top right=2
        bottom right=3
     */
    private int position = 0;

    /*
        left=0
        center=1
        right=2
     */
    int pixelLocation() {
        return 0;
    };

    public void runOpMode() {

    }
}
