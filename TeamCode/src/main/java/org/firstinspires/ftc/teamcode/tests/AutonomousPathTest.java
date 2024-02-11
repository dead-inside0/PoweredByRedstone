package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.ToolBox;

public class AutonomousPathTest extends LinearOpMode{

    final private ElapsedTime runtime = new ElapsedTime();


    final private MyHardwareMap hMap = new MyHardwareMap(hardwareMap);
    final private DcMotor backLeftMotor = hMap.backLeftMotor;
    final private DcMotor backRightMotor = hMap.backRightMotor;
    final private DcMotor frontLeftMotor = hMap.frontLeftMotor;
    final private DcMotor frontRightMotor = hMap.frontRightMotor;
    private double[][] path = {
            {500, 0, 0},
            {500,1000, 0},
            {-500,1000,0},
            {-500,0,0},
            {0,0,0}
    };

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    private boolean checkIfAtPosition(double selfX, double selfY, double targetX, double targetY) {
        double tolerance = 10;
        return Math.abs(selfX - targetX) < tolerance && Math.abs(selfY - targetY) < tolerance;
    }
    public void runOpMode() {
        for (int i = 0; i < path.length; i++) {
            while(!checkIfAtPosition(posX, posY, path[i][0], path[i][1])){
                int deltaContactsLeftOdo = backLeftMotor.getCurrentPosition() - passedContactsLeftOdo;
                int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - passedContactsRightOdo;
                int deltaContactsMiddleOdo = frontLeftMotor.getCurrentPosition() - passedContactsMiddleOdo;

                //Update passed odo contacts
                passedContactsLeftOdo += deltaContactsLeftOdo;
                passedContactsRightOdo += deltaContactsRightOdo;
                passedContactsMiddleOdo += deltaContactsMiddleOdo;

                //Get position change
                double[] positionChange = Odometry.getPositionChange(deltaContactsRightOdo, deltaContactsLeftOdo, deltaContactsMiddleOdo, robotRotation);
                double deltaX = positionChange[0];
                double deltaY = positionChange[1];
                double deltaRotation = positionChange[2];

                //Update position
                posX += deltaX;
                posY += deltaY;
                robotRotation += deltaRotation;
                robotRotation = ToolBox.clampAngle(robotRotation);

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, path[i][0], path[i][1], 1,0);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);
            }
        }
    }
}
