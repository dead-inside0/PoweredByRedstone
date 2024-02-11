package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.ToolBox;

@Autonomous(name="AutonomousPathTest", group="Tests")
public class AutonomousPathTest extends LinearOpMode{

    final private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
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

    private boolean checkIfAtPosition(double selfX, double selfY, double targetX, double targetY,double selfRot,double targetRot){
        double tolerance = 10;
        double rotTolerance = Math.PI/40;
        return Math.abs(selfX - targetX) < tolerance && Math.abs(selfY - targetY) < tolerance && Math.abs(selfRot - targetRot) < rotTolerance;
    }
    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        backLeftMotor = hMap.backLeftMotor;
        backRightMotor = hMap.backRightMotor;
        frontLeftMotor = hMap.frontLeftMotor;
        frontRightMotor = hMap.frontRightMotor;
        for (int i = 0; i < path.length; i++) {
            telemetry.addData("Next point: ", "X: %d, Y: %d, R: %d", path[i][0], path[i][1], path[i][2]);
            while(!checkIfAtPosition(posX, posY, path[i][0], path[i][1], robotRotation, path[i][2])){
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

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, path[i][0], path[i][1], robotRotation, path[i][2], 1);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);
            }
        }
    }
}
