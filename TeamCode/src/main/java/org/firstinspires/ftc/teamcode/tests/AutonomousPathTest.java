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

    final private double[][] path = {
            {50, 0, 0},
            {50,100, 0},
            {-50,100,0},
            {-50,0,0},
            {0,0,0}
    };

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    int passedContactsRightOdo = 0;
    int passedContactsLeftOdo = 0;
    int passedContactsMiddleOdo = 0;

    private boolean checkIfAtPosition(double selfX, double selfY, double targetX, double targetY,double selfRot,double targetRot){
        double tolerance = 5;
        double rotTolerance = Math.PI/180;
        return Math.abs(selfX - targetX) < tolerance && Math.abs(selfY - targetY) < tolerance && Math.abs(selfRot - targetRot) < rotTolerance;
    }
    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;
        for (double[] doubles : path) {
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", doubles[0], doubles[1], doubles[2]);
            telemetry.update();
            while (!checkIfAtPosition(posX, posY, doubles[0], doubles[1], robotRotation, doubles[2])) {
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

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, doubles[0], doubles[1], robotRotation, doubles[2], 0.5);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);
            }
        }
    }
}
