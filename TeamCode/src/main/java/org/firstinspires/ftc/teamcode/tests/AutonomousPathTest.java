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



    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;

        int passedContactsRightOdo = backRightMotor.getCurrentPosition();
        int passedContactsLeftOdo = backLeftMotor.getCurrentPosition();
        int passedContactsMiddleOdo = frontLeftMotor.getCurrentPosition();

        for (double[] point : path) {
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
            while (!(Math.abs(posX - point[0]) < ToolBox.movementTolerance && Math.abs(posY - point[1]) < ToolBox.movementTolerance && Math.abs(robotRotation - point[2]) < ToolBox.rotateTolerance)) {
                int deltaContactsLeftOdo = -backLeftMotor.getCurrentPosition() - passedContactsLeftOdo;
                int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - passedContactsRightOdo;
                int deltaContactsMiddleOdo = frontLeftMotor.getCurrentPosition() - passedContactsMiddleOdo;

                //Update passed odo contacts
                passedContactsRightOdo += deltaContactsRightOdo;
                passedContactsLeftOdo += deltaContactsLeftOdo;
                passedContactsMiddleOdo += deltaContactsMiddleOdo;
                //Get position change
                double[] positionChange = Odometry.getPositionChange(deltaContactsRightOdo, deltaContactsLeftOdo, deltaContactsMiddleOdo);
                double deltaX = positionChange[0];
                double deltaY = positionChange[1];
                double deltaRotation = positionChange[2];

                //Update position
                posX += deltaX;
                posY += deltaY;
                robotRotation += deltaRotation;
                robotRotation = ToolBox.scaleAngle(robotRotation);

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, point[0], point[1], 0, 0, 0.5);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);

                telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
                telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
                telemetry.addData("test",passedContactsMiddleOdo);
                telemetry.update();
            }
        }
    }
}
