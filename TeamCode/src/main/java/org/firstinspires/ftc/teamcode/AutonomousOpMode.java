package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.ToolBox;

public class AutonomousOpMode extends LinearOpMode{

    final private ElapsedTime runtime = new ElapsedTime();

    private double[][] path;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;



    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;

        Servo droneServo = hMap.droneServo;

        DcMotor linearMechanismMotor = hMap.linearMechanismMotor;

        DcMotor leftOdo = hMap.leftOdo;
        DcMotor middleOdo = hMap.middleOdo;
        DcMotor rightOdo = hMap.rightOdo;

        int passedContactsRightOdo = rightOdo.getCurrentPosition();
        int passedContactsLeftOdo = leftOdo.getCurrentPosition();
        int passedContactsMiddleOdo = middleOdo.getCurrentPosition();




        waitForStart();

        int elementLocation = getElementLocation();

        switch (elementLocation) {
            case 0:
                break;
            case 1:
                break;
            default:
                break;
        }


        for (double[] point : path) {
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
            while (!(Math.abs(posX - point[0]) < ToolBox.movementTolerance && Math.abs(posY - point[1]) < ToolBox.movementTolerance && Math.abs(robotRotation - point[2]) < ToolBox.rotateTolerance)) {
                int deltaContactsLeftOdo = leftOdo.getCurrentPosition() - passedContactsLeftOdo;
                int deltaContactsRightOdo = rightOdo.getCurrentPosition() - passedContactsRightOdo;
                int deltaContactsMiddleOdo = middleOdo.getCurrentPosition() - passedContactsMiddleOdo;

                //Update passed odo contacts
                passedContactsRightOdo += deltaContactsRightOdo;
                passedContactsLeftOdo += deltaContactsLeftOdo;
                passedContactsMiddleOdo += deltaContactsMiddleOdo;
                //Get position change
                double[] positionChange = Odometry.getPositionChange(-deltaContactsRightOdo, deltaContactsLeftOdo, -deltaContactsMiddleOdo, robotRotation);
                double deltaX = positionChange[0];
                double deltaY = positionChange[1];
                double deltaRotation = positionChange[2];

                //Update position
                posX += deltaX;
                posY += deltaY;
                robotRotation += deltaRotation;
                robotRotation = ToolBox.scaleAngle(robotRotation);

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, point[0], point[1], robotRotation, point[2], 0.5);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);

                telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
                telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
                telemetry.update();
            }
        }
    }

    int getElementLocation() {
        return 0;
    }
}
