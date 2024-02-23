package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.ToolBox;

@Autonomous
public class AutOp extends LinearOpMode {
    double posX = 0;
    double posY = 0;
    double robotRotation = 0;

    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;
        DcMotor leftOdo = hMap.leftOdo;
        DcMotor middleOdo = hMap.middleOdo;
        DcMotor rightOdo = hMap.rightOdo;
        int passedContactsRightOdo = rightOdo.getCurrentPosition();
        int passedContactsLeftOdo = leftOdo.getCurrentPosition();
        int passedContactsMiddleOdo = middleOdo.getCurrentPosition();
        waitForStart();
        while (!(ToolBox.pythagoras(0-posX,400-posY) < ToolBox.movementTolerance && Math.abs(0-robotRotation) < ToolBox.rotateTolerance)) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 400, robotRotation, 0, 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        while (!(ToolBox.pythagoras(400-posX,400-posY) < ToolBox.movementTolerance && Math.abs(0-robotRotation) < ToolBox.rotateTolerance)) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 400, 400, robotRotation, 0, 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        while (!(ToolBox.pythagoras(400-posX,0-posY) < ToolBox.movementTolerance && Math.abs(0-robotRotation) < ToolBox.rotateTolerance)) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 400, 0, robotRotation, 0, 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        while (!(ToolBox.pythagoras(0-posX,0-posY) < ToolBox.movementTolerance && Math.abs(0-robotRotation) < ToolBox.rotateTolerance)) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, 0, 0, robotRotation, 0, 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
    }
}
