package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.ToolBox;

@Autonomous
public class Park extends LinearOpMode {
    final private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        runtime.reset();
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;
        DcMotor pickUpMotor = hMap.pickUpMotor;

        waitForStart();
        runtime.reset();
        double actionStart = runtime.seconds();
        double[] motorPowers = ToolBox.getMotorPowersByDirection(0, 0.75,0);
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        actionStart = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

        actionStart = runtime.seconds();
        motorPowers = ToolBox.getMotorPowersByDirection(0, 0,-0.75);
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        pickUpMotor.setPower(1);

        actionStart = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < actionStart + 1) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

        pickUpMotor.setPower(0);

        actionStart = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

        actionStart = runtime.seconds();
        motorPowers = ToolBox.getMotorPowersByDirection(0, 0,0.75);
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        actionStart = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

        actionStart = runtime.seconds();
        motorPowers = ToolBox.getMotorPowersByDirection(Math.PI, 0.75,0);
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        actionStart = runtime.seconds();
        while(opModeIsActive() && runtime.seconds() < actionStart + 0.5) {
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

        actionStart = runtime.seconds();
        motorPowers = ToolBox.getMotorPowersByDirection(Math.PI/2, 0.7,0);
        while(opModeIsActive() && runtime.seconds() < actionStart + 1) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
    }
}
