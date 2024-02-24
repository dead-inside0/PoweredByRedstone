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
        double[] motorPowers = ToolBox.getMotorPowersByDirection(0, 0.7,0);
        while(opModeIsActive() && runtime.seconds() < 0.5) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }

        while(opModeIsActive() && runtime.seconds() < 1) {
            backLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            frontLeftMotor.setPower(-1);
            frontRightMotor.setPower(-1);
        }

        pickUpMotor.setPower(1);

        while(opModeIsActive() && runtime.seconds() < 1.5) {
            backLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            frontRightMotor.setPower(1);
        }
        motorPowers = ToolBox.getMotorPowersByDirection(Math.PI, 0.7,0);
        while(opModeIsActive() && runtime.seconds() < 2) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        motorPowers = ToolBox.getMotorPowersByDirection(Math.PI/2, 0.7,0);
        while(opModeIsActive() && runtime.seconds() < 4) {
            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
    }
}
