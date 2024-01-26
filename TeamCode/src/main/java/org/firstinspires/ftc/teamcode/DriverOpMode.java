package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver")
public class DriverOpMode extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private MyHardwareMap hMap;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;

    double posX = 0;
    double posY = 0;
    double robotAngle = 0;

    int contactsRightOdo = 0;
    int contactsLeftOdo = 0;
    int contactsMiddleOdo = 0;

    @Override
    public void init() {
        //get motors from hardware map
        hMap = new MyHardwareMap(hardwareMap);
        backLeftMotor = hMap.motor1; //odo1
        backRightMotor = hMap.motor2; //odo2
        frontRightMotor = hMap.motor3; //odo3
        frontLeftMotor = hMap.motor4;
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //get gamepad input
        double joystickY = -gamepad1.left_stick_y;
        double joystickX = gamepad1.left_stick_x;
        double rotate = gamepad2.right_stick_x;


        //move robot in local direction
        //backLeftMotor.setPower(Range.clip(joystickY - joystickX + rotate, -1.0, 1.0));
        //backRightMotor.setPower(Range.clip(joystickY + joystickX - rotate, -1.0, 1.0));
        //frontLeftMotor.setPower(Range.clip(joystickY + joystickX + rotate, -1.0, 1.0));
        //frontRightMotor.setPower(Range.clip(joystickY - joystickX - rotate, -1.0, 1.0));


        //update position
        int deltaContactsRightOdo = backRightMotor.getCurrentPosition() - contactsRightOdo;
        int deltaContactsLeftOdo = backLeftMotor.getCurrentPosition() - contactsLeftOdo;
        int deltaContactsMiddleOdo = frontRightMotor.getCurrentPosition() - contactsMiddleOdo;
        contactsRightOdo += deltaContactsRightOdo;
        contactsLeftOdo += deltaContactsLeftOdo;
        contactsMiddleOdo += deltaContactsMiddleOdo;
        double[] positionChange = Odometry.getPositionChange(deltaContactsRightOdo, deltaContactsLeftOdo, deltaContactsMiddleOdo, robotAngle);
        posX += positionChange[0];
        posY += positionChange[1];
        robotAngle += positionChange[2];


        //move robot
        double joystickAngle = Math.atan2(joystickX, joystickY);
        double moveAngle = ToolBox.joystickToRobot(joystickAngle, robotAngle);
        double[] motorPowers = ToolBox.getMotorPowersByDirection(moveAngle);
        double magnitude = ToolBox.pythagoras(joystickX, joystickY);
        backLeftMotor.setPower(Range.clip(motorPowers[0]+rotate, -1, 1) * magnitude);
        backRightMotor.setPower(Range.clip(motorPowers[1]+rotate, -1, 1) * magnitude);
        frontLeftMotor.setPower(Range.clip(motorPowers[2]+rotate, -1, 1) * magnitude);
        frontRightMotor.setPower(Range.clip(motorPowers[3]+rotate, -1, 1) * magnitude);


        //output data
        //telemetry.addData("Joystick X", joystickX);
        //telemetry.addData("Joystick Y", joystickY);
        //telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Direction Angle", robotAngle);
    }
}
