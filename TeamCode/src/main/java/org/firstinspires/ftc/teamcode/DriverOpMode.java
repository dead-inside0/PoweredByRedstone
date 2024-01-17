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
    private DcMotor backleftMotor;
    private DcMotor backrightMotor;
    private DcMotor frontrightMotor;
    private DcMotor frontleftMotor;

    double posX = 0;
    double posY = 0;
    double directionAngle = 0;

    @Override
    public void init() {
        //get motors from hardware map
        hMap = new MyHardwareMap(hardwareMap);
        backleftMotor = hMap.motor1; //odo1
        backrightMotor = hMap.motor2; //odo2
        frontrightMotor = hMap.motor3; //odo3
        frontleftMotor = hMap.motor4;
        //set directions
        backleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontrightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //get gamepad input
        double forward = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        //move robot
        backleftMotor.setPower(Range.clip(forward - side + rotate, -1.0, 1.0));
        backrightMotor.setPower(Range.clip(forward + side - rotate, -1.0, 1.0));
        frontleftMotor.setPower(Range.clip(forward + side + rotate, -1.0, 1.0));
        frontrightMotor.setPower(Range.clip(forward - side - rotate, -1.0, 1.0));

        //update position
        double contactsRightOdo = backrightMotor.getCurrentPosition();
        double contactsLeftOdo = backleftMotor.getCurrentPosition();
        double contactsMiddleOdo = frontrightMotor.getCurrentPosition();
        double[] positionChange = Odometry.getPositionChange(contactsRightOdo, contactsLeftOdo, contactsMiddleOdo, directionAngle);
        posX += positionChange[0];
        posY += positionChange[1];
        directionAngle += positionChange[2];
        //output data
        //telemetry.addData("Joystick X", side);
        //telemetry.addData("Joystick Y", forward);
        //telemetry.addData("Runtime", runtime.toString());
        telemetry.addData("Position X", posX);
        telemetry.addData("Position Y", posY);
        telemetry.addData("Direction Angle", directionAngle);
    }
}
