//note: Motors config works only for the test rig. As the naming refers to the positions of the motor on the rig.
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.radians;
import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="odoTest")
public class odoTest extends OpMode {
    final private ElapsedTime runtime = new ElapsedTime();
    private MyHardwareMap hMap;
    private DcMotor backleftMotor; //odo left
    private DcMotor backrightMotor; //odo right
    private DcMotor frontrightMotor; //odo middle
    private DcMotor frontleftMotor;
    @Override
    public void init() {
        //get motors from hardware map
        hMap = new MyHardwareMap(hardwareMap);
        backleftMotor = hMap.motor1; //odo left
        backrightMotor = hMap.motor2; //odo right
        frontrightMotor = hMap.motor3; //odo middle
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

    double[] getPositionChange(double contactsRightOdo, double contactsLeftOdo, double contactsMiddleOdo, double prevAngle){
        final double sideOdosDistance = 306.69;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        double rightArcLength = wheelCircumference * contactsRightOdo / sensorResolution;
        double leftArcLength = wheelCircumference * contactsLeftOdo / sensorResolution;

        double centerArcLength = (rightArcLength + leftArcLength) / 2;
        double centerArcAngle = (rightArcLength - leftArcLength) / sideOdosDistance;
        double directionAngle  = centerArcAngle / 2;

        double centerArcRadius = (360 * centerArcLength) / (centerArcAngle * 2 * Math.PI);
        double shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(centerArcAngle));

        double deltaX = shiftLength * Math.cos(prevAngle + directionAngle);
        double deltaY = shiftLength * Math.sin(prevAngle + directionAngle);

        double strafing = wheelCircumference * contactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + directionAngle);
        double strafingDeltaY = strafing * Math.cos(prevAngle + directionAngle);

        double totalDeltaX = deltaX + strafingDeltaX;
        double totalDeltaY = deltaY + strafingDeltaY;

        double[] positionChange = {totalDeltaX, totalDeltaY, directionAngle}; //x, y, angle
        return positionChange;
    }

    @Override
    public void loop() {
        //get gamepad input
        double forward = -gamepad1.left_stick_y;
        double side = gamepad1.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        //move robot
        backleftMotor.setPower(Range.clip(forward-side+rotate,-1.0,1.0));
        backrightMotor.setPower(Range.clip(forward+side-rotate,-1.0,1.0));
        frontleftMotor.setPower(Range.clip(forward+side+rotate,-1.0,1.0));
        frontrightMotor.setPower(Range.clip(forward-side-rotate,-1.0,1.0));
    }
}
