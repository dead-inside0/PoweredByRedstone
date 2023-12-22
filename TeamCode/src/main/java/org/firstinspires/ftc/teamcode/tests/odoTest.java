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

    double[] getCurrentPos(double[] prevPos, double deltaRightOdo, double deltaLeftOdo, double deltaMiddleOdo){
        final double sideSensorsDistance = 0.3;

        double[] currentPos = {0, 0, 0}; //x, y, angle
        return currentPos;
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
