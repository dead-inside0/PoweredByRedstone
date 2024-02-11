package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MyHardwareMap;

@TeleOp(name="One motor")
public class OneMotorTest extends OpMode {
    DcMotor motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor0");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        double joystickY = -gamepad1.left_stick_y;
        motor.setPower(joystickY);

        if(gamepad1.b){
            if(motor.getDirection() == DcMotorSimple.Direction.FORWARD){
                motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            else{
                motor.setDirection(DcMotorSimple.Direction.FORWARD);
            }
        }

        telemetry.addData("Encoders passed", motor.getCurrentPosition());
        telemetry.addData("Motor direction", motor.getDirection());
    }
}
