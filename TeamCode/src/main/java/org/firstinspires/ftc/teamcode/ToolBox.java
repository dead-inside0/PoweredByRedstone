package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class ToolBox {
    public static double movementTolerance = 30;

    public static double movementDecel = 150;

    public static double rotateTolerance = Math.PI/140;

    //converts the joystick angle (global) to the angle needed to move the robot (local) in that direction
    public static double globalToRobot(double joystickAngle, double robotAngle){
        double localAngle = joystickAngle - robotAngle;
        return scaleAngle(localAngle);
    }

    //returns motor powers needed to go in a specific angle https://seamonsters-2605.github.io/archive/mecanum/
    public static double[] getMotorPowersByDirection(double targetDirectionAngle, double moveSpeed, double rotate){
        targetDirectionAngle += Math.PI/2;
        targetDirectionAngle = scaleAngle(targetDirectionAngle);

        double motorPowerBlue = Math.sin(targetDirectionAngle + Math.PI / 4) * moveSpeed;
        double motorPowerRed = Math.sin(targetDirectionAngle - Math.PI / 4) * moveSpeed;

        double maxMotorPower = Math.max(motorPowerBlue, motorPowerRed);
        double factor = 1;
        if(Math.abs(maxMotorPower) + rotate > 1){
            factor = Math.abs(maxMotorPower) + rotate;
        }

        double[] motorPowers = { // motor powers are scaled so the max power < 1
                (-motorPowerBlue - rotate) / factor, //backleft
                (motorPowerRed - rotate) / factor, //backright
                (-motorPowerRed - rotate) / factor, //frontleft
                (motorPowerBlue - rotate) / factor //frontright
        };

        return motorPowers;
    }

    //Get motor powers to drive to a specific point
    public static double[] getMotorPowersToPoint(double currentX, double currentY, double targetX, double targetY, double currentRot, double targetRot, double speed){
        double angleToTarget = globalToRobot(Math.atan2(targetX - currentX, targetY - currentY),currentRot);

        double rotate = 0;
        double rotateSpeed = 1;

        double rotDiff = scaleAngle(targetRot - currentRot);
        if(rotDiff > Math.PI) {
            rotDiff -= 2 * Math.PI;
            rotate = -0.25;
        }
        else {
            rotate = 0.25;
        }
        //rotate = Range.clip(rotDiff * 5/Math.PI,-1,1);

        if(pythagoras(targetX - currentX, targetY - currentY) < movementDecel) {
            speed = Range.clip(pythagoras(targetX - currentX, targetY - currentY) / movementDecel,0.2,speed);
        }

        if(pythagoras(targetX - currentX, targetY - currentY) > movementTolerance){
            rotateSpeed = 0;
        }
        //Rotate either - or + based on difference in angles
        else if(Math.abs(rotDiff) > rotateTolerance){
            speed = 0;
        }
        else {
            rotate = 0;
            speed = 0;
        }

        return getMotorPowersByDirection(angleToTarget, speed, rotate * rotateSpeed);
    }

    //pythagoras theorem
    public static double pythagoras(double x, double y){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    //Makes sure angle is between 0 and 2PI
    public static double scaleAngle(double angle){
        if(angle > 2*Math.PI){
            double newAngle = angle;
            while(newAngle > 2*Math.PI){
                newAngle -= 2*Math.PI;
            }
            return newAngle;
        }
        else if(angle < 0){
            double newAngle = angle;
            while(newAngle < 0){
                newAngle += 2*Math.PI;
            }
            return newAngle;
        }
        else{
            return angle;
        }
    }
}
