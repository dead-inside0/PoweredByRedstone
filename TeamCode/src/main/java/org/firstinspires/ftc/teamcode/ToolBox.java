package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class ToolBox {
    public static double movementTolerance = 10;

    public static double movementDecelerationDistance = 200;
    public static double rotateTolerance = Math.PI/36;

    //converts the joystick angle (global) to the angle needed to move the robot (local) in that direction
    public static double globalToRobot(double joystickAngle, double robotAngle){
        double localAngle = joystickAngle - robotAngle;
        return scaleAngle(localAngle);
    }


    //returns motor powers needed to go in a specific angle
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
                (motorPowerBlue + rotate) / factor, //backleft
                (-motorPowerRed + rotate) / factor, //backright
                (motorPowerRed + rotate) / factor, //frontleft
                (-motorPowerBlue + rotate) / factor //frontright
        };

        return motorPowers;
    }

    //Get motor powers to drive to a specific point
    public static double[] getMotorPowersToPoint(double currentX, double currentY, double targetX, double targetY, double currentRot, double targetRot, double speed){
        double angleToTarget = globalToRobot(Math.atan2(targetX - currentX, targetY - currentY), currentRot);

        double rotate = 0;
        double rotateFactor = 1;
        //Rotate either - or + based on difference in angles
        if(Math.abs(currentRot - targetRot) >= rotateTolerance){
            double rotDiff = scaleAngle(targetRot - currentRot);
            if(rotDiff > Math.PI) {
                rotDiff -= 2*Math.PI;
            }
            //Rotate less if closer to target
            rotate = rotDiff/Math.PI * rotateFactor;
            //Clip from 0.3 to 1 or from -1 to -0.3 respectively (less than 0.3 does not move)
            rotate = Range.clip(Math.abs(rotate),0.3,1) * rotate / Math.abs(rotate);
        }

        //Slow down if closer to target
        double modifiedSpeed = speed;
        if(pythagoras(currentX - targetX, currentY - targetY) <= movementDecelerationDistance){
            modifiedSpeed = Range.clip(pythagoras(currentX - targetX, currentY - targetY) / movementDecelerationDistance,0.3,1);
        }

        return getMotorPowersByDirection(angleToTarget, modifiedSpeed, rotate);
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
