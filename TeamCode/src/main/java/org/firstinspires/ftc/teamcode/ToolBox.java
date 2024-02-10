package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

public class ToolBox {
    //converts the joystick angle (global) to the angle needed to move the robot (local) in that direction
    public static double joystickToRobot(double joystickAngle, double robotAngle){
        double localAngle = joystickAngle - robotAngle; //we MIGHT be fucked
        return clampAngle(localAngle);
    }


    //returns motor powers needed to go in a specific angle
    public static double[] getMotorPowersByDirection(double targetDirectionAngle, double magnitude, double rotate){
        targetDirectionAngle -= Math.PI/2;
        targetDirectionAngle = clampAngle(targetDirectionAngle);

        double motorPowerBlue = (Math.sin(targetDirectionAngle + Math.PI / 4) * magnitude + rotate);
        double motorPowerRed = (Math.sin(targetDirectionAngle - Math.PI / 4) * magnitude + rotate);

        double maxMotorPower = Math.max(Math.max(motorPowerBlue, -motorPowerBlue), Math.max(motorPowerRed, motorPowerRed));

        double[] motorPowers = { // motor powers are scaled so the max power = 1
                motorPowerRed / maxMotorPower, //backleft
                -motorPowerBlue / maxMotorPower, //backright
                motorPowerRed / maxMotorPower, //frontleft
                -motorPowerBlue / maxMotorPower//frontright
        };

        return motorPowers;
    }

    //pythagoras theorem
    public static double pythagoras(double x, double y){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    //Makes sure angle is between 0 and 2PI
    public static double clampAngle(double angle){
        if(angle > 2*Math.PI){
            return angle - 2* Math.PI;
        }
        else if(angle < 0){
            return angle + 2*Math.PI;
        }
        else{
            return angle;
        }
    }

    //returns the angle between two points
    public static double getAngleToPoint(double selfX, double selfY, double targetX, double targetY){
        return Math.atan2(selfX-targetX, selfY-targetY);
    }

    //returns motor powers to get to a certain point
    public static double[] getMotorPowersToPoint(double selfX, double selfY, double targetX, double targetY, double speed, double rotate){
        return getMotorPowersByDirection(getAngleToPoint(selfX, selfY, targetX, targetY), speed, rotate);
    }
}
