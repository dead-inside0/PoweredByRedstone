package org.firstinspires.ftc.teamcode;

public class ToolBox {
    //converts the joystick angle (global) to the angle needed to move the robot (local) in that direction
    public static double joystickToRobot(double joystickAngle, double robotAngle){
        double localAngle = joystickAngle - robotAngle; //we MIGHT be fucked too
        if(localAngle > 2*Math.PI){
            localAngle -= 2*Math.PI;
        }
        if(localAngle < 0){
            localAngle += 2*Math.PI;
        }
        return localAngle;
    }


    //returns motor powers needed to go in a specific angle
    public static double[] getMotorPowers(double targetDirectionAngle){
        double[] motorPowers = {
                Math.sin(targetDirectionAngle-Math.PI/4), //backleft
                Math.sin(targetDirectionAngle+Math.PI/4), //backright
                Math.sin(targetDirectionAngle+Math.PI/4), //frontleft
                Math.sin(targetDirectionAngle-Math.PI/4)};//frontright
        return motorPowers;
    }
}
