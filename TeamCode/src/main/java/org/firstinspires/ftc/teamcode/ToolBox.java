package org.firstinspires.ftc.teamcode;

public class ToolBox {
    public static double joystickToRobot(double joystickAngle, double robotAngle){
        double localAngle = joystickAngle - robotAngle; //we MIGHT be fucked too
        if(localAngle > 360){
            localAngle -= 360;
        }
        if(localAngle < 0){
            localAngle += 360;
        }
        return localAngle;
    }
}
