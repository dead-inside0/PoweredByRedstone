package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 2048;

        double rightArcLength = wheelCircumference * (deltaContactsRightOdo / sensorResolution);
        double leftArcLength = wheelCircumference * (deltaContactsLeftOdo / sensorResolution);

        double centerArcLength = (leftArcLength + rightArcLength) / 2;
        double centerArcAngle = (leftArcLength - rightArcLength) / sideOdosDistance;
        double alpha = centerArcAngle / 2;

        double shiftLength;
        double deltaX;
        double deltaY;
        if(centerArcLength == 0){
            deltaX = 0;
            deltaY = 0;
        }
        else{
            if(centerArcAngle == 0){
                shiftLength = centerArcLength;
            }
            else{
                double centerArcRadius = centerArcAngle / centerArcLength;
                shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(centerArcAngle));
            }
            deltaX = shiftLength * Math.cos(prevAngle + alpha);
            deltaY = shiftLength * Math.sin(prevAngle + alpha);
        }

        double strafing = wheelCircumference * deltaContactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + alpha);
        double strafingDeltaY = strafing * Math.cos(prevAngle + alpha);

        deltaX += strafingDeltaX;
        deltaY += strafingDeltaY;

        double[] positionChange = {deltaX, deltaY, centerArcAngle}; //x, y, angle
        return positionChange;
    }
}
