package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        double rightArcLength = wheelCircumference * deltaContactsRightOdo / sensorResolution;
        double leftArcLength = wheelCircumference * deltaContactsLeftOdo / sensorResolution;

        double centerArcLength = (rightArcLength + leftArcLength) / 2;
        double centerArcAngle = (rightArcLength - leftArcLength) / sideOdosDistance;
        double directionAngle  = centerArcAngle / 2;

        double shiftLength;
        double deltaX;
        double deltaY;
        if(centerArcLength == 0){
            deltaX = 0;
            deltaY = 0;
        }
        else{
            double centerArcRadius = centerArcAngle / centerArcLength;
            shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(centerArcAngle));
            deltaX = shiftLength * Math.cos(prevAngle + directionAngle);
            deltaY = shiftLength * Math.sin(prevAngle + directionAngle);
        }

        double strafing = wheelCircumference * deltaContactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + directionAngle);
        double strafingDeltaY = strafing * Math.cos(prevAngle + directionAngle);

        deltaX += strafingDeltaX;
        deltaY += strafingDeltaY;

        double[] positionChange = {deltaX, deltaY, directionAngle}; //x, y, angle
        return positionChange;
    }
}
