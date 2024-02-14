package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo){
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        double rightArcLength = wheelCircumference * (deltaContactsRightOdo / sensorResolution);
        double leftArcLength = wheelCircumference * (deltaContactsLeftOdo / sensorResolution);

        double centerArcLength = (rightArcLength + leftArcLength) / 2;
        double centerArcAngle = (rightArcLength - leftArcLength) / sideOdosDistance;
        double alpha = centerArcAngle / 2;

        double shiftLength;
        double deltaX;
        double deltaY;
        //TODO: something wrong here idk what
        if(centerArcAngle == 0){
            shiftLength = centerArcLength;
        }
        else{
            double centerArcRadius = centerArcAngle / centerArcLength;
            shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(centerArcAngle));
        }
        deltaX = shiftLength * Math.cos(alpha);
        deltaY = shiftLength * Math.sin(alpha);

        double strafing = wheelCircumference * deltaContactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(alpha);
        double strafingDeltaY = strafing * Math.cos(alpha);

        deltaX += strafingDeltaX;
        deltaY += strafingDeltaY;

        double[] positionChange = {deltaX, deltaY, centerArcAngle}; //x, y, angle
        return positionChange;
    }
}
