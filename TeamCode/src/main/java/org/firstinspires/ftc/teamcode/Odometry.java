package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static double[] getPositionChange(double contactsRightOdo, double contactsLeftOdo, double contactsMiddleOdo, double prevAngle){
        final double sideOdosDistance = 305;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        double rightArcLength = wheelCircumference * contactsRightOdo / sensorResolution;
        double leftArcLength = wheelCircumference * contactsLeftOdo / sensorResolution;

        double centerArcLength = (rightArcLength + leftArcLength) / 2;
        double centerArcAngle = (rightArcLength - leftArcLength) / sideOdosDistance;
        double directionAngle  = centerArcAngle / 2;

        double centerArcRadius = (360 * centerArcLength) / (centerArcAngle * 2 * Math.PI);
        double shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(centerArcAngle));

        double deltaX = shiftLength * Math.cos(prevAngle + directionAngle);
        double deltaY = shiftLength * Math.sin(prevAngle + directionAngle);

        double strafing = wheelCircumference * contactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + directionAngle);
        double strafingDeltaY = strafing * Math.cos(prevAngle + directionAngle);

        double totalDeltaX = deltaX + strafingDeltaX;
        double totalDeltaY = deltaY + strafingDeltaY;

        double[] positionChange = {totalDeltaX, totalDeltaY, directionAngle}; //x, y, angle
        return positionChange;
    }
}