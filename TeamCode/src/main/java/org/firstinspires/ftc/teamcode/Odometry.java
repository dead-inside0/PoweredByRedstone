package org.firstinspires.ftc.teamcode;

public class Odometry {
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        //Set known variables
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 2048;

        //Distance traveled by either odo wheel
        double rightArcLength = wheelCircumference * (deltaContactsRightOdo / sensorResolution);
        double leftArcLength = wheelCircumference * (deltaContactsLeftOdo / sensorResolution);

        //Length of the center arc - average between the two arcs
        double centerArcLength = (rightArcLength + leftArcLength) / 2;
        //Rotation change of the car
        double deltaRotation = (rightArcLength - leftArcLength) / sideOdosDistance;
        //Angle between center of car at start and end of frame
        double alpha = deltaRotation / 2;

        double shiftLength;
        double deltaX;
        double deltaY;

        //If didn't move
        if(centerArcLength == 0){
            shiftLength = 0;
        }
        else {
            //If drove straight
            if (deltaRotation == 0) {
                //Distance car center middle and after
                shiftLength = centerArcLength;
            }
            else {
                double centerArcRadius = deltaRotation / centerArcLength;
                shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(deltaRotation));
            }
            deltaX = shiftLength * Math.cos(prevAngle + alpha);
            deltaY = shiftLength * Math.sin(prevAngle + alpha);
        }
        deltaX = shiftLength * Math.cos(alpha);
        deltaY = shiftLength * Math.sin(alpha);

        double strafing = wheelCircumference * deltaContactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + alpha);
        double strafingDeltaY = strafing * Math.cos(prevAngle + alpha);

        deltaX += strafingDeltaX;
        deltaY += strafingDeltaY;

        double[] positionChange = {deltaX, deltaY, deltaRotation}; //x, y, angle
        return positionChange;
    }
}
