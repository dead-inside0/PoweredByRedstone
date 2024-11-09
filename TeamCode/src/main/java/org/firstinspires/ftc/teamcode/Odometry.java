package org.firstinspires.ftc.teamcode;

public class Odometry {
    //https://chsftcrobotics.weebly.com/uploads/1/2/3/6/123696510/odometry.pdf
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        //Set known variables
        final double sideOdosDistance = 330;
        final double middleOdoDistance = -120;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        //Distance traveled by either odo wheel
        double rightArcLength = wheelCircumference * (deltaContactsRightOdo / sensorResolution);
        double leftArcLength = wheelCircumference * (deltaContactsLeftOdo / sensorResolution);

        double strafeArcLength = wheelCircumference * (deltaContactsMiddleOdo / sensorResolution);

        //Rotation change of the car
        double deltaRotation = (rightArcLength - leftArcLength) / sideOdosDistance;

        double deltaX;
        double deltaY;
        if(deltaRotation == 0){
            deltaX = strafeArcLength;
            deltaY = (rightArcLength + leftArcLength)/2;
        }
        else{
            //Length of the center arc - average between the two arcs
            double turningRadius = (sideOdosDistance/2 * (rightArcLength + leftArcLength)) / (rightArcLength - leftArcLength);

            double strafeArcRadius = strafeArcLength/deltaRotation - middleOdoDistance;

            deltaX = turningRadius * (Math.cos(deltaRotation) - 1) + strafeArcRadius * Math.sin(deltaRotation);
            deltaY = turningRadius * Math.sin(deltaRotation) + strafeArcRadius * (1 - Math.cos(deltaRotation));
        }

        return new double[]{deltaX, deltaY, deltaRotation};
    }
}
