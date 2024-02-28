package org.firstinspires.ftc.teamcode;

public class Odometry {
    //test podle https://chsftcrobotics.weebly.com/uploads/1/2/3/6/123696510/odometry.pdf
    public static double[] getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        //Set known variables
        final double sideOdosDistance = 300;
        final double middleOdoDistance = 170;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

        //Distance traveled by either odo wheel
        double rightArcLength = wheelCircumference * (deltaContactsRightOdo / sensorResolution);
        double leftArcLength = wheelCircumference * (deltaContactsLeftOdo / sensorResolution);

        //Rotation change of the car
        double deltaRotation = (rightArcLength - leftArcLength) / sideOdosDistance;

        //Length of the center arc - average between the two arcs
        double turningRadius = (sideOdosDistance/2 * (rightArcLength + leftArcLength)) / (rightArcLength - leftArcLength);

        //Length of shift while strafing
        double strafeArcLength = wheelCircumference * (deltaContactsMiddleOdo / sensorResolution);
        double strafeArcRadius = strafeArcLength/deltaRotation - middleOdoDistance;

        double deltaX;
        double deltaY;
        if(deltaRotation == 0){
            deltaX = strafeArcLength;
            deltaY = (rightArcLength + leftArcLength)/2;
        }
        else{
            deltaX = turningRadius * (Math.cos(deltaRotation) - 1) + strafeArcRadius * Math.sin(deltaRotation);
            deltaY = turningRadius * Math.sin(deltaRotation) + strafeArcRadius * (1 - Math.cos(deltaRotation));
        }

        return new double[]{deltaX, deltaY, deltaRotation};
    }

    //nejaka ernestova vec nevim kde to vzal
    @Deprecated
    public static double[] _getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;
        final double middleOdoDistance = 170;

        double deltaLeftOdo = (deltaContactsLeftOdo * wheelCircumference) / sensorResolution;
        double deltaRightOdo = (deltaContactsRightOdo * wheelCircumference) / sensorResolution;
        double deltaMiddleOdo = (deltaContactsMiddleOdo * wheelCircumference) / sensorResolution;

        double deltaAngle = (deltaRightOdo - deltaLeftOdo) / sideOdosDistance;

        double centerDisplacement = (deltaRightOdo + deltaLeftOdo) / 2;
        double horizontalDisplacement = deltaMiddleOdo - (deltaAngle * middleOdoDistance);

        double deltaX = centerDisplacement * Math.cos(deltaAngle + prevAngle) - horizontalDisplacement * Math.sin(deltaAngle + prevAngle);
        double deltaY = centerDisplacement * Math.sin(deltaAngle + prevAngle) + horizontalDisplacement * Math.cos(deltaAngle + prevAngle);

        return new double[]{deltaX, deltaY, deltaAngle+prevAngle};

    }


    //podle eng notebooku 2021/22 - vubec nefunguje
    @Deprecated
    public static double[] __getPositionChange(int deltaContactsRightOdo, int deltaContactsLeftOdo, int deltaContactsMiddleOdo, double prevAngle){
        //Set known variables
        final double sideOdosDistance = 300;
        final double wheelCircumference = 60 * Math.PI;
        final double sensorResolution = 8192;

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
                //Distance between car center before and after
                shiftLength = centerArcLength;
            }
            else {
                double centerArcRadius = deltaRotation / centerArcLength;
                shiftLength = Math.sqrt(2 * Math.pow(centerArcRadius, 2) - 2 * Math.pow(centerArcRadius, 2) * Math.cos(deltaRotation));
            }

        }
        deltaX = shiftLength * Math.cos(prevAngle + alpha);
        deltaY = shiftLength * Math.sin(prevAngle + alpha);

        //Distance traveled by middle odo
        double strafing = wheelCircumference * deltaContactsMiddleOdo / sensorResolution;

        double strafingDeltaX = strafing * Math.sin(prevAngle + alpha);
        double strafingDeltaY = strafing * Math.cos(prevAngle + alpha);

        deltaX += strafingDeltaX;
        deltaY += strafingDeltaY;

        double[] positionChange = {deltaX, deltaY, deltaRotation}; //x, y, angle
        return positionChange;
    }
}
