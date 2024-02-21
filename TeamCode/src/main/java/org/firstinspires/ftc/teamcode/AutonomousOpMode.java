package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MyHardwareMap;
import org.firstinspires.ftc.teamcode.Odometry;
import org.firstinspires.ftc.teamcode.ToolBox;
import org.firstinspires.ftc.teamcode.tests.OpenCVTest;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutonomousOpMode extends LinearOpMode{

    final private ElapsedTime runtime = new ElapsedTime();

    private double[][] path;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;
    OpenCvCamera phoneCam;



    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;

        Servo droneServo = hMap.droneServo;

        DcMotor linearMechanismMotor = hMap.linearMechanismMotor;

        DcMotor leftOdo = hMap.leftOdo;
        DcMotor middleOdo = hMap.middleOdo;
        DcMotor rightOdo = hMap.rightOdo;

        int passedContactsRightOdo = rightOdo.getCurrentPosition();
        int passedContactsLeftOdo = leftOdo.getCurrentPosition();
        int passedContactsMiddleOdo = middleOdo.getCurrentPosition();


        //OPENCV STUFFS
        //Get camera with live preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        Scalar highColor = new Scalar(10, 255, 255);
        Scalar lowColor = new Scalar(0, 150, 30);

        //Set pipeline for frame processing
        phoneCam.setPipeline(new ColorDetect(highColor, lowColor));




        waitForStart();

        int elementLocation = getElementLocation();

        switch (elementLocation) {
            case 0:
                break;
            case 1:
                break;
            default:
                break;
        }


        for (double[] point : path) {
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
            while (!(Math.abs(posX - point[0]) < ToolBox.movementTolerance && Math.abs(posY - point[1]) < ToolBox.movementTolerance && Math.abs(robotRotation - point[2]) < ToolBox.rotateTolerance)) {
                int deltaContactsLeftOdo = leftOdo.getCurrentPosition() - passedContactsLeftOdo;
                int deltaContactsRightOdo = rightOdo.getCurrentPosition() - passedContactsRightOdo;
                int deltaContactsMiddleOdo = middleOdo.getCurrentPosition() - passedContactsMiddleOdo;

                //Update passed odo contacts
                passedContactsRightOdo += deltaContactsRightOdo;
                passedContactsLeftOdo += deltaContactsLeftOdo;
                passedContactsMiddleOdo += deltaContactsMiddleOdo;
                //Get position change
                double[] positionChange = Odometry.getPositionChange(-deltaContactsRightOdo, deltaContactsLeftOdo, -deltaContactsMiddleOdo, robotRotation);
                double deltaX = positionChange[0];
                double deltaY = positionChange[1];
                double deltaRotation = positionChange[2];

                //Update position
                posX += deltaX;
                posY += deltaY;
                robotRotation += deltaRotation;
                robotRotation = ToolBox.scaleAngle(robotRotation);

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, point[0], point[1], robotRotation, point[2], 0.5);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);

                telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
                telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
                telemetry.update();
            }
        }
    }

    int getElementLocation() {
        return 0;
    }

    class ColorDetect extends OpenCvPipeline {
        Scalar highColor;
        Scalar lowColor;

        public ColorDetect(Scalar highColor, Scalar lowColor){
            this.highColor = highColor;
            this.lowColor = lowColor;
        }

        int lastResult;
        @Override
        public Mat processFrame(Mat frame) {

            Imgproc.cvtColor(frame,frame,Imgproc.COLOR_RGB2HSV);

            Rect rect1 = new Rect(80,0, 240, 200);
            Rect rect2 = new Rect(80, 200,240, 240);
            Rect rect3 = new Rect(80, 440, 240, 200);

            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Mat mask3 = new Mat();

            Core.inRange(frame.submat(rect1), lowColor, highColor, mask1);
            Core.inRange(frame.submat(rect2), lowColor, highColor, mask2);
            Core.inRange(frame.submat(rect3), lowColor, highColor, mask3);

            double percentage1 = (float)Core.countNonZero(mask1) / (float)(rect1.height * rect1.width);
            double percentage2 = (float)Core.countNonZero(mask2) / (float)(rect2.height * rect2.width);
            double percentage3 = (float)Core.countNonZero(mask3) / (float)(rect3.height * rect3.width);

            if(percentage1 > percentage2 && percentage1 > percentage3){
                Imgproc.rectangle(frame, rect1, new Scalar(60, 255, 255), 5);
                lastResult = 1;
            }
            else if(percentage2 > percentage3){
                Imgproc.rectangle(frame, rect2, new Scalar(60, 255, 255), 5);
                lastResult = 2;
            }
            else{
                Imgproc.rectangle(frame, rect3, new Scalar(60, 255, 255), 5);
                lastResult = 3;
            }


            //Core.inRange(frame, lowColor, highColor, frame);
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);
            return frame;
        }

        int getLastResult(){
            return lastResult;
        }
    }
}
