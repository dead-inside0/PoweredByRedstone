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
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

public class AutonomousOpMode extends LinearOpMode{

    final private ElapsedTime runtime = new ElapsedTime();

    static double[][] path;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;
    OpenCvCamera phoneCam;


    public double[][] getPath() {
        return path;
    }

    public void runOpMode() {
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;

        Servo droneServo = hMap.droneServo;

        DcMotor linearMechanismMotor = hMap.linearMechanismMotor;

        DcMotor pickupMotor = hMap.pickUpMotor;

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
        ColorDetect pipeline = new ColorDetect(highColor, lowColor);
        phoneCam.setPipeline(pipeline);

        path = getPath();
        //Open camera
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //Start streaming
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("ERROR: Camera could not be accessed");
                telemetry.update();
            }
        });



        waitForStart();

        int elementLocation;
        double runningSum = 0;
        int framesProcessed = 0;
        runtime.reset();
        while(runtime.seconds() < 1){
            runningSum += pipeline.getLastResult();
            framesProcessed ++;
        }
        elementLocation = (int) Math.round(runningSum/framesProcessed);

        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();

        double[] elementPosition;

        switch (elementLocation){
            case 0:
                elementPosition = new double[]{-200,100,-Math.PI/2};
                break;
            case 1:
                elementPosition = new double[]{0,100,-Math.PI/2};
                break;
            case 2:
                elementPosition = new double[]{200,100,-Math.PI/2};
                break;
            default:
                elementPosition = new double[]{0,0,0};
                break;
        }

        while (!(ToolBox.pythagoras(elementPosition[0]-posX,elementPosition[1]-posY) < ToolBox.movementTolerance && Math.abs(elementPosition[2]-robotRotation) < ToolBox.rotateTolerance)) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, elementPosition[0], elementPosition[1], robotRotation, elementPosition[2], 0.5);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);

            pickupMotor.setPower(-1);

            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", elementPosition[0], elementPosition[1], elementPosition[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
        }




        for (double[] point : path) {
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
            while (!(Math.abs(point[0] - posX) < ToolBox.movementTolerance && Math.abs(point[1] - posY) < ToolBox.movementTolerance && Math.abs(point[2]-robotRotation) < ToolBox.rotateTolerance)) {
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

            Rect rect0 = new Rect(80,0, 240, 200);
            Rect rect1 = new Rect(80, 200,240, 240);
            Rect rect2 = new Rect(80, 440, 240, 200);

            Mat mask0 = new Mat();
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();

            Core.inRange(frame.submat(rect0), lowColor, highColor, mask0);
            Core.inRange(frame.submat(rect1), lowColor, highColor, mask1);
            Core.inRange(frame.submat(rect2), lowColor, highColor, mask2);

            double percentage1 = (float)Core.countNonZero(mask0) / (float)(rect0.height * rect0.width);
            double percentage2 = (float)Core.countNonZero(mask1) / (float)(rect1.height * rect1.width);
            double percentage3 = (float)Core.countNonZero(mask2) / (float)(rect2.height * rect2.width);

            if(percentage1 > percentage2 && percentage1 > percentage3){
                Imgproc.rectangle(frame, rect0, new Scalar(60, 255, 255), 5);
                lastResult = 0;
            }
            else if(percentage2 > percentage3){
                Imgproc.rectangle(frame, rect1, new Scalar(60, 255, 255), 5);
                lastResult = 1;
            }
            else{
                Imgproc.rectangle(frame, rect2, new Scalar(60, 255, 255), 5);
                lastResult = 2;
            }


            //Core.inRange(frame, lowColor, highColor, frame);
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);
            return frame;
        }

        public int getLastResult(){
            return lastResult;
        }
    }
}
