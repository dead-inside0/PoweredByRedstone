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

    public final double tile = 600;

    static double[][] path;

    double posX = 0;
    double posY = 0;
    double robotRotation = 0;
    OpenCvCamera phoneCam;

    int linearExtensionIndex = 0;

    Scalar highColorRed = new Scalar(10, 255, 255);
    Scalar lowColorRed = new Scalar(0, 150, 20);

    Scalar highColorBlue = new Scalar(120, 255, 255);
    Scalar lowColorBlue = new Scalar(110, 150, 20);

    /*  PATH:
            0-xPos
            1-yPos
            2-rot
            3-place motor
            4-place servo
            5-linear mechanism
     */



    public double[][] getPath() {
        return path;
    }
    public Scalar[] colorByIndex(char color) {
        switch (color) {
            case 'r':
                return new Scalar[] {highColorRed,lowColorRed};
            case 'b':
                return new Scalar[] {highColorBlue,lowColorBlue};
        }
        return new Scalar[] {};
    }

    public Scalar[] getColorBounds() {return new Scalar[]{};}

    public int linearExtensionIndex() {return linearExtensionIndex;}

    public double[] getPlacementPosition(int elementLocation) {
        if(elementLocation == 0){
            return new double[]{0,tile*1.25,Math.PI,-1,0,0};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile*2,Math.PI * 0.5,-1,0,0};
        }
        else if(elementLocation == 2){
            return new double[]{0,tile*1.25,0,-1,0,0};
        }
        return new double[]{};
    }

    public void runOpMode() {
        runtime.reset();
        MyHardwareMap hMap = new MyHardwareMap(hardwareMap);

        DcMotor backLeftMotor = hMap.backLeftMotor;
        DcMotor backRightMotor = hMap.backRightMotor;
        DcMotor frontLeftMotor = hMap.frontLeftMotor;
        DcMotor frontRightMotor = hMap.frontRightMotor;

        Servo placeServo = hMap.placeServo;

        DcMotor linearMechanismMotor = hMap.linearMechanismMotor;
        linearMechanismMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearMechanismMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        Scalar[] colorBounds = getColorBounds();

        //Set pipeline for frame processing
        ColorDetect pipeline = new ColorDetect(colorBounds[0], colorBounds[1]);
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
        //TEST IF THE AVERAGING FUCKS IT UP
        //double runningSum = 0;
        //int framesProcessed = 0;
        //runtime.reset();
        //while(runtime.seconds() < 1 && opModeIsActive()){
        //    runningSum += pipeline.getLastResult();
        //    framesProcessed ++;
        //}
        //elementLocation = (int) Math.round(runningSum/framesProcessed);
        elementLocation = pipeline.getLastResult();

        telemetry.addData("Element location: ", elementLocation);

        phoneCam.stopStreaming();

        path[0] = getPlacementPosition(elementLocation);


        /*while (!(ToolBox.pythagoras(elementPosition[0]-posX,elementPosition[1]-posY) < ToolBox.movementTolerance && Math.abs(elementPosition[2]-robotRotation) < ToolBox.rotateTolerance) && opModeIsActive()) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, elementPosition[0], elementPosition[1], robotRotation, elementPosition[2], 0.25);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);



            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", elementPosition[0], elementPosition[1], elementPosition[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
        }

        double pauseStartTime = runtime.seconds();
        double pauseStartX = posX;
        double pauseStartY = posY;
        double pauseStartRot = robotRotation;
        pickupMotor.setPower(-1);
        while(runtime.seconds() < pauseStartTime + 2  && opModeIsActive()) {
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

            double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, elementLocation == 0 ? pauseStartX+50 : elementLocation == 2 ? pauseStartX-50 : pauseStartX, elementLocation==1? pauseStartY- 50: pauseStartY, robotRotation, pauseStartRot, 0.25);

            backLeftMotor.setPower(motorPowers[0]);
            backRightMotor.setPower(motorPowers[1]);
            frontLeftMotor.setPower(motorPowers[2]);
            frontRightMotor.setPower(motorPowers[3]);
        }
        pickupMotor.setPower(0);*/




        for (int i = 0; i < path.length; i++) {
            double[] point = path[i];
            telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
            telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
            telemetry.update();
            while (!(ToolBox.pythagoras(point[0]-posX,point[1]-posY) < ToolBox.movementTolerance && Math.abs(point[2]-robotRotation) < ToolBox.rotateTolerance) && opModeIsActive()) {
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

                double[] motorPowers = ToolBox.getMotorPowersToPoint(posX, posY, point[0], point[1], robotRotation, point[2], 0.25);

                backLeftMotor.setPower(motorPowers[0]);
                backRightMotor.setPower(motorPowers[1]);
                frontLeftMotor.setPower(motorPowers[2]);
                frontRightMotor.setPower(motorPowers[3]);

                pickupMotor.setPower(i > 0 ? path[i-1][3] : 0);


                telemetry.addData("Next point: ", "X: %f, Y: %f, R: %f", point[0], point[1], point[2]);
                telemetry.addData("Current position: ", "X: %f, Y: %f, R: %f", posX, posY, robotRotation);
                telemetry.update();
            }
            pickupMotor.setPower(point[3]);
            placeServo.setPosition(point[4]);
            if(point[5] != 0){
                while(linearMechanismMotor.getCurrentPosition() > -2000 && linearMechanismMotor.getCurrentPosition() < 0 && opModeIsActive()) {
                    linearMechanismMotor.setPower(point[5]);
                }
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
