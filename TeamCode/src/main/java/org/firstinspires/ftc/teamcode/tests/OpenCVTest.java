package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="OpenCV Test")
public class OpenCVTest extends OpMode {
    OpenCvCamera phoneCam;
    @Override
    public void init(){
        //Get camera with live preview
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        //Set pipeline for frame processing
        phoneCam.setPipeline(new TestPipeline());

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

        //Output ready
        telemetry.addLine("Initialized, ready to start");
        telemetry.update();
    }

    @Override
    public void loop(){}

    //Pipeline
    class TestPipeline extends OpenCvPipeline {
        Scalar highColor = new Scalar(120, 255, 255);
        Scalar lowColor = new Scalar(110, 150, 20);
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


            Core.inRange(frame, lowColor, highColor, frame);
            //Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);
            return frame;
        }

        int getLastResult(){
            return lastResult;
        }
    }
}
