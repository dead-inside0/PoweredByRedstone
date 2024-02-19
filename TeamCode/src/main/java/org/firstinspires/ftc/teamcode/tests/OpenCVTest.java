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
        int countInRange(Mat frame, Scalar highColor, Scalar lowColor){
            int count = 0;
            double[] lowColorArr = lowColor.val;
            double[] highColorArr = highColor.val;

            for( int i = 0; i < frame.rows(); ++i) {
                for( int j = 0; j < frame.cols(); ++j) {
                    double[] pixel = frame.get(i, j);
                    if(pixel[0] <= highColorArr[0] && pixel[0] >= lowColorArr[0]
                    && pixel[1] <= highColorArr[1] && pixel[1] >= lowColorArr[1]
                    && pixel[2] <= highColorArr[2] && pixel[2] >= lowColorArr[2]){
                        count++;
                    }
                }
            }
            return count;
        }


        Scalar highColor = new Scalar(145, 255, 255);
        Scalar lowColor = new Scalar(95, 50, 25);
        @Override
        public Mat processFrame(Mat frame) {
            Imgproc.cvtColor(frame,frame,Imgproc.COLOR_RGB2HSV);

            Rect topRect = new Rect(0,0, 480, 320);
            Rect bottomRect = new Rect(0, 320,480, 320);

            Mat topMask = new Mat();
            Mat bottomMask = new Mat();

            Core.inRange(frame.submat(topRect), lowColor, highColor, topMask);
            Core.inRange(frame.submat(bottomRect), lowColor, highColor, bottomMask);

            double topPercentage = (float)Core.countNonZero(topMask) / (float)(topRect.height * topRect.width);
            double bottomPercentage = (float)Core.countNonZero(bottomMask) / (float)(bottomRect.height * bottomRect.width);

            if(topPercentage > bottomPercentage){
                Imgproc.rectangle(frame, topRect, new Scalar(120, 255, 255), 25);
            }
            else{
                Imgproc.rectangle(frame, bottomRect, new Scalar(120, 255, 255), 25);
            }


            //Core.inRange(frame, lowColor, highColor, frame);
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_HSV2RGB);
            return frame;
        }
    }
}
