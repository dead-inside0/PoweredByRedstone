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

            for( int i = 0; i < frame.rows(); ++i) {
                for( int j = 0; j < frame.cols(); ++j) {
                    if(frame.get(i, j)[0] <= highColor.val[0] && frame.get(i, j)[0] >= lowColor.val[0]
                    && frame.get(i, j)[1] <= highColor.val[1] && frame.get(i, j)[1] >= lowColor.val[1]
                    && frame.get(i, j)[2] <= highColor.val[2] && frame.get(i, j)[2] >= lowColor.val[2]){
                        count++;
                    }
                }
            }
            return count;
        }


        Scalar highColor = new Scalar(140, 255, 255);
        Scalar lowColor = new Scalar(100, 100, 20);
        @Override
        public Mat processFrame(Mat frame) {
            Imgproc.cvtColor(frame,frame,Imgproc.COLOR_RGB2HSV);

            Rect topRect = new Rect(new Point(0,0), new Point(320, 480));
            Rect bottomRect = new Rect(new Point(320, 0), new Point(640, 480));

            //Mat topMask = new Mat();
            //Mat bottomMask = new Mat();

            //Core.inRange(frame.submat(topRect), lowColor, highColor, topMask);
            //Core.inRange(frame.submat(bottomRect), lowColor, highColor, bottomMask);

            String text;
            //if(Core.countNonZero(topMask) > Core.countNonZero(bottomMask)){
            //    text = "top";
            //}
            //else{
            //    text = "bottom";
            //}

            if(countInRange(frame.submat(topRect), lowColor, highColor) > countInRange(frame.submat(bottomRect), lowColor, highColor)){
                text = "top";
            }
            else{
                text = "bottom";
            }

            Imgproc.putText(frame, text, new Point(25, 200), Imgproc.FONT_HERSHEY_COMPLEX, 0.75, new Scalar(0, 0, 0));

            //Core.inRange(frame, lowColor, highColor, frame);
            return frame;
        }
    }
}
