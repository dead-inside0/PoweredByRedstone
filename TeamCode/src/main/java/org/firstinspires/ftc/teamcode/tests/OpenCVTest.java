package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

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
        //BGR COLOR FORMAT!!!
        Scalar highColorBoundary = new Scalar(255, 255, 255, 255);
        Scalar lowColorBoundary = new Scalar(120, 120, 120, 255);

        @Override
        public Mat processFrame(Mat input) {
            Mat rangeMat = new Mat();
            Core.inRange(input, lowColorBoundary, highColorBoundary, rangeMat);

            String text = String.format("# nonzero: %d, nonzero percentage %f", Core.countNonZero(rangeMat), 100.0 * Core.countNonZero(rangeMat) / (640*480));
            Point position = new Point(10, 440);
            Scalar textColor = new Scalar(0, 0, 255);
            int font = Imgproc.FONT_HERSHEY_SIMPLEX;
            double scale = 0.5;
            int thickness = 1;
            Imgproc.putText(input, text, position, font, scale, textColor, thickness);

            return rangeMat;
        }
    }
}
