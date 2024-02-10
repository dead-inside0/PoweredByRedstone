package org.firstinspires.ftc.teamcode.tests;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
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
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
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
    public void loop(){
        //Output some telemetry data
        telemetry.addData("FPS", phoneCam.getFps());
    }


    //Pipeline
    class TestPipeline extends OpenCvPipeline {
        //BGR COLOR FORMAT!!!
        Scalar highColorBoundary = new Scalar(0, 0, 255);
        Scalar lowColorBoundary = new Scalar(75, 75, 150);
        @Override
        public Mat processFrame(Mat input) {
            Mat rangeMat = input.clone(); //copy size might work
            inRange(input, lowColorBoundary, highColorBoundary, rangeMat);
            Mat out = rangeMat.clone(); //copy size might work
            Core.multiply(rangeMat, new Scalar(255, 255, 255), out);
            //return out;

            telemetry.addData("first pixel in input", input.at(input.getClass(), 0, 0));
            telemetry.addData("first pixel in range", rangeMat.at(rangeMat.getClass(), 0, 0));
            telemetry.addData("first pixel in output", out.at(out.getClass(), 0, 0));

            return input;
        }
    }
}
