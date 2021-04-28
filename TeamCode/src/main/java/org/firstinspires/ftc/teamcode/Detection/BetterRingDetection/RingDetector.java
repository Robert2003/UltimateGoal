package org.firstinspires.ftc.teamcode.Detection.BetterRingDetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp (name = "RingDetector", group = "Detect")
public class RingDetector  extends LinearOpMode
{
    OpenCvCamera phoneCam;
    private String webcamName = "Webcam";
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;

    public RingPipeline visionPipeline = new RingPipeline();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Camera Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
            }
        });

        // Loading pipeline
        phoneCam.setPipeline(visionPipeline);

        // Start streaming the pipeline
        phoneCam.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {
            // Get data from the pipeline and output it to the telemetry. This are the variables you are going to work with.
            telemetry.addData("Ring 1:",visionPipeline.ring1); // Will return 0 if there is 1 ring, otherwise 1
            telemetry.addData("Ring 4:",visionPipeline.ring4); // Will return 0 if there is 4 rings, otherwise 1
            telemetry.update();
        }
    }
}
