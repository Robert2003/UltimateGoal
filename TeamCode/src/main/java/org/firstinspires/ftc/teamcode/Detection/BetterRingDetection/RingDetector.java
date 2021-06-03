package org.firstinspires.ftc.teamcode.Detection.BetterRingDetection;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp (name = "RingDetector", group = "Detect")
@Disabled
public class RingDetector  extends LinearOpMode
{
    OpenCvWebcam webCam;
    String WEBCAM_NAME = "Webcam";
    public RingPipeline visionPipeline = new RingPipeline();

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Camera Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        visionPipeline = new RingPipeline();
        webCam.setPipeline(visionPipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

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
