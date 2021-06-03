package org.firstinspires.ftc.teamcode.DetectieFurataDePeNet;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedAutoHighGoal", group = "Autonomous")
public class UltimateGoalRedAutoHighGoal extends LinearOpMode {

    int stack = 2;
    OpenCvWebcam webCam;

    public void runOpMode(){
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webCam.openCameraDevice();

        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();

        webCam.setPipeline(pipeline);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webCam.resumeViewport();

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        waitForStart();

        while (opModeIsActive())
        {
            stack = pipeline.stack;
            telemetry.addData("stack", stack);
            telemetry.update();
        }
    }
}