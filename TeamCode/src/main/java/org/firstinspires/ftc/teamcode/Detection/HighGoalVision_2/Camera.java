package org.firstinspires.ftc.teamcode.Detection.HighGoalVision_2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {


    //Subsystem Components
    public OpenCvCamera webcam;
    public OpenCvPipeline pipeline;
    public BlueGoalVisionPipeline goalPipeline;

    //Subsystem Component Names

    //Editable Constants
    public static double RING_STACK_POSITION = 0.09;
    public static double HIGH_GOAL_POSITION = 0.16;

    public Camera(HardwareMap hardwareMap, OpenCvPipeline pipeline)
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        this.pipeline = pipeline;
        webcam.setPipeline(pipeline);
        //opens connection to camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }
        });
    }

}