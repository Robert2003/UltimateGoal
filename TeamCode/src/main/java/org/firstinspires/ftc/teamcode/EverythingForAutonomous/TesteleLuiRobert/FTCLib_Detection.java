package org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Scalar;

@Config
@Autonomous(name = "FTCLib_Detection", group = "Autonomous")
@Disabled
public class FTCLib_Detection extends LinearOpMode {
    public static double lowerThresholdCr = 120;
    public static double UpperThresholdCr = 230;
    public static double UpperThresholdCb = 108;

    @Override
    public void runOpMode() {
        //camera control
        UGContourRingDetector detector;
        UGContourRingPipeline.Height height;
        detector = new UGContourRingDetector(hardwareMap, "Webcam", telemetry, true);
        detector.init();
        UGContourRingPipeline.Config.setLowerOrange(new Scalar(0.0, lowerThresholdCr, 0.0));
        UGContourRingPipeline.Config.setUpperOrange(new Scalar(255.0, UpperThresholdCr, UpperThresholdCb));

        height = detector.getHeight();
        while(!opModeIsActive()){
            height = detector.getHeight();
        }

        FtcDashboard.getInstance().startCameraStream(detector.camera, 0);

        waitForStart();

        //chooses path based on vision data
        switch (height) {
            case ZERO:
                break;
            case ONE:
                break;
            case FOUR:
                break;
        }

        while(!isStopRequested()){

            telemetry.update();
        }
    }
}
