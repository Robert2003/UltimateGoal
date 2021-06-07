package org.firstinspires.ftc.teamcode.EverythingForAutonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RedAutoHighGoal", group = "Autonomous")
@Disabled
public class UltimateGoalRedAutoHighGoal extends LinearOpMode {

    int stack = 2;
    OpenCvWebcam webCam;

    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        boolean finishedAuto = false;

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webCam.openCameraDevice();

        UltimateGoalReturnPositionPipeline pipeline = new UltimateGoalReturnPositionPipeline();

        webCam.setPipeline(pipeline);
        webCam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        webCam.resumeViewport();

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        /** CASE 0 **/
            Trajectory shootingPosition_0 = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(58, 3), 6.02)
                    .build();
            Trajectory firstWobble_0 = drive.trajectoryBuilder(shootingPosition_0.end())
                    .splineTo(new Vector2d(74, -20), 4.712)
                    .build();
            Trajectory back_0 = drive.trajectoryBuilder(firstWobble_0.end())
                    .back(10)
                    .build();
            Trajectory grabSecondWobble_0 = drive.trajectoryBuilder(back_0.end(), true)
                    .lineToLinearHeading(new Pose2d(35, -20, 3.14))
                    .build();
            Trajectory forward_0 = drive.trajectoryBuilder(grabSecondWobble_0.end(), true)
                    .forward(7)
                    .build();
            Trajectory dropSecondWobble_0 = drive.trajectoryBuilder(forward_0.end())
                    .lineToLinearHeading(new Pose2d(63, -20, 4.712))
                    .build();
            Trajectory park_0 = drive.trajectoryBuilder(dropSecondWobble_0.end())
                    .lineToLinearHeading(new Pose2d(72, -10, 4.712))
                    .build();
        /** CASE 0 **/

        /** CASE 1 **/
            Trajectory shootingPosition1_1 = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(58, 3), 6.02)
                    .build();
            Trajectory firstWobble_1 = drive.trajectoryBuilder(shootingPosition1_1.end())
                    .splineTo(new Vector2d(85.85, -2.508), 5.3733)
                    .build();
            Trajectory back_1 = drive.trajectoryBuilder(firstWobble_1.end())
                    .back(10)
                    .build();
            Trajectory takeOneDisk_1 = drive.trajectoryBuilder(back_1.end(), true)
                    .splineTo(new Vector2d(38.119, -14), 0.275+3.14)
                    .build();
            Trajectory shootingPosition2_1 = drive.trajectoryBuilder(takeOneDisk_1.end(), false)
                    .splineTo(new Vector2d(55, -18), 0.01)
                    .build();
            Trajectory grabSecondWobble_1 = drive.trajectoryBuilder(shootingPosition2_1.end(), true)
                    .lineToLinearHeading(new Pose2d(40, -21, 3.14))
                    .build();
            Trajectory forward_1 = drive.trajectoryBuilder(grabSecondWobble_1.end(), true)
                    .forward(12)
                    .build();
            Trajectory dropSecondWobble_1 = drive.trajectoryBuilder(forward_1.end())
                    .lineToLinearHeading(new Pose2d(74.23, -13.25, 5.8))
                    .build();
        /** CASE 1 **/

        /** CASE 4 **/
            Trajectory shootingPosition1_4 = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(58, 3), 6.02)
                    .build();
            Trajectory firstWobble_4 = drive.trajectoryBuilder(shootingPosition1_4.end())
                    .splineTo(new Vector2d(108.6, -28.7), 5.43)
                    .addTemporalMarker(0.1, () -> {
                        robot.dropArm(700);
                    })
                    .build();
            Trajectory back_4 = drive.trajectoryBuilder(firstWobble_4.end())
                    .back(10)
                    .build();
            Trajectory goNextToRings_4 = drive.trajectoryBuilder(back_4.end(), true)
                    .lineToLinearHeading(new Pose2d(56.6, -11, 0.02))
                    .addTemporalMarker(0.1, () -> {
                        robot.toggleIntakeServo(true);
                        robot.dropArm(300);
                        robot.toggleIntake();
                        robot.toggleFlyWheel(true, 2995);
                    })
                    .build();
            Trajectory firstRing_4 = drive.trajectoryBuilder(goNextToRings_4.end(), true)
                    .lineToLinearHeading(new Pose2d(47, -11, 6.2))
                    .build();
            Trajectory secondRing_4 = drive.trajectoryBuilder(firstRing_4.end(), true)
                    .lineToLinearHeading(new Pose2d(39, -11, 6.2))
                    .build();
            Trajectory thirdRing_4 = drive.trajectoryBuilder(secondRing_4.end(), true)
                    .lineToLinearHeading(new Pose2d(34.5, -11, 6.25))
                    .addTemporalMarker(0.1, () -> {
                        robot.toggleFlyWheel(true, 3080);
                    })
                    .build();
            Trajectory fourthRing_4 = drive.trajectoryBuilder(thirdRing_4.end(), true)
                    .lineToLinearHeading(new Pose2d(20, -11, 6.25))
                    .build();
            Trajectory secondWobbleGoal_4 = drive.trajectoryBuilder(fourthRing_4.end(), true)
                    .lineToLinearHeading(new Pose2d(15.73, 1.56, 4.43))
                    .addTemporalMarker(0.1, () -> {
                        robot.toggleFlyWheel(false);
                        robot.toggleIntake();
                        robot.dropArm(800);
                    })
                    .build();
            Trajectory forward_4 = drive.trajectoryBuilder(secondWobbleGoal_4.end(), true)
                    .lineToLinearHeading(new Pose2d(12.68, -11, 4.43))
                    .build();
            Trajectory dropSecondWobble_4 = drive.trajectoryBuilder(forward_4.end())
                    .lineToLinearHeading(new Pose2d(103, -29, 5.496))
                    .addTemporalMarker(0.1, () -> {
                        robot.dropArm(650);
                    })
                    .build();
            Trajectory park_4 = drive.trajectoryBuilder(dropSecondWobble_4.end())
                    .lineTo(new Vector2d(80, -29))
                    .addTemporalMarker(0.1, () -> {
                        robot.wobbleServo.setPosition(0.45);
                        robot.dropArm(20);
                    })
                    .build();
        /** CASE 4 **/

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested() && !finishedAuto)
        {
            stack = pipeline.stack;
            telemetry.addData("stack", stack);
            telemetry.addData("treshold1", pipeline.getTreshold1());
            telemetry.addData("treshold2", pipeline.getTreshold2());
            telemetry.update();
            switch (stack) {
                case 0:
                    robot.toggleFlyWheel(true, 2970);
                    drive.followTrajectory(shootingPosition_0);
                    robot.shootrings(3);
                    robot.toggleFlyWheel(false);
                    drive.followTrajectory(firstWobble_0);
                    robot.dropArm(670);
                    sleep(1000);
                    robot.dropWobble();
                    drive.followTrajectory(back_0);
                    robot.dropArm(780);
                    drive.followTrajectory(grabSecondWobble_0);
                    drive.followTrajectory(forward_0);
                    sleep(800);
                    robot.grabWobble();
                    drive.followTrajectory(dropSecondWobble_0);
                    robot.dropArm(670);
                    robot.dropWobble();
                    drive.followTrajectory(park_0);
                    robot.wobbleServo.setPosition(0.45);
                    sleep(400);
                    robot.dropArm(20);
                    finishedAuto = true;
                    break;
                case 1:
                    robot.toggleFlyWheel(true, 2970);
                    drive.followTrajectory(shootingPosition1_1);
                    robot.shootrings(3);
                    robot.toggleFlyWheel(false);
                    drive.followTrajectory(firstWobble_1);
                    robot.dropArm(650);
                    robot.dropWobble();
                    drive.followTrajectory(back_1);
                    robot.dropArm(300);
                    robot.toggleFlyWheel(true, 3000);
                    robot.toggleIntake();
                    drive.followTrajectory(takeOneDisk_1);
                    drive.followTrajectory(shootingPosition2_1);
                    sleep(500);
                    robot.shootrings(1);
                    robot.toggleFlyWheel(false);
                    robot.dropArm(780);
                    robot.toggleIntake();
                    drive.followTrajectory(grabSecondWobble_1);
                    sleep(400);
                    drive.followTrajectory(forward_1);
                    sleep(800);
                    robot.grabWobble();
                    drive.followTrajectory(dropSecondWobble_1);
                    robot.dropArm(650);
                    robot.dropWobble();
                    robot.wobbleServo.setPosition(0.45);
                    sleep(400);
                    robot.dropArm(20);
                    finishedAuto = true;
                    break;
                case 4:
                    drive = new SampleMecanumDrive(hardwareMap, true);

                    robot.toggleFlyWheel(true, 2990);
                    drive.followTrajectory(shootingPosition1_4);
                    robot.shootrings(3);
                    robot.toggleFlyWheel(false);
                    drive.followTrajectory(firstWobble_4);
                    robot.dropWobble();
                    drive.followTrajectory(back_4);
                    drive.followTrajectory(goNextToRings_4);
                    drive.followTrajectory(firstRing_4);
                    drive.followTrajectory(secondRing_4);
                    sleep(500);
                    robot.shootrings(2);
                    drive.followTrajectory(thirdRing_4);
                    drive.followTrajectory(fourthRing_4);
                    sleep(500);
                    robot.shootrings(3);
                    drive.followTrajectory(secondWobbleGoal_4);
                    drive.followTrajectory(forward_4);
                    robot.grabWobble();
                    drive.followTrajectory(dropSecondWobble_4);
                    robot.dropWobble();
                    drive.followTrajectory(park_4);
                    finishedAuto = true;
                    break;
            }




        }
    }
}