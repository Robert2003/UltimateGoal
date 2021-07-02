/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.EverythingForAutonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
@Disabled
public class UltimateGoalDetection_2 extends LinearOpMode
{
    boolean finishedAuto = false;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.78);
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    ElapsedTime runtime = new ElapsedTime();

    public Trajectory spline, traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, pushDisksTraj1, pushDisksTraj2, pushDisksTraj3, pushDisksTraj4;

    OpenCvWebcam webCam;
    SkystoneDeterminationPipeline pipeline;
    String WEBCAM_NAME = "Webcam";

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx flyWheel = null;
        DcMotor wobbleArm = null;
        Servo wobbleServo, servo, intakeServo;
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        servo = hardwareMap.get(Servo.class, "servo");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);
        DcMotor intake1   = null, intake2 = null;
        intake1   = hardwareMap.get(DcMotor.class, "intake1");
        intake2   = hardwareMap.get(DcMotor.class, "intake2");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        intakeServo.setPosition(0.9);
        intake2.setDirection(DcMotor.Direction.REVERSE);
        wobbleServo.setPosition(1);
        servo.setPosition(0.55);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        MotorConfigurationType motorConfigurationType = flyWheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flyWheel.setMotorType(motorConfigurationType);

        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

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
                .lineToLinearHeading(new Pose2d(38, -21, 3.14))
                .build();
        Trajectory forward_0 = drive.trajectoryBuilder(grabSecondWobble_0.end(), true)
                .forward(10)
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
                    robot.dropArm(600);
                })
                .build();
        Trajectory back_4 = drive.trajectoryBuilder(firstWobble_4.end())
                .back(10)
                .build();
        Trajectory goNextToRings_4 = drive.trajectoryBuilder(firstWobble_4.end(), true)
                .lineToLinearHeading(new Pose2d(56.6, -11, 0.02))
                .addTemporalMarker(0.1, () -> {
                    robot.toggleIntakeServo(true);
                    robot.dropArm(300);
                    robot.toggleIntake();
                    robot.toggleFlyWheel(true, 2995);
                })
                .build();
        Trajectory firstRing_4 = drive.trajectoryBuilder(goNextToRings_4.end(), true)
                .lineToLinearHeading(new Pose2d(45, -11, 6.2))
                .build();
        Trajectory secondRing_4 = drive.trajectoryBuilder(firstRing_4.end(), true)
                .lineToLinearHeading(new Pose2d(37, -11, 6.2))
                .build();
        Trajectory thirdRing_4 = drive.trajectoryBuilder(secondRing_4.end(), true)
                .lineToLinearHeading(new Pose2d(30, -11, 6.25))
                .addTemporalMarker(0.1, () -> {
                    robot.toggleFlyWheel(true, 2970);
                    robot.toggleIntakeServo(false);
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
                .lineTo(new Vector2d(75, -29))
                .addTemporalMarker(0.1, () -> {
                    robot.wobbleServo.setPosition(0.45);
                    robot.dropArm(20);
                })
                .build();
        /** CASE 4 **/

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !finishedAuto)
        {
            while(runtime.milliseconds()<100);

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            SkystoneDeterminationPipeline.RingPosition lastPosition = pipeline.position;

            switch(lastPosition)
            {
                case NONE:
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
                case ONE:
                    robot.toggleFlyWheel(true, 2970);
                    drive.followTrajectory(shootingPosition1_1);
                    robot.shootrings(3);
                    robot.toggleFlyWheel(false);
                    drive.followTrajectory(firstWobble_1);
                    sleep(500);
                    robot.dropArm(570);
                    sleep(500);
                    robot.dropWobble();
                    drive.followTrajectory(back_1);
                    sleep(800);
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
                case FOUR:
                    drive = new SampleMecanumDrive(hardwareMap, true);

                    robot.toggleFlyWheel(true, 2990);
                    drive.followTrajectory(shootingPosition1_4);
                    robot.shootrings(3);
                    //robot.toggleFlyWheel(false);
                    drive.followTrajectory(firstWobble_4);
                    robot.dropWobble();
                    //drive.followTrajectory(back_4);
                    drive.followTrajectory(goNextToRings_4);
                    drive.followTrajectory(firstRing_4);
                    drive.followTrajectory(secondRing_4);
                    sleep(500);
                    robot.shootrings(2);
                    sleep(300);
                    drive.followTrajectory(thirdRing_4);
                    drive.followTrajectory(fourthRing_4);
                    sleep(400);
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
            // Don't burn CPU cycles busy-looping in this sample
            //sleep(50);


        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,15);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 149;
        final int ONE_RING_THRESHOLD = 136;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }
}