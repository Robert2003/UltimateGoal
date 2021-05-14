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

package org.firstinspires.ftc.teamcode.Detection;

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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOP.RobotDefinition;
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
public class UltimateGoalDetection extends LinearOpMode
{
    public RobotDefinition robot = new RobotDefinition();
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
    public void runOpMode()
    {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        
        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot.wobbleServo.setPosition(1);

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

        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !finishedAuto)
        {
            while(runtime.milliseconds()<100);
           // sleep(600);

            telemetry.update();

            SkystoneDeterminationPipeline.RingPosition lastPosition = pipeline.position;

            lastPosition = SkystoneDeterminationPipeline.RingPosition.NONE;

            switch(lastPosition)
            {
                case NONE:
                    robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
                    robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.wobbleArm.setPower(1);

                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3600));

                    traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                            .lineTo(new Vector2d(60, -21))
                            .build(); //Shooting point
                    traj2 = drive.trajectoryBuilder(traj1.end(), true)
                            .lineToLinearHeading(new Pose2d(65, -6, Math.toRadians(-60.16973581)))
                            .build(); //Leave first wobble
                    traj3 = drive.trajectoryBuilder(traj2.end(), true)
                            .lineToLinearHeading(new Pose2d(37, -6, Math.toRadians(-180)))
                            .build(); //Approach second wobble
                    traj4 = drive.trajectoryBuilder(traj3.end(), true)
                            .back(10)
                            .build(); //Take second wobble
                    traj5 = drive.trajectoryBuilder(traj4.end(), true)
                            .lineToLinearHeading(new Pose2d(-63, 17, Math.toRadians(-90)))
                            .build(); //Leave second wobble
                    traj6 = drive.trajectoryBuilder(traj5.end(), true)
                            .strafeRight(7)
                            .build(); //Parking

                    drive.followTrajectory(traj1);
                    for(int i = 1; i <= 3; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }

                    drive.followTrajectory(traj2);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleServo.setPosition(0);
                    sleep(800);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj3);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition()-robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj4);
                    robot.wobbleServo.setPosition(1);
                    sleep(800);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj5);
                    robot.wobbleServo.setPosition(0);
                    sleep(800);
                    robot.wobbleArm.setTargetPosition(-50);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    sleep(1000);

                    drive.followTrajectory(traj6);
                    finishedAuto = true;
                    break;
                case ONE:
                    robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
                    robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.wobbleArm.setPower(1);

                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3600));


                    traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                            .lineTo(new Vector2d(-62, 0))
                            .build(); //Left from shooting point
                    traj2 = drive.trajectoryBuilder(traj1.end(), true)
                            .lineTo(new Vector2d(-62, 16.5))
                            .build(); //Shooting point
                    traj3 = drive.trajectoryBuilder(traj2.end(), true)
                            .lineTo(new Vector2d(-82, 15))
                            .build(); //Leave first wobble
                    traj4 = drive.trajectoryBuilder(traj3.end(), true)
                            .lineTo(new Vector2d(-37,16))
                            .build(); //Take the one disk
                    traj5 = drive.trajectoryBuilder(traj4.end(), true)
                            .lineTo(new Vector2d(-63, 16.5))
                            .build(); //Shooting point
                    traj6 = drive.trajectoryBuilder(traj5.end(), true)
                            .lineToLinearHeading(new Pose2d(-37, 21.5, Math.toRadians(-179.8)))
                            .build(); //Approach second wobble
                    traj7 = drive.trajectoryBuilder(traj6.end(), true)
                            .back(10)
                            .build(); //Take second wobble
                    traj8 = drive.trajectoryBuilder(traj7.end(), true)
                            .lineToLinearHeading(new Pose2d(-84, 19, Math.toRadians(0)))
                            .build(); //Leave second wobble
                    traj9 = drive.trajectoryBuilder(traj8.end(), true)
                            .forward(8)
                            .build(); //Parking

                    drive.followTrajectory(traj1);
                    drive.followTrajectory(traj2);
                    for(int i = 1; i <= 3; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }
                    robot.flyWheel.setPower(0);

                    drive.followTrajectory(traj3);
                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleServo.setPosition(0);
                    sleep(700);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleArm.setTargetPosition(-50);
                    robot.wobbleArm.setPower(0.25);
                    robot.intake1.setPower(1);
                    robot.intake2.setPower(1);

                    drive.followTrajectory(traj4);
                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3600));

                    drive.followTrajectory(traj5);
                    sleep(2000);
                    //shoot the one disk
                    for(int i = 1; i <= 2; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }
                    robot.intake1.setPower(0);
                    robot.intake2.setPower(0);
                    robot.flyWheel.setPower(0);

                    drive.followTrajectory(traj6);
                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj7);
                    robot.wobbleServo.setPosition(1);
                    sleep(700);
                    robot.wobbleArm.setTargetPosition(-300);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj8);
                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleServo.setPosition(0);
                    sleep(800);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleArm.setTargetPosition(-50);
                    robot.wobbleArm.setPower(0.25);

                    drive.followTrajectory(traj9);
                    finishedAuto = true;
                    break;
                case FOUR:
                    robot.wobbleArm.setTargetPosition(robot.wobbleArm.getCurrentPosition());
                    robot.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.wobbleArm.setPower(1);

                    spline = drive.trajectoryBuilder(new Pose2d(), true)
                            .splineToConstantHeading(new Vector2d(-50, 0), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(-63, 15.5), Math.toRadians(0))
                            .build();

                    traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                            .lineTo(new Vector2d(-62, 0))
                            .build(); //Left from shooting point
                    traj2 = drive.trajectoryBuilder(traj1.end(), true)
                            .lineTo(new Vector2d(-62, 16.5))
                            .build(); //Shooting point
                    pushDisksTraj1 = drive.trajectoryBuilder(spline.end(), true)
                            .lineTo(new Vector2d(-48, 15.5))
                            .build();
                    pushDisksTraj2 = drive.trajectoryBuilder(pushDisksTraj1.end(), true)
                            .forward(7)
                            .build();
                    pushDisksTraj3 = drive.trajectoryBuilder(pushDisksTraj2.end(), true)
                            .forward(7)
                            .build();
                    pushDisksTraj4 = drive.trajectoryBuilder(pushDisksTraj3.end(), true)
                            .forward(7)
                            .build();
                    traj3 = drive.trajectoryBuilder(pushDisksTraj2.end(), true)
                            .lineToLinearHeading(new Pose2d(-105, 27.67, Math.toRadians(-30)))
                            .build(); //Leave first wobble
                    traj4 = drive.trajectoryBuilder(traj3.end(), true)
                            .lineToLinearHeading(new Pose2d(-37, 20, Math.toRadians(-180)))
                            .build(); //Approach second wobble
                    traj5 = drive.trajectoryBuilder(traj4.end(), true)
                            .back(10)
                            .build(); //Take second wobble
                    traj6 = drive.trajectoryBuilder(traj5.end(), true)
                            .lineToLinearHeading(new Pose2d(-100, 30, Math.toRadians(-30)))
                            .build(); //Leave second wobble
                    traj7 = drive.trajectoryBuilder(traj6.end(), true)
                            .forward(35)
                            .build(); //Parking


                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3600));

                    //drive.followTrajectory(traj1);
                    //drive.followTrajectory(traj2);
                    drive.followTrajectory(spline);

                    for(int i = 1; i <= 3; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }


                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3200));

                    robot.intake1.setPower(1);
                    robot.intake2.setPower(1);
                    robot.intakeServo.setPosition(0);
                    sleep(500);

                    drive.followTrajectory(pushDisksTraj1);
                    sleep(200);
                    drive.followTrajectory(pushDisksTraj2);
                    sleep(200);

                    for(int i = 1; i <= 2; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }

                    robot.flyWheel.setVelocity(rpmToTicksPerSecond(3260));

                    drive.followTrajectory(pushDisksTraj3);
                    sleep(200);
                    drive.followTrajectory(pushDisksTraj4);
                    sleep(200);

                    for(int i = 1; i <= 3; i++) {
                        robot.servo.setPosition(0);
                        sleep(400);
                        robot.servo.setPosition(0.55);
                        sleep(400);
                    }
                    robot.flyWheel.setVelocity(0);
                    robot.intake1.setPower(0);
                    robot.intake2.setPower(0);

                    drive.followTrajectory(traj3);

                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleServo.setPosition(0);
                    sleep(250);
                    robot.wobbleArm.setTargetPosition(-500);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);


                    drive.followTrajectory(traj4);

                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj5);

                    robot.wobbleServo.setPosition(1);
                    sleep(500);
                    robot.wobbleArm.setTargetPosition(-300);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);

                    drive.followTrajectory(traj6);

                    robot.wobbleArm.setTargetPosition(-600);
                    robot.wobbleArm.setPower(0.25);
                    while(Math.abs(robot.wobbleArm.getTargetPosition() - robot.wobbleArm.getCurrentPosition()) > 20);
                    robot.wobbleServo.setPosition(0);
                    sleep(300);

                    robot.wobbleArm.setTargetPosition(-50);
                    robot.wobbleArm.setPower(0.25);

                    drive.followTrajectory(traj7);

                    finishedAuto = true;
                    break;
            }
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

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

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