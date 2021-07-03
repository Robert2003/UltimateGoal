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

package org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases.ConditionalCase0;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases.ConditionalCase1;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases.ConditionalCase4;
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
public class UltimateGoalDetectionConditional extends LinearOpMode {
    boolean finishedAuto = false;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.7);
    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    ElapsedTime runtime = new ElapsedTime();

    public Trajectory spline, traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8, traj9, pushDisksTraj1, pushDisksTraj2, pushDisksTraj3, pushDisksTraj4;
    boolean isRed, isFirst, collectStack, shouldPark, waitingAnswer;
    int selectedCase;
    //delays
    int collectStackDelay, parkDelay, shootDelay;
    int selectedAnswer = 1;

    OpenCvWebcam webCam;
    SkystoneDeterminationPipeline pipeline;
    String WEBCAM_NAME = "Webcam";

    SampleMecanumDrive drive;

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
        drive = new SampleMecanumDrive(hardwareMap, false);
        DcMotor intake1 = null, intake2 = null;
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
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

        webCam.openCameraDeviceAsync(() -> webCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT));

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

        askQuestions();
        showcaseAnswers();
        confirmAnswers();

        ConditionalCase0 conditionalCase0 = new ConditionalCase0(this);
        ConditionalCase1 conditionalCase1 = new ConditionalCase1(this);
        ConditionalCase4 conditionalCase4 = new ConditionalCase4(this);
        waitForStart();
        runtime.reset();

        while (opModeIsActive() && !finishedAuto) {
            while (runtime.milliseconds() < 100) ;

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            SkystoneDeterminationPipeline.RingPosition lastPosition = pipeline.position;
            if (selectedCase == 0) lastPosition = SkystoneDeterminationPipeline.RingPosition.NONE;
            else if (selectedCase == 1)
                lastPosition = SkystoneDeterminationPipeline.RingPosition.ONE;
            else if (selectedCase == 4)
                lastPosition = SkystoneDeterminationPipeline.RingPosition.FOUR;

            switch (lastPosition) {
                case NONE:
                    conditionalCase0.runCase();
                    finishedAuto = true;
                    break;
                case ONE:
                    conditionalCase1.runCase();
                    finishedAuto = true;
                    break;
                case FOUR:
                    conditionalCase4.runCase();
                    finishedAuto = true;
                    break;
            }
            // Don't burn CPU cycles busy-looping in this sample
            //sleep(50);


        }
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition {
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
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 15);

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
        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
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

        public int getAnalysis() {
            return avg1;
        }
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public SampleMecanumDrive getDrive() {
        return drive;
    }

    public boolean getIsRed() {
        return isRed;
    }

    public boolean getPark() {
        return shouldPark;
    }

    public boolean getIsFirst() {
        return isFirst;
    }

    public boolean getCollectStack() {
        return collectStack;
    }

    private void askQuestions() {
        telemetry.addData("Q1", "Is the robot on the red side? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                waitingAnswer = false;
                isRed = true;
            } else if (gamepad1.x) {
                isRed = false;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();

        while (!gamepad1.y && !isStopRequested()) ;

        telemetry.addData("Q2", "Is the robot on the first line? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                waitingAnswer = false;
                isFirst = true;
            } else if (gamepad1.x) {
                isFirst = false;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();

        while (!gamepad1.y && !isStopRequested()) ;

        telemetry.addData("Q3", "Should the robot park? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                waitingAnswer = false;
                shouldPark = true;
            } else if (gamepad1.x) {
                shouldPark = false;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();

        while (!gamepad1.y && !isStopRequested()) ;

        telemetry.addData("Q4", "Should the robot collect the stack? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                waitingAnswer = false;
                collectStack = true;
            } else if (gamepad1.x) {
                collectStack = false;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();

        while (!gamepad1.y && !isStopRequested()) ;

        telemetry.addData("Q5", "What case should the robot run? \n" +
                "DPAD UP - Case 0\n" +
                "DPAD RIGHT - Case 1\n" +
                "DPAD DOWN - Case 4\n" +
                "DPAD LEFT - Detection");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.dpad_up) {
                waitingAnswer = false;
                selectedCase = 0;
            } else if (gamepad1.dpad_right) {
                selectedCase = 1;
                waitingAnswer = false;
            } else if (gamepad1.dpad_down) {
                selectedCase = 4;
                waitingAnswer = false;
            } else if (gamepad1.dpad_left) {
                selectedCase = -1;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();

        while (!gamepad1.y && !isStopRequested()) ;
    }

    private void showcaseAnswers() {
        telemetry.addData("A1", "Side: " + (isRed ? "RED" : "BLUE") + ((selectedAnswer == 1) ? " [X]" : ""));
        telemetry.addData("A2", "Line: " + (isFirst ? "FIRST" : "SECOND") + ((selectedAnswer == 2) ? " [X]" : ""));
        telemetry.addData("A3", "Park: " + (shouldPark ? "YES" : "NO") + ((selectedAnswer == 3) ? " [X]" : ""));
        telemetry.addData("A4", "Collect stack: " + (collectStack ? "YES" : "NO") + ((selectedAnswer == 4) ? " [X]" : ""));
        if (selectedAnswer == -1) telemetry.addData("A5", "Detection");
        else if (selectedAnswer == 0) telemetry.addData("A5", "Case 0");
        else if (selectedAnswer == 1) telemetry.addData("A5", "Case 1");
        else if (selectedAnswer == 4) telemetry.addData("A5", "Case 4");
        telemetry.addData("F", "Press B to modify your answers.\nPress Y to submit your answers.");
        telemetry.update();
    }

    private void confirmAnswers() {
        sleep(500);
        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.b) {
                waitingAnswer = false;
                askQuestions();
            } else if (gamepad1.y) {
                waitingAnswer = false;
                telemetry.addData("R", "Ready for start.");
                telemetry.update();
            } else if (gamepad1.dpad_down && selectedAnswer < 4) {
                selectedAnswer++;
                showcaseAnswers();
            } else if (gamepad1.dpad_up && selectedAnswer > 1) {
                selectedAnswer--;
                showcaseAnswers();
            } else if(gamepad1.dpad_right || gamepad1.dpad_left){
                switch (selectedAnswer){
                    case 1:
                        isRed = !isRed;
                        break;
                    case 2:
                        isFirst = !isFirst;
                        break;
                    case 3:
                        shouldPark = !shouldPark;
                        break;
                    case 4:
                        collectStack = !collectStack;
                        break;
                }
                showcaseAnswers();
            } else if(gamepad1.dpad_right){
                if(selectedAnswer == 5){
                    switch (selectedCase){
                        case -1:
                            selectedCase = 0;
                            break;
                        case 0:
                            selectedCase = 1;
                            break;
                        case 1:
                            selectedCase = 4;
                            break;
                        case 4:
                            selectedCase = -1;
                            break;
                    }
                }
                showcaseAnswers();
            } else if(gamepad1.dpad_left){
                if(selectedAnswer == 5){
                    switch (selectedCase){
                        case -1:
                            selectedCase = 4;
                            break;
                        case 4:
                            selectedCase = 1;
                            break;
                        case 1:
                            selectedCase = 0;
                            break;
                        case 0:
                            selectedCase = -1;
                            break;
                    }
                }
                showcaseAnswers();
            }
        }
    }

}