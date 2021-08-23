/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert.Cases.ConditionalCase0_Robert;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert.Cases.ConditionalCase1_Robert;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert.Cases.ConditionalCase4_Robert;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.UltimateGoalDetectionConditional;
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

import static org.firstinspires.ftc.teamcode.Utils.Constants.*;

@TeleOp(name = "Robert Conditional", group = "Linear Opmode")
@Disabled
public class ConditionalLinear extends LinearOpMode
{
    boolean waitingAnswer = false;
    int selectedAnswer = 1;

    public static ElapsedTime runtime = new ElapsedTime();

    public static SampleMecanumDrive drive;
    public static RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();

    OpenCvWebcam webCam;
    SkystoneDeterminationPipeline pipeline;
    String WEBCAM_NAME = "Webcam";

    @Override
    public void runOpMode()
    {
        ask_questions();
        showcaseAnswers();
        confirmAnswers();

        drive = new SampleMecanumDrive(hardwareMap, false);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, WEBCAM_NAME), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline(this);
        webCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webCam.openCameraDeviceAsync(() -> webCam.startStreaming(800, 600, OpenCvCameraRotation.SIDEWAYS_LEFT)); //320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT)

        FtcDashboard.getInstance().startCameraStream(webCam, 0);

        robot.init(hardwareMap);

        ConditionalCase0_Robert conditionalCase0 = new ConditionalCase0_Robert(this);
        ConditionalCase1_Robert conditionalCase1 = new ConditionalCase1_Robert(this);
        ConditionalCase4_Robert conditionalCase4 = new ConditionalCase4_Robert(this);

        waitForStart();
        runtime.reset();
    }

    void ask_questions()
    {
        boolean left_right_buttonToggled = false;

        telemetry.addData("Q1", "Pe ce parte e robotul? \n" +
                "A - ROSU\nX - ALBASTRU");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer)
        {
            if (gamepad1.a)
            {
                side = Side.RED;
                waitingAnswer = false;
            } else if (gamepad1.x)
            {
                side = Side.BLUE;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        telemetry.addData("Q2", "Pe ce linie este robotul? \n" +
                "A - STANGA\nX - DREAPTA");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer)
        {
            if (gamepad1.a)
            {
                waitingAnswer = false;
                line = Line.LEFT;
            } else if (gamepad1.x)
            {
                waitingAnswer = false;
                line = Line.RIGHT;
            }
        }

        telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        telemetry.addData("Q3", "Robotul colecteaza starter stack-ul? \n" +
                "A - DA\nX - NU");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer)
        {
            if (gamepad1.a)
            {
                stack = Stack.COLLECT_STACK;
                waitingAnswer = false;
            } else if (gamepad1.x)
            {
                stack = Stack.DONT_COLLECT_STACK;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        telemetry.addData("Q4", "Robotul se parcheaza? \n" +
                "A - DA\nX - NU");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer)
        {
            if (gamepad1.a)
            {
                park = Park.PARK;
                waitingAnswer = false;
            } else if (gamepad1.x)
            {
                park = Park.NOT_PARK;
                waitingAnswer = false;
            }
        }

        telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        left_right_buttonToggled = false;
        while (!gamepad1.y && !isStopRequested())
        {
            telemetry.addData("Q5", "Cate secunde sta robotul pe loc inainte sa plece? \n" + startDelay + " secunde");
            telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
            telemetry.update();

            if (gamepad1.dpad_left || gamepad1.dpad_right)
            {
                if (gamepad1.dpad_left && startDelay > 0 && !left_right_buttonToggled)
                {
                    startDelay--;
                    left_right_buttonToggled = true;
                }
                if (gamepad1.dpad_right && startDelay < 30 && !left_right_buttonToggled)
                {
                    startDelay++;
                    left_right_buttonToggled = true;
                }
            } else
                left_right_buttonToggled = false;
        }


        left_right_buttonToggled = false;
        while (!gamepad1.y && !isStopRequested())
        {
            telemetry.addData("Q6", "In ultimele cate secunde se parcheaza? \n" + secondsToPark + " secunde");
            telemetry.addData("C", "Apasa Y pentru a trece la urmatoarea intrebare.");
            telemetry.update();

            if (gamepad1.dpad_left || gamepad1.dpad_right)
            {
                if (gamepad1.dpad_left && secondsToPark > 0 && !left_right_buttonToggled)
                {
                    secondsToPark--;
                    left_right_buttonToggled = true;
                }
                if (gamepad1.dpad_right && secondsToPark < 30 && !left_right_buttonToggled)
                {
                    secondsToPark++;
                    left_right_buttonToggled = true;
                }
            } else
                left_right_buttonToggled = false;
        }
    }

    void showcaseAnswers()
    {
        telemetry.addData("A1", "Partea albastra sau rosie: " + (side == Side.RED ? "ROSU" : "ALBASTRU") + (selectedAnswer == 1 ? " [X]" : ""));
        telemetry.addData("A2", "Linia din stanga sau din dreapta: " + (line == Line.LEFT ? "STANGA" : "DREAPTA") + (selectedAnswer == 2 ? " [X]" : ""));
        telemetry.addData("A3", "Colecteaza starter stack-ul: " + (stack == Stack.COLLECT_STACK ? "DA" : "NU") + (selectedAnswer == 3 ? " [X]" : ""));
        telemetry.addData("A4", "Se parcheaza: " + (park == Park.PARK ? "DA" : "NU") + (selectedAnswer == 4 ? " [X]" : ""));
        telemetry.addData("A5", "Robotul sta pe loc " + startDelay + " secunde inainte sa plece" + (selectedAnswer == 5 ? " [X]" : ""));
        telemetry.addData("A6", "Robotul se parcheaza in ultimele " + secondsToPark + " secunde" + (selectedAnswer == 6 ? " [X]" : ""));
        telemetry.addLine("Apasa B pentru a confirma selectia.");
        telemetry.update();
    }

    void confirmAnswers()
    {
        boolean modifyingAnswers = true;
        boolean up_down_buttonToggled = false, left_right_buttonToggled = false;

        while (modifyingAnswers && !isStopRequested())
        {
            if (gamepad1.b)
            {
                modifyingAnswers = false;
                telemetry.addLine("Gata de lansare frate!");
                telemetry.update();
            }

            if (gamepad1.dpad_down || gamepad1.dpad_up)
            {
                if (gamepad1.dpad_down == true && !up_down_buttonToggled)
                {
                    selectedAnswer++;
                    up_down_buttonToggled = true;
                } else if (gamepad1.dpad_down == true && !up_down_buttonToggled)
                {
                    selectedAnswer--;
                    up_down_buttonToggled = true;
                }
            } else
                up_down_buttonToggled = false;

            if (selectedAnswer >= 6)
                selectedAnswer = 1;
            if (selectedAnswer < 1)
                selectedAnswer = 6;


            if (gamepad1.dpad_left || gamepad1.dpad_right)
            {
                if ((gamepad1.dpad_left || gamepad1.dpad_right) && !left_right_buttonToggled)
                {
                    left_right_buttonToggled = true;

                    if (selectedAnswer == 1)
                    {
                        if (side == Side.RED)
                            side = Side.BLUE;
                        else
                            side = Side.RED;

                        showcaseAnswers();
                    }

                    if (selectedAnswer == 2)
                    {
                        if (line == Line.LEFT)
                            line = Line.RIGHT;
                        else
                            line = Line.LEFT;

                        showcaseAnswers();
                    }

                    if (selectedAnswer == 3)
                    {
                        if (stack == Stack.COLLECT_STACK)
                            stack = Stack.DONT_COLLECT_STACK;
                        else
                            stack = Stack.COLLECT_STACK;

                        showcaseAnswers();
                    }

                    if (selectedAnswer == 4)
                    {
                        if (park == Park.PARK)
                            park = Park.NOT_PARK;
                        else
                            park = Park.PARK;

                        showcaseAnswers();
                    }

                    if (selectedAnswer == 5)
                    {
                        if (gamepad1.dpad_left && startDelay > 0)
                            startDelay--;
                        if (gamepad1.dpad_right && startDelay < 30)
                            startDelay++;
                    }

                    if (selectedAnswer == 6)
                    {
                        if (gamepad1.dpad_left && secondsToPark > 0)
                            secondsToPark--;
                        if (gamepad1.dpad_right && secondsToPark < 30)
                            secondsToPark++;
                    }
                }
            } else
                left_right_buttonToggled = false;
        }
    }

    public SampleMecanumDrive getDrive()
    {
        return drive;
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */

        //public Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 15);
        public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181 * 2.5, 15 * 2.5); //red first
        public static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(195 * 2.5, 270 * 2.5); //red second
        public static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(194 * 2.5, 260 * 2.5); //blue first
        public static Point REGION4_TOPLEFT_ANCHOR_POINT = new Point(181 * 2.5, 0 * 2.5); //blue second
        Point region1_pointA;
        Point region1_pointB;

        public SkystoneDeterminationPipeline(ConditionalLinear cond)
        {
            boolean isRed = side == Side.RED ? true : false;
            boolean isFirst = line == Line.LEFT ? true : false;
            if (isRed)
            {
                if (isFirst)
                {
                    cond.telemetry.addData("C", "red first");
                } else
                {
                    REGION1_TOPLEFT_ANCHOR_POINT = REGION2_TOPLEFT_ANCHOR_POINT;
                    cond.telemetry.addData("C", "red second");
                }
            } else
            {
                if (isFirst)
                {
                    REGION1_TOPLEFT_ANCHOR_POINT = REGION3_TOPLEFT_ANCHOR_POINT;
                    cond.telemetry.addData("C", "blue first");
                } else
                {
                    REGION1_TOPLEFT_ANCHOR_POINT = REGION4_TOPLEFT_ANCHOR_POINT;
                    cond.telemetry.addData("C", "blue second");
                }
            }
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
            cond.telemetry.update();
        }

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


        public static final float REGION_WIDTH = 35 * 2.5f;
        public static final float REGION_HEIGHT = 25 * 2.5f;

        final int FOUR_RING_THRESHOLD = 149;
        final int ONE_RING_THRESHOLD = 136;


        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.FOUR;

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

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

             */

            position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD)
            {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD)
            {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.ONE;
            } else
            {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
