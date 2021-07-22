package org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class CameraAdjusting extends LinearOpMode {

    OpenCvWebcam webCam;
    SkystoneDeterminationPipeline pipeline;
    String WEBCAM_NAME = "Webcam";

    @Override
    public void runOpMode() throws InterruptedException {
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
        waitForStart();
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {
        /*
         * An enum to define the skystone position
         */

        public Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 15);
        public Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 15);
        public Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(181, 15);
        public Point REGION4_TOPLEFT_ANCHOR_POINT = new Point(181, 15);
        Point region1_pointA;
        Point region1_pointB;
        Point region2_pointA;
        Point region2_pointB;
        Point region3_pointA;
        Point region3_pointB;
        Point region4_pointA;
        Point region4_pointB;

        public SkystoneDeterminationPipeline() {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 15); //red first
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(195, 270); //red second
            REGION3_TOPLEFT_ANCHOR_POINT = new Point(194, 260); //blue first
            REGION4_TOPLEFT_ANCHOR_POINT = new Point(181, 0); //blue second

            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            region2_pointA = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x,
                    REGION2_TOPLEFT_ANCHOR_POINT.y);
            region2_pointB = new Point(
                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            region3_pointA = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x,
                    REGION3_TOPLEFT_ANCHOR_POINT.y);
            region3_pointB = new Point(
                    REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

            region4_pointA = new Point(
                    REGION4_TOPLEFT_ANCHOR_POINT.x,
                    REGION4_TOPLEFT_ANCHOR_POINT.y);
            region4_pointB = new Point(
                    REGION4_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION4_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }


        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar LIGHT_BLUE = new Scalar(52, 229, 235);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar LIGHT_RED = new Scalar(255, 120, 120);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */


        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

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

            /*Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

             */

            position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = UltimateGoalDetectionConditional.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    LIGHT_RED, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region4_pointA, // First point which defines the rectangle
                    region4_pointB, // Second point which defines the rectangle
                    LIGHT_BLUE, // The color the rectangle is drawn in
                    2); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }

}
