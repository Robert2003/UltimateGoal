package org.firstinspires.ftc.teamcode.T265;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.EverythingForTeleOP.RobotDefinition_ForTeleOP;
import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.Utils.Constants;
import org.firstinspires.ftc.teamcode.Utils.MathFunctions;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
@Disabled
public class TeleOP_Augmented_T265 extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }
    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(45, 45);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    public RobotDefinition_ForTeleOP robot = new RobotDefinition_ForTeleOP();



    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private ElapsedTime runtime = new ElapsedTime();
    boolean supress2 = false, buttonPressed = false;
    double second_surpress = 1;

    private static T265Camera slamra = null;
    com.arcrobotics.ftclib.geometry.Pose2d startingPose = new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0));

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        robot.init(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);



        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        slamra.stop();
        slamra.setPose(startingPose);

        waitForStart();

        slamra.start();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            final int robotRadius = 9; // inches

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();

            T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
            if (up == null) return;

            // We divide by 0.0254 to convert meters to inches
            Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
            Rotation2d rotation = up.pose.getRotation();

            field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
            double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
            double x1 = translation.getX() + arrowX / 2, y1 = translation.getY() + arrowY / 2;
            double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
            field.strokeLine(x1, y1, x2, y2);

            dashboard.sendTelemetryPacket(packet);

            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            Pose2d RobotPosition = new Pose2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254, rotation.getRadians());
            Pair<Double, Double> Position = MathFunctions.RobotPosition(RobotPosition.vec(), Constants.RedTowerPose);

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("Odometry X", poseEstimate.getX());
            telemetry.addData("Odometry Y", poseEstimate.getY());
            telemetry.addData("Odometry Heading", poseEstimate.getHeading());
            telemetry.addData(" ", " ");
            telemetry.addData("T265 X", up.pose.getTranslation().getX() / 0.0254);
            telemetry.addData("T265 Y", up.pose.getTranslation().getY() / 0.0254);
            telemetry.addData("T265 Heading", up.pose.getRotation());
            telemetry.addData(" ", " ");
            telemetry.addData("Distance", Position.first);
            telemetry.addData("Angle", Angle.normDelta(Position.second - poseEstimate.getHeading()));
            telemetry.update();

            SupressButtons();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    if(gamepad1.right_bumper)
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        gamepad1.right_stick_y*0.30,
                                        gamepad1.right_stick_x*0.30,
                                        (-gamepad1.right_trigger + gamepad1.left_trigger)*0.30
                                )
                        );
                    else
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        gamepad1.right_stick_y*second_surpress,
                                        gamepad1.right_stick_x*second_surpress,
                                        (-gamepad1.right_trigger + gamepad1.left_trigger)*second_surpress
                                )
                        );

                    if(gamepad2.left_bumper)
                    {
                        if(runtime.seconds() > 1)
                        {
                            runtime.reset();
                            robot.servo.setPosition(0);
                        }
                        else if(runtime.seconds() > 0.5)
                            robot.servo.setPosition(0.55);
                    }
                    else
                        robot.servo.setPosition(0.55);

                    robot.Gamepad1Actions(gamepad1);
                    robot.Gamepad2Actions(gamepad2, poseEstimate.getX(), poseEstimate.getY());

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }

    void SupressButtons()
    {
        if(gamepad1.dpad_up)
            robot.intakeServo.setPosition(1);
        if(gamepad1.dpad_down)
            robot.intakeServo.setPosition(0.42);

        if(gamepad1.left_bumper)
        {
            if(!buttonPressed)
            {
                supress2 = !supress2;
                buttonPressed = true;
            }
        }
        else
            buttonPressed = false;

        if(supress2)
            second_surpress = 0.6;
        else
            second_surpress = 1;
    }
}
