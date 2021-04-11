package org.firstinspires.ftc.teamcode.AugmentedDriving;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

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
//@Disabled
public class AugmentedDriving extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    RobotConstants robot = new RobotConstants();
    Vector2d targetPosition = new Vector2d(-131, 13);
    double theta=0.0;
    private ElapsedTime runtime = new ElapsedTime();

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(-62, 16.4);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-62, 16.4);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();


            double deltaY = Math.abs(-126-poseEstimate.getX())*2.54;
            double deltaX = Math.abs(14.56-poseEstimate.getY())*2.54;
            double hp = sqrt(deltaX*deltaX+deltaY*deltaY)/100;
            double sp = speed(poseEstimate.getX(), poseEstimate.getY());
            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("theta", Math.toDegrees(theta));
            telemetry.addData("speed", sp);
            telemetry.addData("hypot", hp);
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    if(gamepad1.right_bumper)
                        drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.right_stick_y*0.35,
                                    -gamepad1.right_stick_x*0.35,
                                    (-gamepad1.right_trigger + gamepad1.left_trigger)*0.35
                            )
                        );
                    else
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.right_stick_y,
                                        -gamepad1.right_stick_x,
                                        (-gamepad1.right_trigger + gamepad1.left_trigger)
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

                    /*if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate, true)
                                .splineTo(targetAVector, targetAHeading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate, true)
                                .lineTo(targetBVector)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else*/
                    if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

                        Vector2d difference = targetPosition.minus(poseEstimate.vec());
                        // Obtain the target angle for feedback and derivative for feedforward
                        theta = difference.angle()+Math.PI;

                        drive.turnAsync(Angle.normDelta(theta - poseEstimate.getHeading()));

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
    double speed(double X, double Y)
    {
        double deltaY = Math.abs(-131-X)*2.54;
        double deltaX = Math.abs(14.56-Y)*2.54;
        double hypot = sqrt(deltaX*deltaX+deltaY*deltaY)/100;
        double flyWheelRadius = 0.05;
        double alfa = 37.0;
        double h = 0.165;
        double numarator = -4.9*hypot*hypot;
        double numitor = (0.93-h-tan(toRadians(alfa))*hypot)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numarator/numitor;
        return 3*sqrt(cat)/flyWheelRadius*60/(2*Math.PI);
    }
}
