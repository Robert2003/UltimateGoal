package org.firstinspires.ftc.teamcode.EverythingForTeleOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.SampleMecanumDriveCancelable;

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@TeleOp(group = "advanced")
public class AugmentedDriving extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }


    Mode currentMode = Mode.DRIVER_CONTROL;

    RobotDefinition_ForTeleOP robot = new RobotDefinition_ForTeleOP();
    private ElapsedTime runtime = new ElapsedTime();

    boolean supress2 = false, buttonPressed = false;
    double second_surpress = 1;

    @Override
    public void runOpMode() throws InterruptedException {
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

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("arm", robot.wobbleArm.getCurrentPosition());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.update();

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

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode)
            {
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

                    /*if (gamepad1.y) {
                        Vector2d difference = targetPosition.minus(poseEstimate.vec());
                        // Obtain the target angle for feedback and derivative for feedforward
                        theta = difference.angle();

                        drive.turnAsync(Angle.normDelta(theta - poseEstimate.getHeading()));

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }*/

                    if(gamepad1.y)
                    {
                        robot.toggleFlyWheel(true);

                        drive.setPoseEstimate(new Pose2d(0,0,0));
                        Trajectory firstStrafe = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(0, -25.5))
                                .addDisplacementMarker(() -> {
                                    try {
                                        robot.shootrings(1);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                })
                                .build();
                        Trajectory secondStrafe = drive.trajectoryBuilder(firstStrafe.end())
                                .lineTo(new Vector2d(0, -32))
                                .addDisplacementMarker(() -> {
                                    try {
                                        robot.shootrings(1);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                })
                                .build();
                        Trajectory thirdStrafe = drive.trajectoryBuilder(secondStrafe.end())
                                .lineTo(new Vector2d(0, -37.5))
                                .addDisplacementMarker(() -> {
                                    try {
                                        robot.shootrings(1);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }
                                })
                                .build();
                        drive.followTrajectoryAsync(firstStrafe);
                        drive.followTrajectoryAsync(secondStrafe);
                        drive.followTrajectoryAsync(thirdStrafe);

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
