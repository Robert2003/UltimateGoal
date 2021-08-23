package org.firstinspires.ftc.teamcode.EverythingForTeleOP.secondrobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EverythingForTeleOP.RobotDefinition_ForTeleOP;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.FromRoadRunner.advanced.SampleMecanumDriveCancelable;

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@TeleOp(group = "advanced")
@Disabled
public class AugmentedDrivingSecondRobot extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }


    Mode currentMode = Mode.DRIVER_CONTROL;

    RobotDefinition_SecondRobot robot = new RobotDefinition_SecondRobot();
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
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.right_stick_y*second_surpress,
                            gamepad1.right_stick_x*second_surpress,
                            (-gamepad1.right_trigger + gamepad1.left_trigger)*second_surpress
                    )
            );
        }
    }
}
