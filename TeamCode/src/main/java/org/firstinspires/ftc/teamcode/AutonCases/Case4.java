package org.firstinspires.ftc.teamcode.AutonCases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Case4", group="Cases")
public class Case4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, true);
        //drive.setPoseEstimate(new Pose2d(-60, -22.5, 0));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory shootingPosition1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.127)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition1.end())
                .splineTo(new Vector2d(108.5, -29.7), 5.55)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(firstWobble.end(), true)
                .lineToLinearHeading(new Pose2d(30, -20, 3.16))
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(grabSecondWobble.end())
                .lineToLinearHeading(new Pose2d(99, -31, 5.76))
                .build();
        Trajectory takedisk = drive.trajectoryBuilder(dropSecondWobble.end(), true)
                .lineToLinearHeading(new Pose2d(54, -13, 0.014))
                .build();
        Trajectory takeOneDisk = drive.trajectoryBuilder(firstWobble.end(), true)
                .splineTo(new Vector2d(38.119, -14), 0.275+3.14)
                .build();
        Trajectory shootingPosition2 = drive.trajectoryBuilder(takeOneDisk.end(), false)
                .splineTo(new Vector2d(55, -11.37), 0.01)
                .build();

        drive.followTrajectory(shootingPosition1);
        drive.followTrajectory(firstWobble);
        drive.followTrajectory(grabSecondWobble);
        drive.followTrajectory(dropSecondWobble);
        drive.followTrajectory(takedisk);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
