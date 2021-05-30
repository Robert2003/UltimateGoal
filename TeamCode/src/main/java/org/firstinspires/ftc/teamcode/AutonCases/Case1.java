package org.firstinspires.ftc.teamcode.AutonCases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Case1", group="Cases")
public class Case1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);
        //drive.setPoseEstimate(new Pose2d(-60, -22.5, 0));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory shootingPosition1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.127)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition1.end())
                .splineTo(new Vector2d(85.85, -7.508), 5.3733)
                .build();
        Trajectory takeOneDisk = drive.trajectoryBuilder(firstWobble.end(), true)
                .splineTo(new Vector2d(38.119, -14), 0.275+3.14)
                .build();
        Trajectory shootingPosition2 = drive.trajectoryBuilder(takeOneDisk.end(), false)
                .splineTo(new Vector2d(55, -11.37), 0.01)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(shootingPosition2.end(), true)
                .lineToLinearHeading(new Pose2d(30, -20, 3.16))
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(grabSecondWobble.end())
                .lineToLinearHeading(new Pose2d(74.23, -13.25, 5.8))
                .build();

        drive.followTrajectory(shootingPosition1);
        drive.followTrajectory(firstWobble);
        drive.followTrajectory(takeOneDisk);
        drive.followTrajectory(shootingPosition2);
        drive.followTrajectory(grabSecondWobble);
        drive.followTrajectory(dropSecondWobble);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
