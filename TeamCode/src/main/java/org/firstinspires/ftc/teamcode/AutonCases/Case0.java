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
@Autonomous(name="Case0", group="Cases")
public class Case0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory shootingPosition = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.127)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition.end())
                .splineTo(new Vector2d(110, -27), 5.3)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(31.6, -20), 0)
                .build();

        drive.followTrajectory(shootingPosition);
        drive.followTrajectory(firstWobble);
        drive.turn(3.14-drive.getPoseEstimate().getHeading());
        drive.followTrajectory(grabSecondWobble);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
