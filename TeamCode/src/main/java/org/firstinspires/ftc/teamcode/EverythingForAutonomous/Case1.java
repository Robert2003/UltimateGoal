package org.firstinspires.ftc.teamcode.EverythingForAutonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Case1", group="Cases")
@Disabled
public class Case1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);


        Trajectory shootingPosition1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.02)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition1.end())
                .splineTo(new Vector2d(85.85, -2.508), 5.3733)
                .build();
        Trajectory back = drive.trajectoryBuilder(firstWobble.end())
                .back(10)
                .build();
        Trajectory takeOneDisk = drive.trajectoryBuilder(back.end(), true)
                .splineTo(new Vector2d(38.119, -14), 0.275+3.14)
                .build();
        Trajectory shootingPosition2 = drive.trajectoryBuilder(takeOneDisk.end(), false)
                .splineTo(new Vector2d(55, -18), 0.01)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(shootingPosition2.end(), true)
                .lineToLinearHeading(new Pose2d(40, -21, 3.14))
                .build();
        Trajectory forward = drive.trajectoryBuilder(grabSecondWobble.end(), true)
                .forward(10)
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(74.23, -13.25, 5.8))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.toggleFlyWheel(true, 2970);
        drive.followTrajectory(shootingPosition1);
        robot.shootrings(3);
        robot.toggleFlyWheel(false);
        drive.followTrajectory(firstWobble);
        robot.dropArm(650);
        robot.dropWobble();
        drive.followTrajectory(back);
        robot.dropArm(300);
        robot.toggleFlyWheel(true, 3000);
        robot.toggleIntake();
        drive.followTrajectory(takeOneDisk);
        drive.followTrajectory(shootingPosition2);
        sleep(500);
        robot.shootrings(1);
        robot.toggleFlyWheel(false);
        robot.dropArm(780);
        robot.toggleIntake();
        drive.followTrajectory(grabSecondWobble);
        sleep(400);
        drive.followTrajectory(forward);
        sleep(800);
        robot.grabWobble();
        drive.followTrajectory(dropSecondWobble);
        robot.dropArm(650);
        robot.dropWobble();
        robot.wobbleServo.setPosition(0.45);
        sleep(400);
        robot.dropArm(20);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
