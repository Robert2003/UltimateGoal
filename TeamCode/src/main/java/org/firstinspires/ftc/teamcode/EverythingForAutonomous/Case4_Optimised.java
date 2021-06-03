package org.firstinspires.ftc.teamcode.EverythingForAutonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Case4_Optimised", group="Cases")
public class Case4_Optimised extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, true);

        Trajectory shootingPosition1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.02)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition1.end())
                .splineTo(new Vector2d(108.6, -28.7), 5.43)
                .addTemporalMarker(0.1, () -> {
                    robot.dropArm(700);
                })
                .build();
        Trajectory back = drive.trajectoryBuilder(firstWobble.end())
                .back(10)
                .build();
        Trajectory goNextToRings = drive.trajectoryBuilder(back.end(), true)
                .lineToLinearHeading(new Pose2d(56.6, -11, 0.02))
                .addTemporalMarker(0.1, () -> {
                    robot.toggleIntakeServo(true);
                    robot.dropArm(300);
                    robot.toggleIntake();
                    robot.toggleFlyWheel(true, 3010);
                })
                .build();
        Trajectory firstRing = drive.trajectoryBuilder(goNextToRings.end(), true)
                .lineToLinearHeading(new Pose2d(47, -11, 6.2))
                .build();
        Trajectory secondRing = drive.trajectoryBuilder(firstRing.end(), true)
                .lineToLinearHeading(new Pose2d(39, -11, 6.2))
                .build();
        Trajectory thirdRing = drive.trajectoryBuilder(secondRing.end(), true)
                .lineToLinearHeading(new Pose2d(34.5, -11, 6.25))
                .addTemporalMarker(0.1, () -> {
                    robot.toggleFlyWheel(true, 3080);
                })
                .build();
        Trajectory fourthRing = drive.trajectoryBuilder(thirdRing.end(), true)
                .lineToLinearHeading(new Pose2d(20, -11, 6.25))
                .build();
        Trajectory secondWobbleGoal = drive.trajectoryBuilder(fourthRing.end(), true)
                .lineToLinearHeading(new Pose2d(15.73, 1.56, 4.43))
                .addTemporalMarker(0.1, () -> {
                    robot.toggleFlyWheel(false);
                    robot.toggleIntake();
                    robot.dropArm(800);
                })
                .build();
        Trajectory forward = drive.trajectoryBuilder(secondWobbleGoal.end(), true)
                .lineToLinearHeading(new Pose2d(12.68, -11, 4.43))
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(forward.end())
                .splineTo(new Vector2d(105, -29), 5.43)
                .addTemporalMarker(0.1, () -> {
                    robot.dropArm(650);
                })
                .build();
        Trajectory park = drive.trajectoryBuilder(firstWobble.end())
                .lineTo(new Vector2d(80, -29))
                .addTemporalMarker(0.1, () -> {
                    robot.wobbleServo.setPosition(0.45);
                    robot.dropArm(20);
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.toggleFlyWheel(true, 2990);
        drive.followTrajectory(shootingPosition1);
        robot.shootrings(3);
        robot.toggleFlyWheel(false);
        drive.followTrajectory(firstWobble);
        robot.dropWobble();
        drive.followTrajectory(back);
        drive.followTrajectory(goNextToRings);
        drive.followTrajectory(firstRing);
        drive.followTrajectory(secondRing);
        sleep(500);
        robot.shootrings(3);
        drive.followTrajectory(thirdRing);
        drive.followTrajectory(fourthRing);
        sleep(500);
        robot.shootrings(3);
        drive.followTrajectory(secondWobbleGoal);
        drive.followTrajectory(forward);
        robot.grabWobble();
        drive.followTrajectory(dropSecondWobble);
        robot.dropWobble();
        drive.followTrajectory(park);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
