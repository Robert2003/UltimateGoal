package org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.UltimateGoalDetectionConditional;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

public class ConditionalCase4 {

    UltimateGoalDetectionConditional goalDetection;
    SampleMecanumDrive drive;
    Trajectory shootingPositionTraj, wobbleTraj, backTraj, parkTraj, goNextToRingsTraj, firstRingTraj, secondRingTraj, thirdRingTraj, fourthRingTraj, toTheWallTraj;
    RobotDefinition_ForAuto robot;

    public ConditionalCase4(UltimateGoalDetectionConditional goalDetection) {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (goalDetection.getIsRed()) {
            if (goalDetection.getIsFirst()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(108.6, -28.7), 5.43)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                if (goalDetection.getCollectStack()) {
                    goNextToRingsTraj = drive.trajectoryBuilder(backTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(56.6, -11, 0.02))
                            .addTemporalMarker(0.1, () -> {
                                robot.toggleIntakeServo(true);
                                robot.dropArm(300);
                                robot.toggleIntake();
                                robot.toggleFlyWheel(true, 3010);
                            })
                            .build();
                    firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(47, -11, 6.2))
                            .build();
                    secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(39, -11, 6.2))
                            .build();
                    thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(34.5, -11, 6.25))
                            .addTemporalMarker(0.1, () -> {
                                robot.toggleFlyWheel(true, 3080);
                            })
                            .build();
                    fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(20, -11, 6.25))
                            .build();
                }
                if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                    parkTraj = drive.trajectoryBuilder(fourthRingTraj.end())
                            .lineTo(new Vector2d(80, 49))
                            .addTemporalMarker(0.1, () -> {
                                robot.wobbleServo.setPosition(0.45);
                                robot.dropArm(20);
                            })
                            .build();
                } else if (goalDetection.getPark()) {
                    parkTraj = drive.trajectoryBuilder(backTraj.end())
                            .lineTo(new Vector2d(70, -29))
                            .addTemporalMarker(0.1, () -> {
                                robot.wobbleServo.setPosition(0.45);
                                robot.dropArm(20);
                            })
                            .build();
                }

            } else {
                toTheWallTraj = drive.trajectoryBuilder(new Pose2d())
                        .lineTo(new Vector2d(0, -7))
                        .build();
                wobbleTraj = drive.trajectoryBuilder(toTheWallTraj.end())
                        .lineTo(new Vector2d(102, -7))
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .lineToLinearHeading(new Pose2d(56.5, -5., -0.31))
                        .build();
                if (goalDetection.getPark())
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineTo(new Vector2d(68, -6.5))
                            .build();

            }
        } else {
            if (goalDetection.getIsFirst()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(108.6, 28.7), -5.43)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                if (goalDetection.getCollectStack()) {
                    goNextToRingsTraj = drive.trajectoryBuilder(backTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(56.6, 11, -0.02))
                            .addTemporalMarker(0.1, () -> {
                                robot.toggleIntakeServo(true);
                                robot.dropArm(300);
                                robot.toggleIntake();
                                robot.toggleFlyWheel(true, 3010);
                            })
                            .build();
                    firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(47, 11, -6.2))
                            .build();
                    secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(39, 11, -6.2))
                            .build();
                    thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(34.5, 11, -6.25))
                            .addTemporalMarker(0.1, () -> {
                                robot.toggleFlyWheel(true, 3080);
                            })
                            .build();
                    fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(20, 11, -6.25))
                            .build();
                }
                if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                    parkTraj = drive.trajectoryBuilder(fourthRingTraj.end())
                            .lineTo(new Vector2d(80, 29))
                            .addTemporalMarker(0.1, () -> {
                                robot.wobbleServo.setPosition(0.45);
                                robot.dropArm(20);
                            })
                            .build();
                } else if (goalDetection.getPark()) {
                    parkTraj = drive.trajectoryBuilder(backTraj.end())
                            .lineTo(new Vector2d(70, 29))
                            .addTemporalMarker(0.1, () -> {
                                robot.wobbleServo.setPosition(0.45);
                                robot.dropArm(20);
                            })
                            .build();
                }

            } else {
                toTheWallTraj = drive.trajectoryBuilder(new Pose2d())
                        .lineTo(new Vector2d(0, 7))
                        .build();
                wobbleTraj = drive.trajectoryBuilder(toTheWallTraj.end())
                        .lineTo(new Vector2d(102, 7))
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .lineToLinearHeading(new Pose2d(56.5, 5., -0.31))
                        .build();
                if (goalDetection.getPark())
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineTo(new Vector2d(68, 6.5))
                            .build();

            }
        }
    }

    public void runCase() throws InterruptedException {
        robot.init(goalDetection.hardwareMap);
        goalDetection.sleep(goalDetection.getStartDelay());
        if (goalDetection.getIsFirst()) {
            robot.toggleFlyWheel(true, 2990);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(500);
            robot.dropWobble();
            drive.followTrajectory(backTraj);
            if (goalDetection.getCollectStack()) {
                drive.followTrajectory(goNextToRingsTraj);
                drive.followTrajectory(firstRingTraj);
                drive.followTrajectory(secondRingTraj);
                goalDetection.sleep(500);
                robot.shootrings(3);
                drive.followTrajectory(thirdRingTraj);
                drive.followTrajectory(fourthRingTraj);
                goalDetection.sleep(500);
                robot.shootrings(3);
            }
            if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            else {
                robot.wobbleServo.setPosition(0.45);
                goalDetection.sleep(400);
                robot.dropArm(20);
            }
        } else if (!goalDetection.getIsFirst()) {
            drive.followTrajectory(toTheWallTraj);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            robot.wobbleServo.setPosition(0.45);
            robot.dropArm(20);
            robot.shootrings(3, 1000);
            robot.toggleFlyWheel(false);
            if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
        }
    }

}
