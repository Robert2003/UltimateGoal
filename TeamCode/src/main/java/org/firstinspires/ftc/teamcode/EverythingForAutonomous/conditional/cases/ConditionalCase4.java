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
    Trajectory shootingPositionTraj, wobbleTraj, backTraj, parkTraj, goNextToRingsTraj, firstRingTraj, secondRingTraj, thirdRingTraj, fourthRingTraj, toTheWallTraj,
            secondWobbleTraj, forwardTraj, dropSecondWobble;
    RobotDefinition_ForAuto robot;

    public ConditionalCase4(UltimateGoalDetectionConditional goalDetection) {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (goalDetection.getDeliverWobble()) {
            if (goalDetection.getIsRed()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(108.6, -28.7), 5.43)
                        .addTemporalMarker(0.1, () -> {
                            robot.dropArm(630);
                        })
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                goNextToRingsTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(56.6, -11, 0.02))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleIntakeServo(true);
                            robot.dropArm(300);
                            robot.toggleIntake();
                            robot.toggleFlyWheel(true, 3000);
                        })
                        .build();
                firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(47, -11, 6.2))
                        .build();
                secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(37, -11, 6.2))
                        .build();
                thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(32, -11, 6.25))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleFlyWheel(true, 3080);
                        })
                        .build();
                fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(20, -11, 6.25))
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(fourthRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(15.73, 1.56, 4.43))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleFlyWheel(false);
                            robot.toggleIntake();
                            robot.dropArm(800);
                        })
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(12.68, -11, 4.43))
                        .build();
                dropSecondWobble = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(103, -29, 5.496))
                        .addTemporalMarker(0.1, () -> {
                            robot.dropArm(650);
                        })
                        .build();
                parkTraj = drive.trajectoryBuilder(dropSecondWobble.end())
                        .lineTo(new Vector2d(70, -29))
                        .addTemporalMarker(0.1, () -> {
                            robot.wobbleServo.setPosition(0.45);
                            robot.dropArm(20);
                        })
                        .build();
            } else {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(120, 21), -4.712)
                        .addTemporalMarker(0.1, () -> {
                            robot.dropArm(630);
                        })
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                goNextToRingsTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(56.6, 14, -0.02))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleIntakeServo(true);
                            robot.dropArm(300);
                            robot.toggleIntake();
                            robot.toggleFlyWheel(true, 2970);
                        })
                        .build();
                firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(47, 14, -6.2))
                        .build();
                secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(37, 14, -6.2))
                        .build();
                thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(32, 14, -6.25))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleFlyWheel(true, 3000);
                        })
                        .build();
                fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(20, 11, -6.25))
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(fourthRingTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(38, 35, -3.14))
                        .addTemporalMarker(0.1, () -> {
                            robot.toggleFlyWheel(false);
                            robot.toggleIntake();
                            robot.dropArm(800);
                        })
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(12.68, 11, -4.43))
                        .build();
                dropSecondWobble = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(132, 18.5, -4.712))
                        .addTemporalMarker(0.1, () -> {
                            robot.dropArm(650);
                        })
                        .build();
                parkTraj = drive.trajectoryBuilder(dropSecondWobble.end())
                        .lineTo(new Vector2d(70, 29))
                        .addTemporalMarker(0.1, () -> {
                            robot.wobbleServo.setPosition(0.45);
                            robot.dropArm(20);
                        })
                        .build();
            }
        } else {
            if (goalDetection.getIsRed()) {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, 3), 6.12)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .splineTo(new Vector2d(113.5, -14), 5.05)
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
                                    robot.toggleFlyWheel(true, 2970);
                                })
                                .build();
                        firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(47, -11, 6.25))
                                .build();
                        secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(39, -11, 6.25))
                                .build();
                        thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(34.5, -11, 6.25))
                                .addTemporalMarker(0.1, () -> {
                                    robot.toggleFlyWheel(true, 3000);
                                })
                                .build();
                        fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(20, -11, 6.25))
                                .build();
                    }
                    if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                        parkTraj = drive.trajectoryBuilder(fourthRingTraj.end())
                                .lineTo(new Vector2d(72, 12))
                                .addTemporalMarker(0.1, () -> {
                                    robot.wobbleServo.setPosition(0.45);
                                    robot.dropArm(20);
                                })
                                .build();
                    } else if (goalDetection.getPark()) {
                        parkTraj = drive.trajectoryBuilder(backTraj.end())
                                .lineTo(new Vector2d(72, 12))
                                .addTemporalMarker(0.1, () -> {
                                    robot.wobbleServo.setPosition(0.45);
                                    robot.dropArm(20);
                                })
                                .build();
                    }

                } else {
                    toTheWallTraj = drive.trajectoryBuilder(new Pose2d())
                            .lineTo(new Vector2d(0, -3))
                            .build();
                    shootingPositionTraj = drive.trajectoryBuilder(toTheWallTraj.end())
                            .lineToLinearHeading(new Pose2d(56.5, -3, 0.2))
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineToLinearHeading(new Pose2d(102, 8, -1.2))
                            .build();
                    if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                                .lineTo(new Vector2d(68, 5))
                                .build();
                }
            } else {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, -3), -5.95)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .splineTo(new Vector2d(118.5, 15), -4.95)
                            .build();
                    backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .back(10)
                            .build();
                    if (goalDetection.getCollectStack()) {
                        goNextToRingsTraj = drive.trajectoryBuilder(backTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(56.6, 14, -0.02))
                                .addTemporalMarker(0.1, () -> {
                                    robot.toggleIntakeServo(true);
                                    robot.dropArm(300);
                                    robot.toggleIntake();
                                    robot.toggleFlyWheel(true, 2960);
                                })
                                .build();
                        firstRingTraj = drive.trajectoryBuilder(goNextToRingsTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(47, 11, -6.25))
                                .build();
                        secondRingTraj = drive.trajectoryBuilder(firstRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(39, 11, -6.25))
                                .build();
                        thirdRingTraj = drive.trajectoryBuilder(secondRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(34.5, 11, -6.25))
                                .addTemporalMarker(0.1, () -> {
                                    robot.toggleFlyWheel(true, 3000);
                                })
                                .build();
                        fourthRingTraj = drive.trajectoryBuilder(thirdRingTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(20, 11, -6.25))
                                .build();
                    }
                    if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                        parkTraj = drive.trajectoryBuilder(fourthRingTraj.end())
                                .lineTo(new Vector2d(72, -11))
                                .addTemporalMarker(0.1, () -> {
                                    robot.wobbleServo.setPosition(0.45);
                                    robot.dropArm(20);
                                })
                                .build();
                    } else if (goalDetection.getPark()) {
                        parkTraj = drive.trajectoryBuilder(backTraj.end())
                                .lineTo(new Vector2d(72, -11))
                                .addTemporalMarker(0.1, () -> {
                                    robot.wobbleServo.setPosition(0.45);
                                    robot.dropArm(20);
                                })
                                .build();
                    }

                } else {
                    toTheWallTraj = drive.trajectoryBuilder(new Pose2d())
                            .lineTo(new Vector2d(0, 3))
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(toTheWallTraj.end())
                            .lineToLinearHeading(new Pose2d(104, 3, 0))
                            .build();
                    shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(56.5, 0, -0.14))
                            .build();
                    if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                                .lineTo(new Vector2d(68, 0))
                                .build();

                }
            }
        }
    }

    public void runCase() throws InterruptedException {
        robot.init(goalDetection.hardwareMap);
        goalDetection.sleep(goalDetection.getStartDelay());
        if (goalDetection.getDeliverWobble()) {
            drive = new SampleMecanumDrive(goalDetection.hardwareMap, true);
            robot.toggleFlyWheel(true, 2990);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropWobble();
            drive.followTrajectory(goNextToRingsTraj);
            drive.followTrajectory(firstRingTraj);
            drive.followTrajectory(secondRingTraj);
            goalDetection.sleep(300);
            robot.shootrings(2);
            drive.followTrajectory(thirdRingTraj);
            drive.followTrajectory(fourthRingTraj);
            goalDetection.sleep(400);
            robot.shootrings(3);
            drive.followTrajectory(secondWobbleTraj);
            drive.followTrajectory(forwardTraj);
            robot.grabWobble();
            drive.followTrajectory(dropSecondWobble);
            robot.dropWobble();
            drive.followTrajectory(parkTraj);
        } else {
            if (goalDetection.getIsFirst()) {
                robot.toggleFlyWheel(true, 2920);
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
                robot.toggleFlyWheel(true, 2950);
                drive.followTrajectory(shootingPositionTraj);
                robot.shootrings(3, 1000);
                robot.toggleFlyWheel(false);
                drive.followTrajectory(wobbleTraj);
                robot.dropArm(670);
                goalDetection.sleep(1000);
                robot.dropWobble();
                robot.wobbleServo.setPosition(0.45);
                robot.dropArm(20);
                if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            }
        }
    }

}
