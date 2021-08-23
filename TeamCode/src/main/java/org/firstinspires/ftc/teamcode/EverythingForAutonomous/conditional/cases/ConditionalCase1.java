package org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.UltimateGoalDetectionConditional;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

public class ConditionalCase1 {

    UltimateGoalDetectionConditional goalDetection;
    SampleMecanumDrive drive;
    Trajectory shootingPositionTraj, wobbleTraj, parkTraj, stackTraj, strafeTraj, shootingPositionTraj2, parkTraj1, parkTraj2, backTraj, secondWobbleTraj, forwardTraj, dropSecondWobble;
    RobotDefinition_ForAuto robot;

    public ConditionalCase1(UltimateGoalDetectionConditional goalDetection) {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (goalDetection.getDeliverWobble()) {
            if (goalDetection.getIsRed()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(85.85, -2.508), 5.3733)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                stackTraj = drive.trajectoryBuilder(backTraj.end(), true)
                        .splineTo(new Vector2d(38.119, -14), 0.275 + 3.14)
                        .build();
                shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                        .splineTo(new Vector2d(55, -18), 0.01)
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(shootingPositionTraj2.end(), true)
                        .lineToLinearHeading(new Pose2d(40, -21, 3.14))
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .forward(12)
                        .build();
                dropSecondWobble = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(74.23, -13.25, 5.8))
                        .build();
            } else {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(95, 3), -4.712)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(3)
                        .build();
                stackTraj = drive.trajectoryBuilder(backTraj.end(), true)
                        .splineTo(new Vector2d(38.119, 14), -0.275 - 3.14)
                        .build();
                shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                        .splineTo(new Vector2d(55, 18), -0.01)
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(shootingPositionTraj2.end(), true)
                        .lineToLinearHeading(new Pose2d(38, 31, -3.14))
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .forward(12)
                        .build();
                dropSecondWobble = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(111, -5.5, -4.712))
                        .build();
            }
        } else {
            if (goalDetection.getIsRed()) {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, 3), 6.02)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineToLinearHeading(new Pose2d(92, 9.6, -1.5))
                            .build();
                    strafeTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(40, 9.6, 1.57))
                            .build();
                    if (goalDetection.getCollectStack()) {
                        stackTraj = drive.trajectoryBuilder(strafeTraj.end(), true)
                                .splineTo(new Vector2d(40, -14), -1.57)
                                .build();
                        shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                                .lineToLinearHeading(new Pose2d(58, 3, 6.02))
                                .build();
                    }
                    if (goalDetection.getPark() && goalDetection.getCollectStack())
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                                .lineToLinearHeading(new Pose2d(72, 11, 0))
                                .build();
                    else if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(strafeTraj.end())
                                .lineToLinearHeading(new Pose2d(72, 11, 0))
                                .build();
                } else {
                    wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(80.80, 2.45), 0.544)
                            .build();
                    shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(56.5, -5, 0.27))
                            .build();
                    if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                        stackTraj = drive.trajectoryBuilder(shootingPositionTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(40, 16.7, 5.5))
                                .build();
                        shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end())
                                .lineToLinearHeading(new Pose2d(56.5, -5, 0.27))
                                .build();
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                                .lineTo(new Vector2d(73, 6.5))
                                .build();
                    }
                    else if(goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                                .lineTo(new Vector2d(73, 6.5))
                                .build();
                }

            } else {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, -3), -5.95)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineToLinearHeading(new Pose2d(85, 0, 0.251))
                            .build();
                    strafeTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(40, -9.6, -1.57))
                            .build();
                    if (goalDetection.getCollectStack()) {
                        stackTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                                .splineTo(new Vector2d(40, 14), 1.57)
                                .build();
                        shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                                .splineTo(new Vector2d(58, -3), -5.93)
                                .build();
                    }
                    if (goalDetection.getPark() && goalDetection.getCollectStack())
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                                .lineToLinearHeading(new Pose2d(72, -11, 0))
                                .build();
                    else if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                                .lineToLinearHeading(new Pose2d(72, -11, 0))
                                .build();
                } else {
                    wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(77, -7.7), -0.544)
                            .build();
                    shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(56.5, 5., -0.26))
                            .build();
                    if (goalDetection.getPark() && goalDetection.getCollectStack()) {
                        stackTraj = drive.trajectoryBuilder(shootingPositionTraj.end(), true)
                                .lineToLinearHeading(new Pose2d(40, -16.7, -5.5))
                                .build();
                        shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end())
                                .lineToLinearHeading(new Pose2d(56.5, 5, -0.27))
                                .build();
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                                .lineTo(new Vector2d(73, 6.5))
                                .build();
                    }
                    else if(goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                                .lineTo(new Vector2d(73, 6.5))
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
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            goalDetection.sleep(500);
            robot.dropArm(600);
            goalDetection.sleep(500);
            robot.dropWobble();
            drive.followTrajectory(backTraj);
            robot.dropArm(300);
            robot.toggleFlyWheel(true, 2970);
            robot.toggleIntake();
            robot.toggleIntakeServo(true);
            drive.followTrajectory(stackTraj);
            drive.followTrajectory(shootingPositionTraj2);
            goalDetection.sleep(500);
            robot.shootrings(1);
            robot.toggleFlyWheel(false);
            robot.dropArm(780);
            robot.toggleIntake();
            drive.followTrajectory(secondWobbleTraj);
            goalDetection.sleep(300);
            drive.followTrajectory(forwardTraj);
            goalDetection.sleep(400);
            robot.grabWobble();
            drive.followTrajectory(dropSecondWobble);
            robot.dropArm(650);
            robot.dropWobble();
            robot.wobbleServo.setPosition(0.45);
            goalDetection.sleep(300);
            robot.dropArm(20);
            robot.toggleIntakeServo(false);
        } else {
            if (goalDetection.getIsFirst()) {
                robot.toggleFlyWheel(true, goalDetection.getIsRed() ? 2970 : 2950);
                drive.followTrajectory(shootingPositionTraj);
                robot.shootrings(3);
                robot.toggleFlyWheel(false);
                drive.followTrajectory(wobbleTraj);
                robot.dropArm(670);
                goalDetection.sleep(1000);
                robot.dropWobble();
                robot.dropArm(20);
                robot.wobbleServo.setPosition(0.45);
                goalDetection.sleep(400);
                if (goalDetection.getIsRed()) drive.followTrajectory(strafeTraj);
                if (goalDetection.getCollectStack()) {
                    robot.toggleIntakeServo(true);
                    robot.toggleFlyWheel(true, goalDetection.getIsFirst() ? 2950 : 2930);
                    robot.toggleIntake();
                    drive.followTrajectory(stackTraj);
                    drive.followTrajectory(shootingPositionTraj2);
                    goalDetection.sleep(500);
                    robot.shootrings(3);
                    goalDetection.sleep(500);
                    robot.toggleFlyWheel(false);
                    robot.toggleIntakeServo(false);
                }
                if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            } else {
                drive.followTrajectory(wobbleTraj);
                robot.dropArm(670);
                goalDetection.sleep(1000);
                robot.dropWobble();
                robot.toggleFlyWheel(true, 2970);
                drive.followTrajectory(shootingPositionTraj);
                robot.wobbleServo.setPosition(0.45);
                robot.dropArm(20);
                goalDetection.sleep(500);
                robot.shootrings(3, 1000);
                robot.toggleIntake();
                if(goalDetection.getCollectStack())
                {
                    drive.followTrajectory(stackTraj);
                    drive.followTrajectory(shootingPositionTraj2);
                    robot.shootrings(3, 500);
                    robot.toggleIntake();
                    robot.toggleFlyWheel(false);
                }
                if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            }
        }
    }

}
