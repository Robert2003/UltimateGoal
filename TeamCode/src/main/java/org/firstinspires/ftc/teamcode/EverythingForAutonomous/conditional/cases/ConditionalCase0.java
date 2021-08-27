package org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.UltimateGoalDetectionConditional;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

public class ConditionalCase0 {

    UltimateGoalDetectionConditional goalDetection;
    SampleMecanumDrive drive;
    Trajectory left, shootingPositionTraj, wobbleTraj, parkTraj, waitForOthersTraj, backTraj, forwardTraj, dropSecondWobbleTaj, secondWobbleTraj;
    RobotDefinition_ForAuto robot;

    public ConditionalCase0(UltimateGoalDetectionConditional goalDetection) {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (goalDetection.getDeliverWobble()) {
            if (goalDetection.getIsRed()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(70, -20), 4.712)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(backTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(38, -21, 3.14))
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .forward(10)
                        .build();
                dropSecondWobbleTaj = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(63, -20, 4.712))
                        .build();
                parkTraj = drive.trajectoryBuilder(dropSecondWobbleTaj.end())
                        .lineToLinearHeading(new Pose2d(72, -10, 4.712))
                        .build();
            } else {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(74, 23), -4.712)
                        .build();
                backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .back(10)
                        .build();
                secondWobbleTraj = drive.trajectoryBuilder(backTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(38, 35, -3.14))
                        .build();
                forwardTraj = drive.trajectoryBuilder(secondWobbleTraj.end(), true)
                        .forward(10)
                        .build();
                dropSecondWobbleTaj = drive.trajectoryBuilder(forwardTraj.end())
                        .lineToLinearHeading(new Pose2d(90, 18.5, -4.712))
                        .build();
                parkTraj = drive.trajectoryBuilder(dropSecondWobbleTaj.end())
                        .lineToLinearHeading(new Pose2d(72, 10, -4.712))
                        .build();
            }
        } else {
            if (goalDetection.getIsRed()) {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, 3), 6.07)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .splineTo(new Vector2d(92.67, -18.8), 3.644) // 74, -20 4.712
                            .build();
                    if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                                .lineToLinearHeading(new Pose2d(72, 12, 4.712))
                                .build();
                } else {
                    wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(52.11, 1.5, 5.5531))
                            .build();
                    /*shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(46.43, 3.25, -6.4226))
                            .build();

                     */
                    waitForOthersTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(33, -4, 0))
                            .build();
                    left = drive.trajectoryBuilder(waitForOthersTraj.end())
                            .lineToLinearHeading(new Pose2d(33, 15, 0))
                            .build();
                    if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(left.end())
                                .lineToLinearHeading(new Pose2d(70.17, 17.5, 0))
                                .build();
                }
            } else {
                if (goalDetection.getIsFirst()) {
                    shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                            .splineTo(new Vector2d(58, -3), 0.3)
                            .build();
                    wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .splineTo(new Vector2d(94, 25), -3.844) // 74, -20 4.712
                            .build();
                    if (goalDetection.getPark()) {
                        backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                                .lineToLinearHeading(new Pose2d(95, -12.5, 0))
                                .build();
                        parkTraj = drive.trajectoryBuilder(backTraj.end())
                                .lineToLinearHeading(new Pose2d(72, -9, 0))
                                .build();
                    }

                } else {
                    wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                            .lineToLinearHeading(new Pose2d(52.11, 7, -0.25))
                            .build();
                    /*shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                            .lineToLinearHeading(new Pose2d(46.43, 0, -0.16)) // y era -3.25
                            .build();*/
                    waitForOthersTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(36.5, 4.3, 0))
                            .build();
                    left = drive.trajectoryBuilder(waitForOthersTraj.end())
                            .lineToLinearHeading(new Pose2d(33, -15, 0))
                            .build();
                    if (goalDetection.getPark())
                        parkTraj = drive.trajectoryBuilder(left.end())
                                .lineToLinearHeading(new Pose2d(72, -11, 0))
                                .build();
                }
            }
        }
    }


    public void runCase() throws InterruptedException {
        robot.init(goalDetection.hardwareMap);
        goalDetection.sleep(goalDetection.getStartDelay());
        goalDetection.telemetry.update();
        if (goalDetection.getDeliverWobble()) {
            robot.toggleFlyWheel(true, 3215);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            drive.followTrajectory(backTraj);
            robot.dropArm(780);
            drive.followTrajectory(secondWobbleTraj);
            drive.followTrajectory(forwardTraj);
            goalDetection.sleep(800);
            robot.grabWobble();
            drive.followTrajectory(dropSecondWobbleTaj);
            robot.dropArm(670);
            robot.dropWobble();
            drive.followTrajectory(parkTraj);
            robot.wobbleServo.setPosition(0.45);
            goalDetection.sleep(400);
            robot.dropArm(20);
        } else {
            if (goalDetection.getIsFirst()) {
                robot.toggleFlyWheel(true, 3200);
                drive.followTrajectory(shootingPositionTraj);
                robot.shootrings(3);
                robot.toggleFlyWheel(false);
                drive.followTrajectory(wobbleTraj);
                robot.dropArm(670);
                goalDetection.sleep(1000);
                robot.dropWobble();
                robot.wobbleServo.setPosition(0.45);
                goalDetection.sleep(400);
                robot.dropArm(20);
                if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            } else {
                drive.followTrajectory(wobbleTraj);
                robot.dropArm(670);
                goalDetection.sleep(1000);
                robot.dropWobble();
                robot.wobbleServo.setPosition(0.45);
                robot.dropArm(20);
                /*robot.toggleFlyWheel(true, 2950);
                drive.followTrajectory(shootingPositionTraj);
                goalDetection.telemetry.addData("U", drive.getPoseEstimate().getHeading());
                goalDetection.telemetry.update();
                robot.shootrings(3, 1000);
                robot.toggleFlyWheel(false);

                 */
                robot.toggleFlyWheel(true, 3100);
                drive.followTrajectory(waitForOthersTraj);
                goalDetection.sleep(2000);
                drive.followTrajectory(left);
                robot.shootrings(3, 1000);
                robot.toggleFlyWheel(false);
                //while (goalDetection.runtime.seconds() <= 25) ;
                if (goalDetection.getPark())
                    drive.followTrajectory(parkTraj);
            }
        }
    }

}
