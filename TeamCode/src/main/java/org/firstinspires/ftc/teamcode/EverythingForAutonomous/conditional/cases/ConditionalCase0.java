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
    Trajectory shootingPositionTraj, wobbleTraj, parkTraj, waitForOthersTraj, backTraj;
    RobotDefinition_ForAuto robot;

    public ConditionalCase0(UltimateGoalDetectionConditional goalDetection) {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (goalDetection.getIsRed()) {
            if (goalDetection.getIsFirst()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(92.67, -18.8), 3.644) // 74, -20 4.712
                        .build();
                if (goalDetection.getPark())
                    parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(72, 10, 4.712))
                            .build();
            } else {
                wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(52.11, 1.5, 5.5531))
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(46.43, 3.25, 0))
                        .addTemporalMarker(0.1, () -> {
                            robot.wobbleServo.setPosition(0.45);
                            robot.dropArm(20);
                        })
                        .build();
                waitForOthersTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .lineToLinearHeading(new Pose2d(36.5, -4.3, 0))
                        .build();
                if (goalDetection.getPark())
                    parkTraj = drive.trajectoryBuilder(waitForOthersTraj.end())
                            .lineToLinearHeading(new Pose2d(70.17, 17.5, 0))
                            .build();
            }
        } else {
            if (goalDetection.getIsFirst()) {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -5.92)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(92.67, 23.8), -3.844) // 74, -20 4.712
                        .build();
                if (goalDetection.getPark()) {
                    backTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(95, -12.5, 0))
                            .build();
                    parkTraj = drive.trajectoryBuilder(backTraj.end())
                            .lineToLinearHeading(new Pose2d(72, -12.5, 0))
                            .build();
                }

            } else {
                wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                        .lineToLinearHeading(new Pose2d(52.11, 7, -0.25))
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                        .lineToLinearHeading(new Pose2d(46.43, 0, -0.16)) // y era -3.25
                        .addTemporalMarker(0.1, () -> {
                            robot.wobbleServo.setPosition(0.45);
                            robot.dropArm(20);
                        })
                        .build();
                waitForOthersTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .lineToLinearHeading(new Pose2d(36.5, 4.3, 0))
                        .build();
                if (goalDetection.getPark())
                    parkTraj = drive.trajectoryBuilder(waitForOthersTraj.end())
                            .lineToLinearHeading(new Pose2d(70.17, -17.5, 0))
                            .build();
            }
        }
    }

    public void runCase() throws InterruptedException {
        robot.init(goalDetection.hardwareMap);
        goalDetection.sleep(goalDetection.getStartDelay());
        goalDetection.telemetry.update();
        if (goalDetection.getIsFirst()) {
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            if (goalDetection.getPark()) drive.followTrajectory(parkTraj);
            robot.wobbleServo.setPosition(0.45);
            goalDetection.sleep(400);
            robot.dropArm(20);
        } else {
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            goalDetection.telemetry.addData("U", drive.getPoseEstimate().getHeading());
            goalDetection.telemetry.update();
            robot.shootrings(3, 1000);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(waitForOthersTraj);
            while (goalDetection.runtime.seconds() <= 25) ;
            if (goalDetection.getPark()) {
                if(!goalDetection.getIsRed()) drive.followTrajectory(backTraj);
                drive.followTrajectory(parkTraj);
            }
        }
    }

}
