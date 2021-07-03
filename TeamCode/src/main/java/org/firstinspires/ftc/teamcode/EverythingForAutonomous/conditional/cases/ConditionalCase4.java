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
    Trajectory shootingPositionTraj, wobbleTraj, parkTraj;
    RobotDefinition_ForAuto robot;

    public ConditionalCase4(UltimateGoalDetectionConditional goalDetection){

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if(goalDetection.getIsRed() && goalDetection.getIsFirst()) {
            /*Trajectory back = drive.trajectoryBuilder(firstWobble.end())
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
            Trajectory park = drive.trajectoryBuilder(dropSecondWobble.end())
                    .lineTo(new Vector2d(80, -29))
                    .addTemporalMarker(0.1, () -> {
                        robot.wobbleServo.setPosition(0.45);
                        robot.dropArm(20);
                    })
                    .build();


             */
            shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(58, 3), 6.02)
                    .build();
            wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                    .splineTo(new Vector2d(108.6, -28.7), 5.43)
                    .addTemporalMarker(0.1, () -> {
                        robot.dropArm(700);
                    })
                    .build();
            if(goalDetection.getPark())
                parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                    .lineToLinearHeading(new Pose2d(72, -10, 4.712))
                    .build();
        }
        else if(goalDetection.getIsRed() && !goalDetection.getIsFirst()){
            wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d(52.11, -6.5))
                    .build();
            shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                    .lineTo(new Vector2d(0, 10.6))
                    .build();
            if(goalDetection.getPark())
                parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                    .lineTo(new Vector2d(68, 60))
                    .addTemporalMarker(0.1, () -> {
                        robot.wobbleServo.setPosition(0.45);
                        robot.dropArm(20);
                    })
                    .build();

        }
    }

    public void runCase() throws InterruptedException {
        robot.init(goalDetection.hardwareMap);
        if(goalDetection.getIsRed() && goalDetection.getIsFirst()){
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            if(goalDetection.getPark()) drive.followTrajectory(parkTraj);
            robot.wobbleServo.setPosition(0.45);
            goalDetection.sleep(400);
            robot.dropArm(20);
        }
        else if(goalDetection.getIsRed() && !goalDetection.getIsFirst()){
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            robot.toggleFlyWheel(true,3100);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3,1000);
            robot.toggleFlyWheel(false);
            if(goalDetection.getPark()) drive.followTrajectory(parkTraj);
            else {
                robot.wobbleServo.setPosition(0.45);
                robot.dropArm(20);
            }
        }
    }

}
