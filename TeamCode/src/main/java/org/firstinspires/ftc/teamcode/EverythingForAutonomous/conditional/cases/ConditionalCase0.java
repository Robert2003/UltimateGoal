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
    Trajectory shootingPositionTraj, wobbleTraj, parkTraj;
    RobotDefinition_ForAuto robot;

    public ConditionalCase0(UltimateGoalDetectionConditional goalDetection){

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if(goalDetection.getIsRed() && goalDetection.getIsFirst()) {
            shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(58, 3), 6.02)
                    .build();
            wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                    .splineTo(new Vector2d(74, -20), 4.712)
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
            robot.toggleFlyWheel(true,3200);
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
