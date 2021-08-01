package org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert.Cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.EverythingForAutonomous.RobotDefinition_ForAuto;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert.ConditionalLinear;
import org.firstinspires.ftc.teamcode.EverythingForAutonomous.conditional.UltimateGoalDetectionConditional;
import org.firstinspires.ftc.teamcode.FromRoadRunner.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.Utils.Constants.*;

public class ConditionalCase1_Robert
{

    ConditionalLinear goalDetection;
    SampleMecanumDrive drive;
    Trajectory shootingPositionTraj, wobbleTraj, parkTraj, stackTraj, shootingPositionTraj2, parkTraj1, parkTraj2;
    RobotDefinition_ForAuto robot;

    public ConditionalCase1_Robert(ConditionalLinear goalDetection)
    {

        this.goalDetection = goalDetection;
        drive = goalDetection.getDrive();
        robot = new RobotDefinition_ForAuto();

        if (side == Side.RED)
        {
            if (line == Line.LEFT)
            {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, 3), 6.02)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(85.85, -2.508), 5.3733)
                        .build();
                if (stack == Stack.COLLECT_STACK)
                {
                    stackTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                            .splineTo(new Vector2d(38.119, -14), 0.275 + 3.14)
                            .build();
                    shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                            .splineTo(new Vector2d(55, -18), 0.01)
                            .build();
                }
                if (park == Park.PARK && stack == Stack.COLLECT_STACK)
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                            .lineToLinearHeading(new Pose2d(74.23, -13.25, 0))
                            .build();
                else if (park == Park.PARK)
                    parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(74.23, -13.25, 0))
                            .build();
            }
            else
            {
                wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(80.80, 2.45), 0.544)
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .lineToLinearHeading(new Pose2d(56.5, -5., 0.27))
                        .build();
                if (park == Park.PARK)
                {
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineTo(new Vector2d(73, 6.5))
                            .build();
                }
            }

        }
        else
        {
            if (line == Line.LEFT)
            {
                shootingPositionTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(58, -3), -5.92)
                        .build();
                wobbleTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                        .splineTo(new Vector2d(85.85, 2.508), -5.3733)
                        .build();
                if (stack == Stack.COLLECT_STACK)
                {
                    stackTraj = drive.trajectoryBuilder(wobbleTraj.end(), true)
                            .splineTo(new Vector2d(38.119, 14), -0.275 - 3.14)
                            .build();
                    shootingPositionTraj2 = drive.trajectoryBuilder(stackTraj.end(), false)
                            .splineTo(new Vector2d(55, 18), -0.01)
                            .build();
                }
                if (park == Park.PARK && stack == Stack.COLLECT_STACK)
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj2.end())
                            .lineToLinearHeading(new Pose2d(74.23, 13.25, 0))
                            .build();
                else if (park == Park.PARK)
                    parkTraj = drive.trajectoryBuilder(wobbleTraj.end())
                            .lineToLinearHeading(new Pose2d(74.23, 13.25, 0))
                            .build();
            }
            else
            {
                wobbleTraj = drive.trajectoryBuilder(new Pose2d())
                        .splineTo(new Vector2d(80.80, -9.45), -0.544)
                        .build();
                shootingPositionTraj = drive.trajectoryBuilder(wobbleTraj.end())
                        .lineToLinearHeading(new Pose2d(56.5, 5., -0.26))
                        .build();
                if (park == Park.PARK)
                {
                    parkTraj = drive.trajectoryBuilder(shootingPositionTraj.end())
                            .lineTo(new Vector2d(73, 6.5))
                            .build();
                }
            }
        }
    }

    public void runCase() throws InterruptedException
    {
        robot.init(goalDetection.hardwareMap);
        goalDetection.sleep(startDelay);
        if (line == Line.LEFT)
        {
            robot.toggleFlyWheel(true, 2970);
            drive.followTrajectory(shootingPositionTraj);
            robot.shootrings(3);
            robot.toggleFlyWheel(false);
            drive.followTrajectory(wobbleTraj);
            robot.dropArm(670);
            goalDetection.sleep(1000);
            robot.dropWobble();
            if (stack == Stack.COLLECT_STACK)
            {
                robot.toggleFlyWheel(true, 3000);
                robot.toggleIntake();
                drive.followTrajectory(stackTraj);
                drive.followTrajectory(shootingPositionTraj2);
                robot.wobbleServo.setPosition(0.45);
                goalDetection.sleep(400);
                robot.dropArm(20);
                goalDetection.sleep(500);
                robot.shootrings(1);
                goalDetection.sleep(500);
                robot.toggleFlyWheel(false);
            }
            else
            {
                robot.wobbleServo.setPosition(0.45);
                goalDetection.sleep(400);
                robot.dropArm(20);
            }
            if (park == Park.PARK) drive.followTrajectory(parkTraj);
        }
        else
        {
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
            robot.toggleFlyWheel(false);
            if (park == Park.PARK) drive.followTrajectory(parkTraj);
        }
    }

}
