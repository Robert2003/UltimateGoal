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
@Autonomous(name="Case0", group="Cases")
@Disabled
public class Case0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RobotDefinition_ForAuto robot = new RobotDefinition_ForAuto();
        robot.init(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);


        Trajectory shootingPosition = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.02)
                .build();
        Trajectory firstWobble = drive.trajectoryBuilder(shootingPosition.end())
                .splineTo(new Vector2d(74, -20), 4.712)
                .build();
        Trajectory back = drive.trajectoryBuilder(firstWobble.end())
                .back(10)
                .build();
        Trajectory grabSecondWobble = drive.trajectoryBuilder(back.end(), true)
                .lineToLinearHeading(new Pose2d(30, -20, 3.14))
                .build();
        Trajectory forward = drive.trajectoryBuilder(grabSecondWobble.end(), true)
                .forward(2)
                .build();
        Trajectory dropSecondWobble = drive.trajectoryBuilder(forward.end())
                .lineToLinearHeading(new Pose2d(63, -20, 4.712))
                .build();
        Trajectory park = drive.trajectoryBuilder(dropSecondWobble.end())
                .lineToLinearHeading(new Pose2d(72, -10, 4.712))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.toggleFlyWheel(true, 2970);
        drive.followTrajectory(shootingPosition);
        robot.shootrings(3);
        robot.toggleFlyWheel(false);
        drive.followTrajectory(firstWobble);
        robot.dropArm(670);
        sleep(1000);
        robot.dropWobble();
        drive.followTrajectory(back);
        robot.dropArm(780);
        drive.followTrajectory(grabSecondWobble);
        drive.followTrajectory(forward);
        sleep(800);
        robot.grabWobble();
        drive.followTrajectory(dropSecondWobble);
        robot.dropArm(670);
        robot.dropWobble();
        drive.followTrajectory(park);
        robot.wobbleServo.setPosition(0.45);
        sleep(400);
        robot.dropArm(20);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
