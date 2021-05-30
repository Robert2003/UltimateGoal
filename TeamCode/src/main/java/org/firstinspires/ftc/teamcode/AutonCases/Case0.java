package org.firstinspires.ftc.teamcode.AutonCases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOP.RobotDefinition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(name="Case0", group="Cases")
public class Case0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RobotDefinition robot = new RobotDefinition();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, false);
        robot.init(hardwareMap);

        Trajectory shootingPosition = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(58, 3), 6.127)
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
        Trajectory dropSecondWobble = drive.trajectoryBuilder(grabSecondWobble.end())
                .lineToLinearHeading(new Pose2d(63, -20, 4.712))
                .build();
        Trajectory park = drive.trajectoryBuilder(dropSecondWobble.end())
                .lineToLinearHeading(new Pose2d(72, -10, 4.712))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.toggleFlyWheel();
        drive.followTrajectory(shootingPosition);
        robot.shoot3rings();
        robot.toggleFlyWheel();
        drive.followTrajectory(firstWobble);
        robot.dropArm();
        robot.dropWobble();
        drive.followTrajectory(back);
        robot.dropArm(true);
        drive.followTrajectory(grabSecondWobble);
        robot.grabWobble();
        drive.followTrajectory(dropSecondWobble);
        robot.dropArm();
        robot.dropWobble();
        drive.followTrajectory(park);

        telemetry.addData("heading", drive.getPoseEstimate().getHeading());
        telemetry.update();
        while(opModeIsActive());
    }
}
