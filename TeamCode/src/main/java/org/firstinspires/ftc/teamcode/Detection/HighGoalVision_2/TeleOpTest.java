package org.firstinspires.ftc.teamcode.Detection.HighGoalVision_2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;


@TeleOp(name="TeleOp Test", group="test")
@Config
public class TeleOpTest extends OpMode {

    public static double aimTimer = 1;

    //Subsystems
    SampleMecanumDriveCancelable drive;
    Camera camera;
    BlueGoalVisionPipeline pipeline;

    //Predeclared Trajectories
    Trajectory driveToPowershotPosition;


    ElapsedTime timer = new ElapsedTime();

    double launcherRPM;
    boolean launcherOn;
    int armPos;
    boolean buttonReleased1;
    boolean buttonReleased2;
    boolean triggerReleased;

    //initialization variables
    int rings = 0;
    int powerShotState = 1; // *** changed from 0 to 1 ***
    double[] powerShotAngles;
    double initialAngle;

    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;
    double strafePower = 1.0;
    double powershotHeadingOffset = 0;
    boolean exitToAutoAim = false;


    //target angle
    double angle = -6.5;

    enum Mode {
        DRIVER_CONTROL,
        LINE_TO_POINT,
        SHOOT_RINGS,
        STAFE_TO_POWERSHOT_POSITION,
        PREPARE_TO_SHOOT_POWERSHOTS,
        SHOOT_RINGS_POWERSHOT,
        ALIGN_TO_ANGLE,
        TURN_TO,
        WAIT_FOR_TURN_TO_FINISH,
        ALIGN_TO_GOAL;
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Init Camera
        pipeline = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, pipeline);

        // Initialization values
        launcherRPM = 3200;
        launcherOn = false;

        armPos = 0;
        buttonReleased1 = true;
        buttonReleased2 = true;
        triggerReleased = true;


        drive = new SampleMecanumDriveCancelable(hardwareMap);


        //create trajectories
       // driveToPowershotPosition = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0.0)))
                //.lineToConstantHeading(new Vector2d(-9.9, 15))
                //.build();

        //set read mode to manual
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void loop() {

        //Clear Bulk Cache at beginning of loop
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();


        // Retrieve pose

        //log telemetry data for drivers
        telemetry.addData("Goal Visibility", pipeline.isGoalVisible());
        telemetry.addData("RPM", launcherRPM);
        telemetry.addData("Drive Mode: ", currentMode);

        /*switch (currentMode){

            case DRIVER_CONTROL:

                break;

            case ALIGN_TO_ANGLE:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                if (!drive.isBusy() && !exitToAutoAim)  {
                    currentMode = Mode.DRIVER_CONTROL;
                }  else if (!drive.isBusy() && exitToAutoAim && pipeline.isGoalVisible()) {
                    exitToAutoAim = false;
                    currentMode = Mode.ALIGN_TO_GOAL;
                    timer.reset();
                }
                //switch to auto aim  after align to angle sequence ends
                if(gamepad1.right_bumper) {
                    exitToAutoAim = true;
                }
                break;

            case ALIGN_TO_GOAL:
                //if goal is centered for 1 second shoot rings, else reset timer
                if (timer.seconds() > aimTimer) {
                    if(pipeline.isGoalVisible()){
                        rings = 3;
                        timer.reset();
                        launcherOn = true;
                        currentMode = Mode.SHOOT_RINGS;
                    } else {
                        timer.reset();
                    }
                }

                //emergency exit
                if (gamepad1.left_bumper) {
                    rings = 0;
                    currentMode = Mode.DRIVER_CONTROL;
                }


                if(pipeline.isGoalVisible()) {
                    //returns positive if robot needs to turn counterclockwise

                }
                break;

            // generate a trajectory based on powershot state and move to stop and aim state
            case STAFE_TO_POWERSHOT_POSITION:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                    // 0 1 2
                } else if (!drive.isBusy()) {
                    launcherRPM -= 50;
                    currentMode = Mode.PREPARE_TO_SHOOT_POWERSHOTS;
                    powershotHeadingOffset = drive.getRawExternalHeading();
                    timer.reset();
                }
                break;

            //when robot has reached the end of it's generated trajectory, reset timer and rings to 1, then move to shoot rings state
            case PREPARE_TO_SHOOT_POWERSHOTS:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                else if (!drive.isBusy()) {
                    rings = 1;
                    timer.reset();
                    currentMode = Mode.SHOOT_RINGS_POWERSHOT;
                }

                break;



            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS_POWERSHOT:
                //emergency exit

                break;


            case TURN_TO:
                break;


            case WAIT_FOR_TURN_TO_FINISH:
                if(!drive.isBusy()) {
                    currentMode = Mode.PREPARE_TO_SHOOT_POWERSHOTS;
                }

                break;


            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS:


                break;
        }
        */


        //update robot
        drive.update();
        telemetry.update();



    }


}