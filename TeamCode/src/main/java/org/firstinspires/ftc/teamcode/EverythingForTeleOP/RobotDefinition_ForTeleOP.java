package org.firstinspires.ftc.teamcode.EverythingForTeleOP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static java.lang.Thread.sleep;

public class RobotDefinition_ForTeleOP
{
    public DcMotor intake1   = null, intake2 = null;
    public DcMotor wobbleArm = null;
    public DcMotorEx flyWheel  = null;

    public Servo servo, wobbleServo, intakeServo;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.7);

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    public static int GOAL_RPM = 3200;
    public static int POWERSHOTS_RPM = 2840;

    boolean  trigger = false;


    HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap)
    {
        hardwareMap = ahwMap;

        intake1   = hardwareMap.get(DcMotor.class, "intake1");
        intake2   = hardwareMap.get(DcMotor.class, "intake2");
        flyWheel  = hardwareMap.get(DcMotorEx.class, "flyWheel");
        wobbleArm = hardwareMap.get(DcMotor.class, "wobbleArm");

        intake2.setDirection(DcMotor.Direction.REVERSE);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servo       = hardwareMap.get(Servo.class, "servo");
        wobbleServo = hardwareMap.get(Servo.class, "wobbleServo");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        servo.setPosition(0.55);
        intakeServo.setPosition(0.9);

        MotorConfigurationType motorConfigurationType = flyWheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flyWheel.setMotorType(motorConfigurationType);

        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));
    }

    public void Gamepad1Actions(Gamepad gamepad1)
    {
        if(gamepad1.dpad_up)
            intakeServo.setPosition(1);
        if(gamepad1.dpad_down)
            intakeServo.setPosition(0.42);
    }

    public void Gamepad2Actions(Gamepad gamepad2, double x, double y)
    {
        /**WobbleArm*/
        if(gamepad2.left_trigger!=0)
        {
            trigger = true;
            wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleArm.setPower(-0.5);
        }
        else if(trigger)
        {
            trigger = false;
            wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(0);
        }

        if(!trigger)
        {
            if(gamepad2.left_stick_button)
            {
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setTargetPosition(799);
                wobbleArm.setPower(0.5);
            }
            else if(gamepad2.right_stick_button)
            {
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setTargetPosition(480);
                wobbleArm.setPower(0.7);
            }
            else if(Math.abs(wobbleArm.getTargetPosition()-wobbleArm.getCurrentPosition()) < 15)
            {
                wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(1);
            }
        }

        /**intake*/
        if (gamepad2.y) {
            intake1.setPower(1);
            intake2.setPower(1);
        } else if (gamepad2.a) {
            intake1.setPower(-1);
            intake2.setPower(-1);
        } else if (gamepad2.x) {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        /**WobbleServo*/
        if(gamepad2.dpad_left)
            wobbleServo.setPosition(0.53);
        if(gamepad2.dpad_right)
            wobbleServo.setPosition(0);
        if(gamepad2.dpad_down)
            wobbleServo.setPosition(0.3);

        /**FlyWheel*/
        if(gamepad2.right_bumper && !gamepad2.dpad_up)
            flyWheel.setVelocity(rpmToTicksPerSecond(GOAL_RPM));
        else if(gamepad2.right_bumper)
            flyWheel.setVelocity(rpmToTicksPerSecond(POWERSHOTS_RPM));
        else
            flyWheel.setVelocity(10);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public void toggleFlyWheel(boolean shouldTurnOn)
    {
        if(shouldTurnOn==true)
            flyWheel.setVelocity(rpmToTicksPerSecond(POWERSHOTS_RPM));
        else
            flyWheel.setVelocity(0);
    }

    public void shootrings(int timesToShoot) throws InterruptedException {
        while(Math.abs(flyWheel.getVelocity()-rpmToTicksPerSecond(POWERSHOTS_RPM)) > 20);
        for(int i = 1; i <= timesToShoot; i++) {
            servo.setPosition(0);
            sleep(400);
            servo.setPosition(0.55);
            sleep(400);
        }
    }
}

