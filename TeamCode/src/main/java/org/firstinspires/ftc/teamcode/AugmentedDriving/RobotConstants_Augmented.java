package org.firstinspires.ftc.teamcode.AugmentedDriving;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

public class RobotConstants_Augmented
{
    public DcMotor intake1   = null, intake2 = null;
    public DcMotor wobbleArm = null;
    public DcMotorEx flyWheel  = null;

    public Servo servo, wobbleServo, intakeServo;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.78);

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    public static int TOWER_RPM = 3600;
    public static int POWERSHOTS_RPM = 3200;

    double max;

    boolean supress2 = false, buttonPressed = false, trigger = false;


    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

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
            intakeServo.setPosition(0.9);
        if(gamepad1.dpad_down)
            intakeServo.setPosition(0);
    }

    public void Gamepad2Actions(Gamepad gamepad2, double x, double y)
    {
        /**WobbleArm*/
        if(gamepad2.left_trigger!=0)
        {
            trigger = true;
            wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            wobbleArm.setPower(0.3);
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
                wobbleArm.setTargetPosition(-640);
                wobbleArm.setPower(0.3);
            }
            else if(gamepad2.right_stick_button)
            {
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setTargetPosition(-300);
                wobbleArm.setPower(0.6);
            }
            else if(Math.abs(wobbleArm.getTargetPosition()-wobbleArm.getCurrentPosition()) < 15)
            {
                wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
                wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                wobbleArm.setPower(1);
            }
        }

        /*if(gamepad2.right_trigger != 0 || gamepad2.left_trigger!=0)
        {
            wobbleArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad2.right_trigger != 0)
            wobbleArm.setPower(-0.25);
        else if(gamepad2.left_trigger != 0)
            wobbleArm.setPower(0.25);
        else if(!wobbleArm.isBusy())
        {
            wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(1);
        }
*/
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
            wobbleServo.setPosition(1);
        if(gamepad2.dpad_right)
            wobbleServo.setPosition(0);
        if(gamepad2.dpad_down)
            wobbleServo.setPosition(0.5);

        /**FlyWheel*/
        if(gamepad2.right_bumper && !gamepad2.dpad_up)
            flyWheel.setVelocity(rpmToTicksPerSecond(POWERSHOTS_RPM));
        else if(gamepad2.right_bumper)
            flyWheel.setVelocity(rpmToTicksPerSecond(POWERSHOTS_RPM));
        else
            flyWheel.setVelocity(0);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    public void setRPM(DcMotor motoras)
    {
        MotorConfigurationType motorConfigurationType = motoras.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motoras.setMotorType(motorConfigurationType);
    }

    public void setRPM(DcMotorEx motoras)
    {
        MotorConfigurationType motorConfigurationType = motoras.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motoras.setMotorType(motorConfigurationType);
    }

    double speed(double X, double Y)
    {
        double deltaY = Math.abs(-131-X)*2.54;
        double deltaX = Math.abs(14.56-Y)*2.54;
        double hypot = sqrt(deltaX*deltaX+deltaY*deltaY)/100;
        double flyWheelRadius = 0.05;
        double alfa = 37.0;
        double h = 0.165;
        double numarator = -4.9*hypot*hypot;
        double numitor = (0.93-h-tan(toRadians(alfa))*hypot)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numarator/numitor;
        return 3*sqrt(cat)/flyWheelRadius*60/(2*Math.PI);
    }
}

