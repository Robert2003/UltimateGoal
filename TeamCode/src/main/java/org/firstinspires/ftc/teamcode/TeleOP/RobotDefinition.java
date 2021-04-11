package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class RobotDefinition
{
    public DcMotor leftFront = null, leftRear = null, rightFront = null, rightRear = null;
    public DcMotor intake1   = null, intake2 = null;
    public DcMotor wobbleArm = null;
    public DcMotorEx flyWheel  = null;

    public Servo servo, wobbleServo, intakeServo;

    public double rearLeftPower;
    public double rearRightPower;
    public double frontLeftPower;
    public double frontRightPower;

    public double drive;
    public double strafe;
    public double rotate;

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.78);

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    public static int TOWER_RPM = 3600;
    public static int POWERSHOTS_RPM = 2890;

    double max;

    boolean supress2 = false, buttonPressed = false;

    HardwareMap hardwareMap = null;
    private ElapsedTime period = new ElapsedTime();

    public RobotDefinition(){}

    public void init(HardwareMap ahwMap)
    {
        hardwareMap = ahwMap;

        leftFront  = hardwareMap.get(DcMotor.class, "fl_motor");
        rightFront = hardwareMap.get(DcMotor.class, "fr_motor");
        leftRear   = hardwareMap.get(DcMotor.class, "bl_motor");
        rightRear  = hardwareMap.get(DcMotor.class, "br_motor");

        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

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
        drive = gamepad1.right_stick_y;
        strafe = -gamepad1.right_stick_x;
        rotate = -gamepad1.right_trigger + gamepad1.left_trigger;

        rearLeftPower = -strafe + drive + rotate;
        frontLeftPower = strafe + drive + rotate;
        rearRightPower = strafe + drive - rotate;
        frontRightPower = -strafe + drive - rotate;

        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        if(Math.abs(rearLeftPower)>1 || Math.abs(frontLeftPower)>1 || Math.abs(rearRightPower)>1 || Math.abs(frontRightPower)>1)
        {
            max = Math.max(Math.abs(rearLeftPower), Math.abs(frontLeftPower));
            max = Math.max(Math.abs(rearRightPower), max);
            max = Math.max(Math.abs(frontRightPower), max);

            rearLeftPower /= max;
            frontLeftPower /= max;
            rearRightPower /= max;
            frontRightPower /= max;
        }

        if (gamepad1.right_bumper) {
            rearLeftPower *= 0.3;
            frontLeftPower *= 0.3;
            rearRightPower *= 0.3;
            frontRightPower *= 0.3;
        }


        if(gamepad1.left_bumper)
        {
            if(!buttonPressed)
            {
                supress2 = !supress2;
                buttonPressed = true;
            }
        }
        else
            buttonPressed = false;

        if(supress2)
        {
            rearLeftPower *= 0.6;
            frontLeftPower *= 0.6;
            rearRightPower *= 0.6;
            frontRightPower *= 0.6;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftRear.setPower(rearLeftPower);
        rightRear.setPower(rearRightPower);

        if(gamepad1.dpad_up)
            intakeServo.setPosition(0.9);
        if(gamepad1.dpad_down)
            intakeServo.setPosition(0);
    }

    public void Gamepad2Actions(Gamepad gamepad2)
    {
        /**WobbleArm*/
        if(gamepad2.left_stick_button)
        {
            wobbleArm.setTargetPosition(-640);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(0.3);
        }
        else if(gamepad2.right_stick_button)
        {
            wobbleArm.setTargetPosition(-300);
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(0.6);
        }
        else if(Math.abs(wobbleArm.getTargetPosition()-wobbleArm.getCurrentPosition()) < 15)
        {
            wobbleArm.setTargetPosition(wobbleArm.getCurrentPosition());
            wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobbleArm.setPower(1);
        }

        if(gamepad2.right_trigger != 0 || gamepad2.left_trigger!=0)
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
            flyWheel.setVelocity(rpmToTicksPerSecond(TOWER_RPM));
        else if(gamepad2.right_bumper)
            flyWheel.setVelocity(rpmToTicksPerSecond(POWERSHOTS_RPM));
        else
            flyWheel.setVelocity(0);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

 }

