package org.firstinspires.ftc.teamcode.EverythingForTeleOP.secondrobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import static java.lang.Thread.sleep;

public class RobotDefinition_SecondRobot {

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 7, 13.7);

    public static double MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_GEAR_RATIO = 1;

    public static int GOAL_RPM = 3000; //era 3200, 3100
    public static int POWERSHOTS_RPM = 2750; // era 2800
    public static int INTERMEDIATE_RPM = 2940;

    boolean trigger = false;


    HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
    }

    public void Gamepad1Actions(Gamepad gamepad1) {
    }

    public void Gamepad2Actions(Gamepad gamepad2, double x, double y) {
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

}

