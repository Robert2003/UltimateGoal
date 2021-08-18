package org.firstinspires.ftc.teamcode.EverythingForTeleOP.adjusting;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.EverythingForTeleOP.RobotDefinition_ForTeleOP;

public class Adjusting_RobotDefinition extends RobotDefinition_ForTeleOP {

    int flyWheelRPM, addition = 5;
    boolean confirmChage = true;
    Adjusting_AugmentedDriving driving;

    public Adjusting_RobotDefinition(Adjusting_AugmentedDriving driving) {
        this.driving = driving;
        flyWheelRPM = GOAL_RPM;
    }

    @Override
    public void Gamepad2Actions(Gamepad gamepad2, double x, double y) {
        /*intake*/
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
        if (gamepad2.dpad_right && confirmChage) {
            flyWheelRPM += addition;
            confirmChage = false;
        }
        else if (gamepad2.dpad_left && confirmChage) {
            flyWheelRPM -= addition;
            confirmChage = false;
        }
        if(!confirmChage && gamepad2.b)
            confirmChage = true;
        /*FlyWheel*/
        if (gamepad2.right_bumper)
            flyWheel.setVelocity(rpmToTicksPerSecond(flyWheelRPM));
        else
            flyWheel.setVelocity(10);
        driving.telemetry.addData("Flywheel RPM", flyWheelRPM);
        driving.telemetry.addData("Confirmed", ((confirmChage = true) ? "Yes" : "No"));
        driving.telemetry.update();
    }

}
