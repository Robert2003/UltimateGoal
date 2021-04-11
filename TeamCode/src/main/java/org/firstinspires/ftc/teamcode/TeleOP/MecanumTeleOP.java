/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@Config
@TeleOp(name="MecanumTeleOP", group="Iterative Opmode")
//@Disabled
public class MecanumTeleOP extends OpMode
{
    private RobotDefinition robot = new RobotDefinition();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init()
    {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start()
    {
        runtime.reset();
    }

    @Override
    public void loop()
    {
        telemetry.addData("flyWheel velocity", robot.flyWheel.getVelocity());
        telemetry.addData("position", robot.wobbleArm.getCurrentPosition());
        telemetry.addData("p1", robot.leftFront.getCurrentPosition());
        telemetry.addData("p2", robot.rightFront.getCurrentPosition());
        telemetry.addData("p3", robot.rightRear.getCurrentPosition());
        telemetry.addData("p4", robot.leftRear.getCurrentPosition());
        telemetry.update();

        if(gamepad2.left_bumper)
        {
            if(runtime.seconds() > 1)
            {
                runtime.reset();
                robot.servo.setPosition(0);
            }
            else if(runtime.seconds() > 0.5)
                robot.servo.setPosition(0.55);
        }
        else
            robot.servo.setPosition(0.55);

        robot.Gamepad1Actions(gamepad1);
        robot.Gamepad2Actions(gamepad2);
    }

    @Override
    public void stop()
    {

    }

    double speed(double X, double Y)
    {
        double flyWheelRadius=0.05;
        double alfa = 37.0;
        double deltaY = Y;
        double h = 0.15;
        double numarator = -4.9*X*X;
        double numitor = (Y-h-tan(toRadians(alfa))*X)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numarator/numitor;
        return 2*sqrt(cat)/flyWheelRadius*60/(2*Math.PI);
    }
}
