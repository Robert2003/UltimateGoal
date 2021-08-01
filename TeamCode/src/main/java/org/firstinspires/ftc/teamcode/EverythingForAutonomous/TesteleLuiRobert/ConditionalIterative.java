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

package org.firstinspires.ftc.teamcode.EverythingForAutonomous.TesteleLuiRobert;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
@Disabled
public class ConditionalIterative extends OpMode {

    public enum Side
    {
        BLUE,
        RED
    }
    public enum Line
    {
        RIGHT,
        LEFT
    }
    public enum Stack
    {
        COLLECT_STACK,
        DONT_COLLECT_STACK
    }

    Side side = Side.BLUE;
    Line line = Line.LEFT;
    Stack starterStack = Stack.COLLECT_STACK;

    boolean waitingAnswer = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        ask_questions();
        showcaseAnswers();
    }

    @Override
    public void init_loop()
    {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
    }

    void ask_questions()
    {
        telemetry.addData("Q1", "Is the robot on the red side? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                side = Side.RED;
                waitingAnswer = false;
            } else if (gamepad1.x) {
                side = Side.BLUE;
                waitingAnswer = false;
            }
        }


        telemetry.addData("Q2", "Is the robot on the left or the right line? \n" +
                "A - Left\nX - Right");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer ) {
            if (gamepad1.a) {
                waitingAnswer = false;
                line = Line.LEFT;
            } else if (gamepad1.x) {
                waitingAnswer = false;
                line = Line.RIGHT;
            }
        }


        telemetry.addData("Q4", "Should the robot collect the stack? \n" +
                "A - Yes\nX - No");
        telemetry.update();

        waitingAnswer = true;
        while (waitingAnswer) {
            if (gamepad1.a) {
                starterStack = Stack.COLLECT_STACK;
                waitingAnswer = false;
            } else if (gamepad1.x) {
                starterStack = Stack.DONT_COLLECT_STACK;
                waitingAnswer = false;
            }
        }
    }

    void showcaseAnswers()
    {
        telemetry.addData("A1", "Blue or red side:" + (side == Side.RED ? "RED" : "BLUE"));
        telemetry.addData("A2", "Left or right line:" + (line == Line.LEFT ? "LEFT" : "RIGHT"));
        telemetry.addData("A3", "Collect the starter stack:" + (starterStack == Stack.COLLECT_STACK ? "YES" : "NO"));
        telemetry.update();
    }

}
