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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
@Disabled
public class ConditionalLinear extends LinearOpMode
{

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
    int selectedAnswer = 1;
    int secondsToPark = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        ask_questions();
        showcaseAnswers();
        confirmAnswers();

        waitForStart();
    }

    void ask_questions()
    {
        telemetry.addData("Q1", "Pe ce parte e robotul? \n" +
                "A - ROSU\nX - ALBASTRU");
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

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        telemetry.addData("Q2", "Pe ce linie este robotul? \n" +
                "A - STANGA\nX - DREAPTA");
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

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        telemetry.addData("Q3", "Robotul colecteaza starter stack-ul? \n" +
                "A - DA\nX - NU");
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

        telemetry.addData("C", "Press Y to continue.");
        telemetry.update();
        while (!gamepad1.y && !isStopRequested()) ;


        boolean left_right_buttonToggled = false;
        while (!gamepad1.y && !isStopRequested()) {
            telemetry.addData("Q4", "In ultimele cate secunde se parcheaza? \n" + secondsToPark + " secunde");
            telemetry.addData("C", "Press Y to continue.");
            telemetry.update();

            if(gamepad1.dpad_left || gamepad1.dpad_right)
            {
                if(gamepad1.dpad_left && secondsToPark>0 && !left_right_buttonToggled)
                {
                    secondsToPark--;
                    left_right_buttonToggled = true;
                }
                if(gamepad1.dpad_right && secondsToPark<30 && !left_right_buttonToggled)
                {
                    secondsToPark++;
                    left_right_buttonToggled = true;
                }
            }
            else
                left_right_buttonToggled = false;
        }
    }

    void showcaseAnswers()
    {
        telemetry.addData("A1", "Partea albastra sau rosie: " + (side == Side.RED ? "ROSU" : "ALBASTRU") + (selectedAnswer == 1 ? " [X]" : ""));
        telemetry.addData("A2", "Linia din stanga sau din dreapta: " + (line == Line.LEFT ? "STANGA" : "DREAPTA") + (selectedAnswer == 2 ? " [X]" : ""));
        telemetry.addData("A3", "Colecteaza starter stack-ul: " + (starterStack == Stack.COLLECT_STACK ? "DA" : "NU") + (selectedAnswer == 3 ? " [X]" : ""));
        telemetry.addData("A3", "Robotul se parcheaza in ultimele " + secondsToPark + " secunde" + (selectedAnswer == 3 ? " [X]" : ""));
        telemetry.update();
    }

    void confirmAnswers()
    {
        boolean modifyingAnswers = true;
        boolean up_down_buttonToggled = false, left_right_buttonToggled = false;

        while(modifyingAnswers && !isStopRequested())
        {
            if(gamepad1.dpad_down || gamepad1.dpad_up)
            {
                if (gamepad1.dpad_down == true && !up_down_buttonToggled)
                {
                    selectedAnswer++;
                    up_down_buttonToggled = true;
                }
                else if (gamepad1.dpad_down == true && !up_down_buttonToggled)
                {
                    selectedAnswer--;
                    up_down_buttonToggled = true;
                }
            }
            else
                up_down_buttonToggled = false;

            if(selectedAnswer > 3)
                selectedAnswer = 1;
            if(selectedAnswer < 1)
                selectedAnswer = 3;


            if(gamepad1.dpad_left || gamepad1.dpad_right)
            {
                if((gamepad1.dpad_left || gamepad1.dpad_right) && !left_right_buttonToggled)
                {
                    left_right_buttonToggled  = true;

                    if(selectedAnswer == 1)
                    {
                        if(side == Side.RED)
                            side = Side.BLUE;
                        else
                            side = Side.RED;

                        showcaseAnswers();
                    }

                    if(selectedAnswer == 2)
                    {
                        if(line == Line.LEFT)
                            line = Line.RIGHT;
                        else
                            line = Line.LEFT;

                        showcaseAnswers();
                    }

                    if(selectedAnswer == 3)
                    {
                        if(starterStack == Stack.COLLECT_STACK)
                            starterStack = Stack.DONT_COLLECT_STACK;
                        else
                            starterStack = Stack.COLLECT_STACK;

                        showcaseAnswers();
                    }

                    if(selectedAnswer == 4)
                    {
                        if(gamepad1.dpad_left && secondsToPark>0)
                            secondsToPark--;
                        if(gamepad1.dpad_right && secondsToPark<30)
                            secondsToPark++;
                    }
                }
            }
            else
                left_right_buttonToggled = false;
        }
    }
}
