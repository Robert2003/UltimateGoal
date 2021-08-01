package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Constants
{
    public static Vector2d RedTowerPose = new Vector2d(120, 120);
    public static Vector2d BlueTowerPose = new Vector2d(120, 120);

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

    public enum Park
    {
        PARK,
        NOT_PARK
    }

    public static Side side = Side.BLUE;
    public static Line line = Line.LEFT;
    public static Stack stack = Stack.COLLECT_STACK;
    public static Park park = Park.PARK;
    public static int startDelay = 0;
    public static int secondsToPark = 0;
}
