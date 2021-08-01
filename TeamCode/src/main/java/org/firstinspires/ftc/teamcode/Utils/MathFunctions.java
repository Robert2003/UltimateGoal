package org.firstinspires.ftc.teamcode.Utils;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MathFunctions
{
    public static Pair<Double, Double> RobotPosition(Vector2d RobotPosition, Vector2d TowerPosition)
    {
        /*double X = abs(RobotPosition.getX()-TowerPosition.getX());
        double Y = abs(RobotPosition.getY()-TowerPosition.getY());

        double distance = sqrt(X*X+Y*Y);
         */

        //Vector2d difference = RobotPosition.minus(TowerPosition);
        double distance = RobotPosition.distTo(TowerPosition);

        double angle = RobotPosition.angleBetween(TowerPosition);
        Pair<Double, Double> Position = new Pair<>(distance, angle);

        return Position;
    }
}
