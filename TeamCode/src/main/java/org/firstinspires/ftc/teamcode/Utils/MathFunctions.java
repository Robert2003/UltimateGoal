package org.firstinspires.ftc.teamcode.Utils;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import static java.lang.Math.*;

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

        double angle1 = RobotPosition.minus(TowerPosition).angle();
        double angle2 = TowerPosition.minus(RobotPosition).angle();
        double angle3 = atan2(TowerPosition.getY()-RobotPosition.getY(), TowerPosition.getX()-RobotPosition.getX());
        Pair<Double, Double> Position = new Pair<>(angle1, angle2);

        return Position;
    }

    public double speed(double X, double Y)
    {
        double deltaY = abs(132-X)*2.54;
        double deltaX = abs(-15-Y)*2.54;
        double hypot = sqrt(deltaX*deltaX+deltaY*deltaY)/100;
        double flyWheelRadius = 0.05;
        double alfa = 37.0;
        double h = 0.165;
        double numarator = -4.9*hypot*hypot;
        double numitor = (0.93-h-tan(toRadians(alfa))*hypot)*cos(toRadians(alfa))*cos(toRadians(alfa));
        double cat = numarator/numitor;
        return 3*sqrt(cat)/flyWheelRadius*60/(2*PI);
    }
}
