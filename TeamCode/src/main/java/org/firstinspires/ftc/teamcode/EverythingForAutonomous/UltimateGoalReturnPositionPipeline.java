package org.firstinspires.ftc.teamcode.EverythingForAutonomous;

import com.acmerobotics.dashboard.config.Config;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class UltimateGoalReturnPositionPipeline extends OpenCvPipeline {
    public static int BOTTOMLEFTX = 505;
    public static int BOTTOMLEFTY = 238;//bot left coordinates of small 1 ring box
    public static int WIDTH = 105;//width of 1 ring box
    public static int HEIGHT1 = 49;//height of 1 ring
    public static int BUFFER = 20;//12
    public static int HEIGHT2 = 56;//height of 4 rings
    public double HThresholdLow = 5;
    public double HThresholdHigh = 18;
    public int stack;

    //Pentru telemetry
    double treshold1;
    double treshold2;

    @Override
    public Mat processFrame(Mat input) {
        Mat HSVmat = new Mat();
        Imgproc.rectangle(input, new Point(BOTTOMLEFTX,BOTTOMLEFTY), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1), new Scalar(255,255,255),1);
        Imgproc.rectangle(input, new Point(BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1+BUFFER), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2), new Scalar(255,255,255),1);
        Imgproc.cvtColor(input,HSVmat,Imgproc.COLOR_RGB2HSV);
        double[] topBox = Core.mean(HSVmat.submat(new Rect(new Point(BOTTOMLEFTX,BOTTOMLEFTY), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1)))).val;
        treshold1 = topBox[0];
        //getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT1);
        double[] bottomBox = Core.mean(HSVmat.submat(new Rect(new Point(BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1+BUFFER), new Point(BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2)))).val;
        treshold2 = bottomBox[0];
        //getAverageYCrCb(yCrCbmat, BOTTOMLEFTX,BOTTOMLEFTY+HEIGHT1,BOTTOMLEFTX+WIDTH,BOTTOMLEFTY+HEIGHT2);
        Imgproc.putText(input,"TopBoxH: "+topBox[0],new Point(0,100),1,1,new Scalar(0,0,0));
        Imgproc.putText(input,"BottomBoxH: "+bottomBox[0],new Point(0,200),1,1,new Scalar(0,0,0));
        if(isAboveThresholds(topBox[0])){
            stack = 4;
        }
        else if(isAboveThresholds(bottomBox[0])){
            stack = 1;
        }
        else{
            stack = 0;
        }
        return input;
    }

    public boolean isAboveThresholds(double avgH){
        return avgH > HThresholdLow && avgH < HThresholdHigh;
    }

    //Pentru telemetry din HighGoal

    public double getTreshold1() {
        return treshold1;
    }

    public double getTreshold2() {
        return treshold2;
    }

}