package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.*;
import java.util.ArrayList;
import java.util.List;

public class BlockDetector extends OpenCVPipeline {

    public int contourCount = 0;
    public int maxValIdx = -1;
    public int x = 0;
    public int y = 0;
    public int z = 0;

    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        Mat hsv = new Mat(rgba.rows(), rgba.cols(), rgba.type());
        int minArea = 1000;
        int maxArea = 1000000000;
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV);
        Mat threshold = gray;
        Core.inRange(hsv, new Scalar(0, 120, 80), new Scalar(75, 255, 255), threshold);//154 old s
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        contourCount = contours.size();
        maxValIdx = -1;
        if(contours.size()>0) {
            double minVal = 400;

            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
                double yVal = Imgproc.boundingRect(contours.get(contourIdx)).y;

                if((minVal > yVal) && Imgproc.contourArea(contours.get(contourIdx)) > minArea && Imgproc.contourArea(contours.get(contourIdx)) < maxArea) {
                    minVal = yVal;
                    maxValIdx = contourIdx;
                }
            }

            if(maxValIdx > -1){
                Rect boundingRect = Imgproc.boundingRect(contours.get(maxValIdx));
                Imgproc.rectangle(rgba, boundingRect.tl(),boundingRect.br(),new Scalar (255, 0, 0), 3);

                x = Imgproc.boundingRect(contours.get(maxValIdx)).x;
                y = Imgproc.boundingRect(contours.get(maxValIdx)).y;


            }
        }

        return rgba;
    }

}
