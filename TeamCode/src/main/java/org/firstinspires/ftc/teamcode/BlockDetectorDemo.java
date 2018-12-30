package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;
@TeleOp(name="BlockDetectorDemo")
public class BlockDetectorDemo extends OpMode{
    private BlockDetector detector;

    public void init(){
        detector = new BlockDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        detector.enable();
    }

    public void loop(){
        telemetry.addData("seesBlock" , detector.maxValIdx > -1 && detector.y < 200);
        telemetry.addData("yVal", detector.y);
    }
}
