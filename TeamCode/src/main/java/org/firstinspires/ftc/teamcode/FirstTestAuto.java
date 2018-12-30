package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
@Autonomous(name="AutonTest")

public class FirstTestAuto extends LinearOpMode {

    private Robot myRobot;
    private ElapsedTime runtime;
    private Autobot myAutoBot;

    //private BlockDetector detector;

    @Override
    public void runOpMode() {

        myRobot = new Robot(hardwareMap);

        myRobot.init();
        telemetry.addData("initialized", null);
        runtime = new ElapsedTime();
        runtime.reset();
        myAutoBot = new Autobot(myRobot, this);
        waitForStart();
        myAutoBot.unHang();

    }

}
