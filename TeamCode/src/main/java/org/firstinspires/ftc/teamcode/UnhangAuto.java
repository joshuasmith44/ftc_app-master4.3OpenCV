package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="UnhangMotor")
public class UnhangAuto extends OpMode{
    private final int DOWN_POSITION = -19170;
    Robot myRobot;
    ElapsedTime runtime;
    Autobot myAutoBot;
    ACTION currentAction = ACTION.RESET_ACTION;

    private enum ACTION{
        RESET_ACTION,
        DOWN_ACTION,
        LEFT_HOOK_ACTION,
        BACKUP_HOOK_ACTION,
        STOP_ACTION
    }

    @Override
    public void init() {
        myRobot = new Robot(hardwareMap);

        myRobot.init();
        telemetry.addData("initialized", null);
        runtime = new ElapsedTime();
        runtime.reset();
        myAutoBot = new Autobot(myRobot, this);
        myRobot.mHangingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hAits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch(currentAction){
            case RESET_ACTION:
                if(myRobot.mHangingMotor.getCurrentPosition() == 0)
                    myRobot.mHangingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    myRobot.mHangingMotor.setTargetPosition(DOWN_POSITION);
                    myRobot.mHangingMotor.setPower(1.0);
                    currentAction = ACTION.DOWN_ACTION;
                break;
            case DOWN_ACTION:
                if(Math.abs(myRobot.mHangingMotor.getCurrentPosition() - myRobot.mHangingMotor.getTargetPosition()) < 15){
                    currentAction = ACTION.LEFT_HOOK_ACTION;
                    runtime.reset();
                }
                break;
            case LEFT_HOOK_ACTION:
                myAutoBot.drive(-0.3, 0, 0);
                if(runtime.milliseconds() > 250){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.BACKUP_HOOK_ACTION;
                    runtime.reset();
                }
            case BACKUP_HOOK_ACTION:
                myAutoBot.drive(0,-0.3, 0);
                if(runtime.milliseconds() > 800){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.STOP_ACTION;
                    runtime.reset();
                }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
