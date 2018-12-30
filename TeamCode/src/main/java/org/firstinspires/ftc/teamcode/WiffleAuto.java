package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="WiffleAuto")
public class WiffleAuto extends OpMode{
    private final int DOWN_POSITION = -19170;
    Robot myRobot;
    ElapsedTime runtime;
    ElapsedTime blocktime;
    Autobot myAutoBot;
    ACTION currentAction = ACTION.RESET_ACTION;

    private enum ACTION{
        RESET_ACTION,
        DOWN_ACTION,
        LEFT_HOOK_ACTION,
        BACKUP_HOOK_ACTION,
        RIGHT_HOOK_ACTION,
        FORWARD1_ACTION,
        TURN_LEFT_ACTION,
        FORWARD2_ACTION,
        DROP_ACTION,
        BACK2_ACTION,
        STOP_ACTION,
    }

    @Override
    public void init() {
        myRobot = new Robot(hardwareMap);

        myRobot.init();
        telemetry.addData("initialized", null);
        runtime = new ElapsedTime();
        runtime.reset();
        blocktime = new ElapsedTime();
        blocktime.reset();
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
                if(runtime.milliseconds() > 400){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.BACKUP_HOOK_ACTION;
                    runtime.reset();
                }
                break;
            case BACKUP_HOOK_ACTION:
                myAutoBot.drive(0,-0.3, .05);
                if(runtime.milliseconds() > 500){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.RIGHT_HOOK_ACTION;
                    runtime.reset();
                }
                break;
            case RIGHT_HOOK_ACTION:
                myAutoBot.drive(.6, 0, 0);
                if(runtime.milliseconds() > 800){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    blocktime.reset();
                    currentAction = ACTION.FORWARD1_ACTION;
                }
                break;
            case FORWARD1_ACTION:
                myAutoBot.drive(0, .4, 0);
                if(blocktime.milliseconds() > 100){
                    myRobot.mBlockHitter.setPosition(0);
                }
                if(blocktime.milliseconds() > 1150){
                    myRobot.mBlockHitter.setPosition(1);
                }
                if(runtime.milliseconds() > 1600){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    myRobot.mSideBack.setPosition(1);
                    myRobot.mSideFront.setPosition(0);
                    currentAction = ACTION.TURN_LEFT_ACTION;
                }
                break;
            case TURN_LEFT_ACTION:
                myAutoBot.drive(0, 0, -0.3);
                if(runtime.milliseconds() > 585){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    currentAction = ACTION.FORWARD2_ACTION;

                }
                break;
            case FORWARD2_ACTION:
                myAutoBot.drive(.2, .45, 0);
                if(runtime.milliseconds() > 2200){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    currentAction = ACTION.DROP_ACTION;
                }
                break;
            case DROP_ACTION:
                if(runtime.milliseconds() > 100){
                    myRobot.mRelease.setPosition(0);
                }
                if(runtime.milliseconds() > 600){
                    runtime.reset();
                    currentAction = ACTION.BACK2_ACTION;
                    myAutoBot.drive(0,0,0);
                }
                break;
            case BACK2_ACTION:
                myAutoBot.drive(.12,-0.45,0);
                if(runtime.milliseconds() > 4000){
                    runtime.reset();
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.STOP_ACTION;
                    myRobot.mHangingMotor.setPower(0);
                    myRobot.mHangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    myRobot.mSideBack.setPosition(0);
                    myRobot.mSideFront.setPosition(1);
                }
                break;
            case STOP_ACTION:
                break;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
