package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

@Autonomous(name="BlockAuto80")
public class BlockAuto80 extends OpMode{
    private final int DOWN_POSITION = -19170;
    Robot myRobot;
    ElapsedTime runtime;
    ElapsedTime blocktime;
    Autobot myAutoBot;
    ACTION currentAction = ACTION.RESET_ACTION;
    private boolean hasSeenBlock = false;
    private boolean hasHitBlock = false;
    private BlockDetector detector;
    int timeResetDelta = 0;

    private enum ACTION{
        RESET_ACTION,
        DOWN_ACTION,
        LEFT_HOOK_ACTION,
        BACKUP_HOOK_ACTION,
        RIGHT_HOOK_ACTION,
        DELAY_1,
        CV_BACKUP,
        DELAY_2,
        FORWARD1_ACTION,
        DELAY_3,
        TURN_LEFT_ACTION,
        DELAY_4,
        STRAF_TO_WALL,
        DELAY_5,
        BACKUP_TO_DEPOT,
        DELAY_6,
        STRAF_FROM_WALL,
        DELAY_7,
        ROTATE_FOR_DROP,
        DELAY_8,
        DROP_ACTION,
        DELAY_9,
        ROTATE_FOR_CRATER,
        DELAY_10,
        STRAF_BACK_TO_WALL,
        DELAY_11,
        CRATER_RUN,
        STOP_ACTION,
    }

    @Override
    public void init() {
        currentAction = ACTION.RESET_ACTION;
        hasSeenBlock = false;
        hasHitBlock = false;
        timeResetDelta = 0;
        myRobot = new Robot(hardwareMap);
        detector = new BlockDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        detector.enable();
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
                if(runtime.milliseconds() > 450){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.RIGHT_HOOK_ACTION;
                    runtime.reset();
                }
                break;
            case RIGHT_HOOK_ACTION:
                myAutoBot.drive(.6, 0, 0);
                if(runtime.milliseconds() > 825){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    blocktime.reset();
                    currentAction = ACTION.DELAY_1;
                }
                break;

            case DELAY_1:
                myAutoBot.drive(0,0,0);
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    currentAction = ACTION.CV_BACKUP;
                }
                break;

            case CV_BACKUP:
                myAutoBot.drive(0, -0.3, 0.025);
                if(runtime.milliseconds() > 1500){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    blocktime.reset();
                    currentAction = ACTION.DELAY_2;
                }
                break;

            case DELAY_2:
                myAutoBot.drive(0,0,0);
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    currentAction = ACTION.FORWARD1_ACTION;
                }
                break;
            case FORWARD1_ACTION:
                if(!hasSeenBlock && detector.maxValIdx > -1 && detector.y < 200){
                    hasSeenBlock = true;
                    blocktime.reset();
                    myRobot.mBlockHitter.setPosition(0);
                    myAutoBot.drive(0,0,0);
                }


                if(!hasSeenBlock){
                    myAutoBot.drive(0, .3, 0);
                }else if(!hasHitBlock){
                    if(blocktime.milliseconds() > 1200){
                        myRobot.mBlockHitter.setPosition(1);
                        hasHitBlock = true;
                    }else if(blocktime.milliseconds() > 600){
                        myAutoBot.drive(0,.3, 0);
                        timeResetDelta = 600;
                    }

                }else{
                    myAutoBot.drive(0,.3, 0);
                }

                if(runtime.milliseconds() - timeResetDelta > 4100){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    myRobot.mSideBack.setPosition(1);
                    myRobot.mSideFront.setPosition(0);
                    currentAction = ACTION.DELAY_3;
                }

                break;


            case DELAY_3:
                myAutoBot.drive(0,0,0);
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    currentAction = ACTION.TURN_LEFT_ACTION;
                }
                break;

            case TURN_LEFT_ACTION:
                myAutoBot.drive(0, 0, -0.3 );
                if(runtime.milliseconds() > 1000){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    currentAction = ACTION.DELAY_4;
                }
                break;

            case DELAY_4:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0.3, 0, 0 );
                    currentAction = ACTION.STRAF_TO_WALL;
                }
                break;

            case STRAF_TO_WALL:
                if(runtime.milliseconds() > 1000){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.DELAY_5;
                    runtime.reset();
                }
                break;

            case DELAY_5:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0, -0.5, 0 );
                    currentAction = ACTION.BACKUP_TO_DEPOT;
                }
                break;
            case BACKUP_TO_DEPOT:
                if(runtime.milliseconds() > 1900){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.DELAY_6;
                    runtime.reset();
                }
                break;
            case DELAY_6:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(-0.3, 0, 0 );
                    currentAction = ACTION.STRAF_FROM_WALL;
                }
                break;
            case STRAF_FROM_WALL:
                if(runtime.milliseconds() > 700){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.DELAY_7;
                    runtime.reset();
                }
                break;
            case DELAY_7:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0, 0, -0.3 );
                    currentAction = ACTION.ROTATE_FOR_DROP;
                }
                break;
            case ROTATE_FOR_DROP:
                if(runtime.milliseconds() > 3000){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    currentAction = ACTION.DELAY_8;
                }
                break;
            case DELAY_8:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    currentAction = ACTION.DROP_ACTION;
                    myRobot.mRelease.setPosition(0);
                }
                break;

            case DROP_ACTION:
                if(runtime.milliseconds() > 100){
                    runtime.reset();
                    currentAction = ACTION.DELAY_9;
                }
                break;

            case  DELAY_9:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0,0,0.3);
                    currentAction = ACTION.ROTATE_FOR_CRATER;
                }
                break;
            case ROTATE_FOR_CRATER:
                if(runtime.milliseconds() > 3100){
                    myAutoBot.drive(0,0,0);
                    runtime.reset();
                    currentAction = ACTION.DELAY_10;
                }
                break;
            case DELAY_10:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0.3, 0, 0 );
                    currentAction = ACTION.STRAF_BACK_TO_WALL;
                }
                break;
            case STRAF_BACK_TO_WALL:
                if(runtime.milliseconds() > 780){
                    myAutoBot.drive(0,0,0);
                    currentAction = ACTION.DELAY_11;
                    runtime.reset();
                }
                break;
            case DELAY_11:
                if(runtime.milliseconds() > 400){
                    runtime.reset();
                    myAutoBot.drive(0.08, 0.5, 0 );
                    currentAction = ACTION.CRATER_RUN;
                }
                break;
            case CRATER_RUN:
                if(runtime.milliseconds() > 2750) {
                    runtime.reset();
                    myAutoBot.drive(0, 0, 0);
                    currentAction = ACTION.STOP_ACTION;
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
