/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="FullDrive")
//@Disabled

public class FullDrive extends OpMode
{
    // Declare OpMode members.

    private Robot myRobot;
    private ElapsedTime runtime;
    private final double teleOpArmDelayVal = 20;
    private int tiltEncoderMin;
    private int extensionEncoderMin;
    ElapsedTime armTiltTime = new ElapsedTime();
    ElapsedTime extensionTiltTime = new ElapsedTime();
    boolean autoSweeper = false;
    boolean autoSweeper1 = false;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        myRobot = new Robot(hardwareMap);

        myRobot.init();

        telemetry.addData("initialized", null);
        runtime = new ElapsedTime();
        runtime.reset();

        myRobot.mExtensionMotor.setPower(1.0);
        myRobot.mTiltMotor.setPower(1.0);

        tiltEncoderMin = myRobot.mTiltMotor.getCurrentPosition();
        extensionEncoderMin = myRobot.mExtensionMotor.getCurrentPosition();
        myRobot.mTiltMotor.setTargetPosition(myRobot.mTiltMotor.getCurrentPosition());
        myRobot.mExtensionMotor.setTargetPosition(myRobot.mExtensionMotor.getCurrentPosition());


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        myRobot.mBlockHitter.setPosition(.7);

        // Setup a variable for each drive wheel to save power level for telemetry
        telemetry.addData("Hanging Val: ", myRobot.mHangingMotor.getCurrentPosition() );
        telemetry.addData("Tilt Val: ", myRobot.mTiltMotor.getCurrentPosition() );
        telemetry.addData("Extension Val: ", myRobot.mExtensionMotor.getCurrentPosition() );

        driveMotors();
        handleArm();
        handleHang();
        // Show the elapsed game time and wheel power.

    }

    private void handleHang(){
        if(Math.abs(gamepad1.right_trigger) > 0.3){
            myRobot.mHangingMotor.setPower(-1.0);
        }else if (Math.abs(gamepad1.left_trigger) > 0.3){
            myRobot.mHangingMotor.setPower(1.0);
        }else{
            myRobot.mHangingMotor.setPower(0);
        }
    }

    private void handleArm(){
        if(gamepad2.x){
            myRobot.mDeploy.setPosition(0);
        }else{
            myRobot.mDeploy.setPosition(0.5);
        }

        if (gamepad2.y){
            myRobot.mDeposit.setPosition(1);
            autoSweeper = true;
        } else {

            myRobot.mDeposit.setPosition(0);
            autoSweeper = false;
        }



        if(Math.abs(gamepad2.right_trigger) > 0.4){
            myRobot.mSweeperMotor.setPower(-1.0);
        }else if(Math.abs(gamepad2.left_trigger)>0.4){
            myRobot.mSweeperMotor.setPower(1.0);
        } else if (autoSweeper){
            myRobot.mSweeperMotor.setPower(-0.7);
        }else if (autoSweeper1){
            myRobot.mSweeperMotor.setPower(-1.0);
        }
        else{
            myRobot.mSweeperMotor.setPower(0);
        }

        if(gamepad2.dpad_up){
            autoSweeper1 = true;
            myRobot.mTiltMotor.setTargetPosition(tiltEncoderMin + myRobot.tiltDelta);
            myRobot.mExtensionMotor.setTargetPosition(extensionEncoderMin);
        }else if(gamepad2.dpad_down){
            myRobot.mTiltMotor.setTargetPosition(extensionEncoderMin + myRobot.groundLiftVal);
            myRobot.mExtensionMotor.setTargetPosition(extensionEncoderMin);
            autoSweeper1 = false;
        }

        if(Math.abs(myRobot.mTiltMotor.getCurrentPosition() - (tiltEncoderMin+ myRobot.tiltDelta))< 30) {
            autoSweeper1 = false;
        }



        if(Math.abs(gamepad2.right_stick_y) > 0.2){
            if(extensionTiltTime.milliseconds()>10){
                myRobot.mExtensionMotor.setTargetPosition(myRobot.mExtensionMotor.getCurrentPosition() + (int)(190 * -gamepad2.right_stick_y));
                extensionTiltTime.reset();
            }
        }
        myRobot.mExtensionMotor.setTargetPosition(clipMinMax(extensionEncoderMin, extensionEncoderMin + myRobot.extensionDelta, myRobot.mExtensionMotor.getTargetPosition()));


        if(Math.abs(gamepad2.left_stick_y) > 0.2){
            if(armTiltTime.milliseconds()>10){
                myRobot.mTiltMotor.setTargetPosition(myRobot.mTiltMotor.getCurrentPosition() + (int)(300 * -gamepad2.left_stick_y));
                armTiltTime.reset();
            }
        }
        myRobot.mTiltMotor.setTargetPosition(clipMinMax(tiltEncoderMin, tiltEncoderMin + myRobot.tiltDelta, myRobot.mTiltMotor.getTargetPosition()));

    }

    private int clipMinMax(int min, int max, int val){
        if(val < min){
            val = min;
        }else if(val > max){
            val = max;
        }
        return val;
    }

    private void driveMotors(){
        double y = -gamepad1.left_stick_y;
        y = y * Math.abs(y);
        double x = gamepad1.left_stick_x;
        x = x * Math.abs(x);
        double r = gamepad1.right_stick_x * 0.5;

        if(gamepad1.dpad_up){
            y = 0.350;
        }else if(gamepad1.dpad_down){
            y = -0.350;
        }

        if(gamepad1.dpad_left){
            x = -0.350;
        }else if(gamepad1.dpad_right){
            x = 0.350;
        }


        if( Math.abs(y) <= 0.10){
            y = 0;
        }
        if( Math.abs(x) <= 0.10){
            x = 0;
        }
        if( Math.abs(r) <= 0.10){
            r = 0;
        }


        double FRPower = y-x-r;
        double FLPower = y+x+r;
        double BLPower = y-x+r;
        double BRPower = y+x-r;

        double max = Math.abs(FLPower);
        if(Math.abs(FRPower)>max) max = Math.abs(FRPower);
        if(Math.abs(BRPower)>max) max = Math.abs(BRPower);
        if(Math.abs(BLPower)>max) max = Math.abs(BLPower);

        if(max > 1){
            FLPower /= max;
            FRPower /= max;
            BRPower /= max;
            BLPower /= max;
        }

        myRobot.mFrontLeftMotor.setPower(FLPower);
        myRobot.mFrontRightMotor.setPower(FRPower);
        myRobot.mBackRightMotor.setPower(BRPower);
        myRobot.mBackLeftMotor.setPower(BLPower);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
