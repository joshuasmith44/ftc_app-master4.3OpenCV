package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Matrix34F;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

/**
 * Created by joshuasmith on 11/12/17.
 */

public class Autobot {
    private Robot myRobot;
    private int angleOffset = 0;
    private OpMode mOpmode;
    private boolean mProcessedImage = false;
    private final int X_CROP_DELTA = 38;
    private final int Y_CROP_DELTA = 38;
    private OpenGLMatrix mLastRawPose;
    public static final double SERVO_LOW  = 0.0 ;
    public static final double SERVO_HIGH = 1.0 ;
    public Autobot(Robot inputBot, OpMode opmode){
        myRobot = inputBot;
        mOpmode = opmode;
        //myRobot.initImu();

    }


    public void strafeForLength2(int encoderVal, boolean right, int timeout){
        int encoderFLOffset = myRobot.mFrontLeftMotor.getCurrentPosition();
        int encoderFROffset = myRobot.mFrontRightMotor.getCurrentPosition();
        int encoderBLOffset = myRobot.mBackLeftMotor.getCurrentPosition();
        int encoderBROffset = myRobot.mBackRightMotor.getCurrentPosition();
        int averageEncoderVal = 0;
        long startTime = System.currentTimeMillis();
        while(averageEncoderVal < encoderVal && (int)(System.currentTimeMillis() - startTime) < timeout){
            if(right){
                drive(0.25, 0, 0);
            }else{
                drive(-0.25, 0, 0);
            }
            averageEncoderVal = (Math.abs(myRobot.mFrontLeftMotor.getCurrentPosition() - encoderFLOffset) + Math.abs(myRobot.mFrontRightMotor.getCurrentPosition()- encoderFROffset) +
                    Math.abs(myRobot.mBackRightMotor.getCurrentPosition()- encoderBROffset) + Math.abs(myRobot.mBackLeftMotor.getCurrentPosition()- encoderBLOffset)) / 4;
        }
        drive(0,0,0);
    }



    public void driveStraightForLength( int encoderLength, int timeout){
        int averageEncoderVal = (Math.abs(myRobot.mFrontLeftMotor.getCurrentPosition()) + Math.abs(myRobot.mFrontRightMotor.getCurrentPosition()) +
                                 Math.abs(myRobot.mBackRightMotor.getCurrentPosition()) + Math.abs(myRobot.mBackLeftMotor.getCurrentPosition())) / 4;
        int encoderValOffset = averageEncoderVal;
        long startTime = System.currentTimeMillis();
        while(averageEncoderVal - encoderValOffset < encoderLength && (int)(System.currentTimeMillis() - startTime) < timeout){
            drive(0, 0.2, 0);
            averageEncoderVal = (Math.abs(myRobot.mFrontLeftMotor.getCurrentPosition()) + Math.abs(myRobot.mFrontRightMotor.getCurrentPosition()) +
                                 Math.abs(myRobot.mBackRightMotor.getCurrentPosition()) + Math.abs(myRobot.mBackLeftMotor.getCurrentPosition())) / 4;
        }
        drive(0,0,0);
    }

    public void driveStraightForLength( int encoderLength, int timeout, double speed){
        int encoderFLOffset = myRobot.mFrontLeftMotor.getCurrentPosition();
        int encoderFROffset = myRobot.mFrontRightMotor.getCurrentPosition();
        int encoderBLOffset = myRobot.mBackLeftMotor.getCurrentPosition();
        int encoderBROffset = myRobot.mBackRightMotor.getCurrentPosition();
        int averageEncoderVal = 0;
        long startTime = System.currentTimeMillis();
        while(averageEncoderVal < encoderLength && (int)(System.currentTimeMillis() - startTime) < timeout){
            drive(0, speed, 0);
            averageEncoderVal = (Math.abs(myRobot.mFrontLeftMotor.getCurrentPosition() - encoderFLOffset) + Math.abs(myRobot.mFrontRightMotor.getCurrentPosition()- encoderFROffset) +
                    Math.abs(myRobot.mBackRightMotor.getCurrentPosition()- encoderBROffset) + Math.abs(myRobot.mBackLeftMotor.getCurrentPosition()- encoderBLOffset)) / 4;
        }
        drive(0,0,0);
    }



    public void printHeading(){
        while(true){
            Orientation angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            mOpmode.telemetry.addData("Degrees", angles.firstAngle);
            mOpmode.telemetry.update();
        }
    }

    public void remapAxes(){
        byte AXIS_MAP_CONFIG_BYTE=0x6;
        byte AXIS_MAP_SIGN_BYTE=0x1;
        myRobot.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        pause(100);
        myRobot.imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        myRobot.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        myRobot.imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        pause(100);
    }

    public void pause(int milli) {
        ElapsedTime timer=new ElapsedTime();
        timer.reset();
        while(timer.milliseconds()<milli){

        }
    }

    public void turnForTime(double r, int time){
        drive(0,0,r);
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < time){
        }
        drive(0,0,0);
    }

    public void pTurnDegrees(double degrees, int timeout){
        Orientation angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int currentHeading = (int)angles.firstAngle;
        angleOffset = currentHeading;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.milliseconds() < timeout){

        }
    }

    public void turnDegrees(double degrees){
        degrees *= -1;
        Orientation angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        int currentHeading = (int)angles.firstAngle;
        angleOffset = currentHeading;
        while(Math.abs((currentHeading - angleOffset) - degrees) > 10) {
            angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = (int) angles.firstAngle;
            mOpmode.telemetry.addData("Angle", currentHeading);
            mOpmode.telemetry.update();
            if (degrees >= 0) {
                drive(0, 0, -0.3);
            } else {
                drive(0, 0, 0.3);
            }
            angles = myRobot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        }
        drive(0,0,0);
    }





    public void drive(double x, double y, double r){
        /*if( Math.abs(y) <= 0.09){
            y = 0;
        }
        if( Math.abs(x) <= 0.09){
            x = 0;
        }
        if( Math.abs(r) <= 0.09){
            r = 0;
        }*/


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

    public void unHang() {
        myRobot.mHangingMotor.setPower(-1.0);
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 8000){
        }
        myRobot.mHangingMotor.setPower(-0.5);
        startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 1000){
        }
        myRobot.mHangingMotor.setPower(0);
        drive(.35, 0, 0);
        startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 500){
        }
        drive(-0.25, -0.3, -0.4);
        startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < 1300){
        }
        drive(0,0,0);
    }

    public void startDetection() {
        myRobot.detector.init(myRobot.mHardwareMap.appContext, CameraViewDisplay.getInstance());
        myRobot.detector.enable();
    }

    public int getBlockX(){
        return myRobot.detector.x;
    }

    public int getBlockY(){
        return myRobot.detector.y;
    }

    public boolean seeBlock(){
        return myRobot.detector.maxValIdx > -1;
    }

    public void drop() {
        myRobot.mRelease.setPosition(0);
    }


}
