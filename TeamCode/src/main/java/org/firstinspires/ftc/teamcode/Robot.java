package org.firstinspires.ftc.teamcode;

/**
 * Created by joshuasmith on 11/12/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.*;
import com.vuforia.ar.pl.DrawOverlayView;

//import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

public class Robot {
    public HardwareMap mHardwareMap;
    public final int LIFT_ENCODER_DELTA = 45;
    public DcMotor mFrontLeftMotor;
    public DcMotor mFrontRightMotor;
    public DcMotor mBackRightMotor;
    public DcMotor mBackLeftMotor;
    public DcMotor mSweeperMotor;
    public DcMotor mTiltMotor;
    public DcMotor mExtensionMotor;
    public DcMotor mHangingMotor;

    public Servo mSideFront;
    public Servo mSideBack;
    public final int tiltDelta = 6155;
    public final int extensionDelta = 3602;
    public final int groundLiftVal = 1000;
    public Servo mDeposit;
    public Servo mDeploy;
    public Servo mRelease;
    public Servo mBlockHitter;
    public ModernRoboticsI2cRangeSensor range;
    BNO055IMU imu;
    public BlockDetector detector;

    public Robot(HardwareMap input) {
        mHardwareMap = input;
    }

    public void initImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        /*imu = mHardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {

        }*/
    }

    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        mFrontLeftMotor = mHardwareMap.get(DcMotor.class, "FrontLeft");
        mFrontRightMotor = mHardwareMap.get(DcMotor.class, "FrontRight");
        mBackLeftMotor = mHardwareMap.get(DcMotor.class, "BackLeft");
        mBackRightMotor = mHardwareMap.get(DcMotor.class, "BackRight");
        mSweeperMotor = mHardwareMap.get(DcMotor.class, "DaWrist");
        mTiltMotor = mHardwareMap.get(DcMotor.class, "DaScrew");
        mExtensionMotor = mHardwareMap.get(DcMotor.class, "DaStretch");
        mHangingMotor = mHardwareMap.get(DcMotor.class, "DaHang");

        mFrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mFrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mBackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mSweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mTiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mHangingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        mFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mSweeperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mTiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mExtensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mHangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        mBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        mDeposit = mHardwareMap.get(Servo.class, "DaDump");
        mDeposit.setPosition(0);
        mSideBack = mHardwareMap.get(Servo.class, "SideBack");
        mSideBack.setPosition(0);
        mSideFront = mHardwareMap.get(Servo.class, "SideFront");
        mSideFront.setPosition(1);
        mBlockHitter = mHardwareMap.get(Servo.class, "DaSmack");
        mBlockHitter.setPosition(1);
        mDeploy = mHardwareMap.get(Servo.class, "Deploy");
        mDeploy.setPosition(.5);
        mRelease = mHardwareMap.get(Servo.class, "Release");
        mRelease.setPosition(.5);
        //detector = new BlockDetector();
        // start the vision system
    }

}
