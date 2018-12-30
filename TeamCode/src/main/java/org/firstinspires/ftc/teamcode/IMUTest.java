package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="IMUTest")
//@Disabled
public class IMUTest extends OpMode {

    private Robot myRobot;
    private ElapsedTime runtime;
    private final double teleOpArmDelayVal = 20;
    private int tiltEncoderMin;
    private int extensionEncoderMin;
    ElapsedTime armTiltTime = new ElapsedTime();
    ElapsedTime extensionTiltTime = new ElapsedTime();

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        myRobot = new Robot(hardwareMap);

        myRobot.init();


        runtime = new ElapsedTime();
        runtime.reset();

        myRobot.mExtensionMotor.setPower(1.0);
        myRobot.mTiltMotor.setPower(1.0);

        tiltEncoderMin = myRobot.mTiltMotor.getCurrentPosition();
        extensionEncoderMin = myRobot.mExtensionMotor.getCurrentPosition();
        myRobot.mTiltMotor.setTargetPosition(myRobot.mTiltMotor.getCurrentPosition());
        myRobot.mExtensionMotor.setTargetPosition(myRobot.mExtensionMotor.getCurrentPosition());
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("initialized", null);




    }


}
