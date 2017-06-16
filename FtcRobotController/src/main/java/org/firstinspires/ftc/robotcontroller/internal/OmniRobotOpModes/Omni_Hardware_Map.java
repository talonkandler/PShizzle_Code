package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Omni_Hardware_Map {

    //Declaring variables
    public DcMotor fleft, fright, bleft, bright;
    public BNO055IMU gyro;
    public float heading;
    public float dp = .3f; //Drive Power (range = 0-1)
    private HardwareMap hwMap;
    private Telemetry telemetry;

    //Constructor; Put program's hardwaremap first, then telemetry,  then put true if gyro will be used or false if it won't
    public Omni_Hardware_Map(HardwareMap hwmap, Telemetry telem, boolean usesGyro){
        hwMap = hwmap;
        telemetry = telem;

        //Setting up drive motors
        fleft = hwMap.dcMotor.get("fleft");
        fright = hwMap.dcMotor.get("fright");
        bleft = hwMap.dcMotor.get("bleft");
        bright = hwMap.dcMotor.get("bright");
        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);

        //Setting up gyro sensor if necessary
        if(usesGyro) {
            gyro = hwMap.get(BNO055IMU.class, "imu");
            //Setting up data for gyro sensors
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro.initialize(parameters);
        }

        //Alerts user that initialization is done
        telemetry.addData("Ready to go", true);
        telemetry.update();
    }

    public void drive(float fl, float fr, float bl, float br) {
        fleft.setPower(ClipValue(fl));
        fright.setPower(ClipValue(fr));
        bleft.setPower(ClipValue(bl));
        bright.setPower(ClipValue(br));
    }

    public void updateGyro() {
        //May not have to make negative?
        heading = - gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
    }

    float ClipValue(float value) {
        if(value > dp || value < - dp)
            return ((Math.abs(value) / value) * dp);
        else
            return value;
    }

    public void doTelemetry() {
        telemetry.addData("Ultra Turbo Mode Activated:", dp == 1);
        telemetry.addData("Gyro Heading:", heading);
        telemetry.update();
    }

}
