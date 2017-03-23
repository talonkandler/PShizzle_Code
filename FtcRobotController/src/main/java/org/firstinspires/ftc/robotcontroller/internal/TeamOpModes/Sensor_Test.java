package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class Sensor_Test extends OpMode {

    //Declaring sensor variables
    BNO055IMU imu;
    ColorSensor color;
    LightSensor leftLight;
    LightSensor rightLight;
    UltrasonicSensor ultrasonic;

    float heading;
    float lastHeading;
    float gyroAdd = 0;
    float rawGyro;

    public void init() {
        //Setting up sensors and such
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        color = hardwareMap.colorSensor.get("color");
        leftLight = hardwareMap.lightSensor.get("leftLight");
        rightLight = hardwareMap.lightSensor.get("rightLight");
        ultrasonic = hardwareMap.ultrasonicSensor.get("ultrasonic");

        color.enableLed(false);
        //Turning on sensor light to enhance accuracy of readings
        leftLight.enableLed(true);
        rightLight.enableLed(true);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    public void loop() {
        rawGyro = -imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;
        if (lastHeading < 60 && rawGyro > 300) {
            gyroAdd = gyroAdd + 360;
        } else if (lastHeading > 300 && rawGyro < 60) {
            gyroAdd = gyroAdd - 360;
        }

        //Turning right makes heading lower, left makes it higher
        heading = gyroAdd - rawGyro;
        lastHeading = rawGyro;

        telemetry.addData("Color sensor blue:", color.blue());
        telemetry.addData("Color sensor red:", color.red());
        telemetry.addData("Left light sensor value:", leftLight.getLightDetected());
        telemetry.addData("Right light sensor value:", rightLight.getLightDetected());
        telemetry.addData("Gyro heading:", heading);
        telemetry.addData("Ultrasonic value:", ultrasonic.getUltrasonicLevel() / 2.54);
    }

}