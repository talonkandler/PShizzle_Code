package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@TeleOp(name = "Holonomic_Drive_Gyro", group = "omni")
public class Holonomic_With_Gyro extends OpMode {
    DcMotor fleft;
    DcMotor fright;
    DcMotor bleft;
    DcMotor bright;

    BNO055IMU gyro;

    float heading;
    double fleftPower = 0;
    double frightPower = 0;
    double bleftPower = 0;
    double brightPower = 0;

    float driveCoefficient = .3f; //Max Power

    @Override
    public void init (){
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);

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

    @Override
    public void loop (){
        //Going into ultra turbo mode when both triggers are pressed
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            driveCoefficient = 1;
        else
            driveCoefficient = 0.3f;

        //Turning clockwise when right trigger is pressed
        if(gamepad1.right_trigger > 0.05) {
            fleftPower = ClipValue(driveCoefficient * gamepad1.right_trigger);
            frightPower = ClipValue(driveCoefficient * gamepad1.right_trigger * -1);
            bleftPower = ClipValue(driveCoefficient * gamepad1.right_trigger);
            brightPower = ClipValue(driveCoefficient * gamepad1.right_trigger * -1);
        }

        //Opposite for left trigger
        else if(gamepad1.left_trigger > 0.05) {
            fleftPower = ClipValue(driveCoefficient * gamepad1.left_trigger * -1);
            frightPower = ClipValue(driveCoefficient * gamepad1.left_trigger);
            bleftPower = ClipValue(driveCoefficient * gamepad1.left_trigger * -1);
            brightPower = ClipValue(driveCoefficient * gamepad1.left_trigger);
        }

        //Stopping motors for concerning
        if(gamepad1.dpad_up && gamepad1.dpad_right)
            frightPower = 0;
        else if(gamepad1.dpad_up && gamepad1.dpad_left)
            fleftPower = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_right)
            brightPower = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_left)
            bleftPower = 0;

        //Updating heading
        heading = gyro.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

        //Translating the joystick values into a polar vector
        double joystickDegrees = Math.toDegrees(Math.atan(gamepad1.right_stick_y / gamepad1.right_stick_x));
        if(gamepad1.right_stick_y < 0)
            joystickDegrees = joystickDegrees + 180;
        double joystickDistance = Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);

        double theta = Math.toRadians(joystickDegrees - heading);

        //Finding ratios of powers
        double flbr = (Math.sin(theta) + Math.cos(theta))/2;
        double frbl = (-Math.sin(theta) + Math.cos(theta))/2;

        //Changing ratios into motor powers
        if(Math.abs(flbr) > Math.abs(frbl)) {
            if(frbl > flbr)
                joystickDistance = - joystickDistance;
            frbl = frbl * ((joystickDistance * driveCoefficient) / flbr);
            flbr = joystickDistance * driveCoefficient;
        }
        else if(Math.abs(frbl) > Math.abs(flbr)) {
            if(flbr > frbl)
                joystickDistance = - joystickDistance;
            flbr = flbr * ((joystickDistance * driveCoefficient) / frbl);
            frbl = joystickDistance * driveCoefficient;
        }
        else {
            flbr = joystickDistance * driveCoefficient * (Math.abs(flbr)/flbr);
            frbl = joystickDistance * driveCoefficient * (Math.abs(frbl)/frbl);
        }

        fleft.setPower(ClipValue(fleftPower + flbr));
        fright.setPower(ClipValue(fleftPower + frbl));
        bleft.setPower(ClipValue(fleftPower + frbl));
        bright.setPower(ClipValue(fleftPower + flbr));

        telemetry.addData("Joystick X Axis:", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y Axis:", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger:", gamepad1.left_trigger);
        telemetry.addData("Right Trigger:", gamepad1.right_trigger);
        telemetry.addData("Pivoting:", gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left);
        telemetry.addData("Ultra Turbo Mode Activated:", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.update();
    }

    double ClipValue(double value) {
        if(value > driveCoefficient || value < - driveCoefficient)
            return ((Math.abs(value) / value) * driveCoefficient);
        else
            return value;
    }
}