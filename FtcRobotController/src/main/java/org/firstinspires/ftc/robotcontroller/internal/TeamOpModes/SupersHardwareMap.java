package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SupersHardwareMap {
    //Setting up empty hardware map to be replaced later on
    HardwareMap hwMap;

    //Declaring motor variables                         Format: config name = M(otor)#, C(ontroller)#
    public DcMotor fleft; //Front left drive motor              fleft = M1, C1 with encoder
    public DcMotor fright; //Front right drive motor            fright = M2, C1 with encoder
    public DcMotor bleft; //Back left drive motor               bleft = M1, C2
    public DcMotor bright; //Back right drive motor             bright = M2, C2
    public DcMotor flicker; //Flicker motor                     flicker = M1, C3 with encoder
    public DcMotor intake; //Intake motor                       intake = M2, C3
    public BNO055IMU imu; //Gyro sensor
    public OpticalDistanceSensor ods; //Optical distance sensor
    public ColorSensor color; //Color sensor

    //Declaring public constants(change to user preference/measurements)
    public static final double WHEEL_DIAMETER = 5;//? inches
    public static final double AUTONOMOUS_DRIVE_SPEED = 0.5f;
    public static final double TELEOP_DRIVE_SPEED = 1f;
    public static final double INTAKE_SPEED = -1f;
    public static final double FLICKER_SPEED = 0.8f;

    //Setting up variables used in program
    public ElapsedTime timer = new ElapsedTime();
    public boolean blue;
    public boolean autonomous;
    LinearOpMode program;

    public float heading;
    public float lastHeading;
    float rawGyro;
    float gyroAdd = 0;

    //Teleop constructor
    public SupersHardwareMap(boolean blu, boolean auto) {
        blue = blu;                         //Determines whether the program is blue side or red side, set to blue side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
    }

    //Autonomous
    public SupersHardwareMap(boolean blu, boolean auto, LinearOpMode op) {
        blue = blu;                         //Determines whether the program is blue side or red side, set to blue side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
        program = op;                       //Allows access to user program
    }

    //Sets up hardware map, reverses motors, etc.
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Assigning values to the previously declared motor, beacon, and sensor variables
        fleft = hwMap.dcMotor.get("fright");    //Name motors/servos/sensors in phone configuration based on the variable name
        fright = hwMap.dcMotor.get("fleft");
        bleft = hwMap.dcMotor.get("bright");
        bright = hwMap.dcMotor.get("bleft");
        flicker = hwMap.dcMotor.get("flicker");
        intake = hwMap.dcMotor.get("intake");
        if(autonomous) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            ods = hwMap.opticalDistanceSensor.get("ods");
            color = hwMap.colorSensor.get("color");
        }

        //Reversing right motors so that all wheels go the same way
        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);
        //May have to reverse flicker or intake

        if(autonomous) {
            //Setting up data for gyro sensors
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            imu.initialize(parameters);
            updateGyro();
            //Waits for the program to start
            program.waitForStart();
        }
    }

    //Moves the left drive wheels
    public void ldrive(double leftspeed) {
            if (blue) {                           //Moves the drive wheels normally if blue side
                fleft.setPower(leftspeed);
                bleft.setPower(leftspeed);
            } else {                              //Moves the drive wheels in the opposite direction and switches motors to drive backwards
                fright.setPower(-leftspeed);
                bright.setPower(-leftspeed);
            }
    }

    //Moves the right drive wheels
    public void rdrive(double rightspeed) {
        if (blue) {                           //Moves the drive wheels normally if blue side
            fright.setPower(rightspeed);
            bright.setPower(rightspeed);
        } else {                              //Moves the drive wheels in the opposite direction and switches motors to drive backwards
            fleft.setPower(-rightspeed);
            bleft.setPower(-rightspeed);
        }
    }

    //Waits a certain amount of time
    public void delay(double seconds) {
        timer.reset();
        while (timer.seconds() < seconds);
    }

    //Runs the flicker a specified number of rotations at the default flicker speed times the specified power coefficient(negative to go backwards)
    public void moveFlicker(double rotations, double powerCoefficient) {
        //May have to change based on gear reduction, multiply by 1/2 for 20 and 3/2 for 60, 40 is standard
        //1440 is one rotation for tetrix, 1120 is one rotation for AndyMark
        int encoderInput = (int) java.lang.Math.floor(rotations * 1440) ;

        //Sets target position for encoder
        flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flicker.setTargetPosition(encoderInput);
        flicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Runs motor until the encoder value is reached
        flicker.setPower(powerCoefficient * FLICKER_SPEED);
        timer.reset();
        if(autonomous)
            while(Math.abs(flicker.getCurrentPosition()) < Math.abs(flicker.getTargetPosition()) && timer.seconds() < 5 && program.opModeIsActive()){}
        else
            while(Math.abs(flicker.getCurrentPosition()) < Math.abs(flicker.getTargetPosition()) && timer.seconds() < 5){}
        flicker.setPower(0);
    }

    //Drives the robot a specified number of inches at the default autonomous speed times the specified power coefficient(negative to go backwards)
    //Only use in autonomous
    public void driveInches(double inches, double powerCoefficient) {
        //Figures out what value to give the encoder based on the amount of inches to be covered
        int encoderInput = (int) java.lang.Math.floor((inches / (WHEEL_DIAMETER * 3.1416)) * 1120); //Change 1120 based on motor type

        //Resets encoders and sets the power and position to be used
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setTargetPosition(encoderInput);
        fleft.setTargetPosition(encoderInput);

        //Sets motor power
        ldrive(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);
        rdrive(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);

        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();

        //If this is too laggy, it may work better to cut out the second encoder
        while(Math.abs(fright.getCurrentPosition()) < Math.abs(fright.getTargetPosition()) || (fleft.getCurrentPosition()) < Math.abs(fleft.getTargetPosition()) && timer.seconds() < 5 && program.opModeIsActive()) {
            if(Math.abs(fright.getCurrentPosition()) >= Math.abs(fright.getTargetPosition())) {
                if(blue) {
                    rdrive(0);
                }
                else {
                    ldrive(0);
                }

            }
            if(Math.abs(fleft.getCurrentPosition()) >= Math.abs(fleft.getTargetPosition())) {
                if(blue) {
                    ldrive(0);
                }
                else {
                    rdrive(0);
                }
            }
        }

        //Stops wheels and sets motors back to their regular mode
        ldrive(0);
        rdrive(0);
        fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Updates the gyro sensor and formats the angle so that it is easier to use
    //Only use in autonomous
    public void updateGyro() {
        //Gets the raw value of the gyro sensor
        rawGyro = -imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX).firstAngle;

        //Detects if the gyro sensor goes from 0-360 or 360-0 and adjusts gyroAdd to compensate
        if (lastHeading < 60 && rawGyro > 300) {
            gyroAdd = gyroAdd + 360;
        } else if (lastHeading > 300 && rawGyro < 60) {
            gyroAdd = gyroAdd - 360;
        }

        //Puts formatted angle in heading variable and sets the current value as last value for the next cycle
        heading = gyroAdd - rawGyro;
        lastHeading = rawGyro;
    }

    //Turns the robot a specified number of degrees, clockwise = negative angle
    //Only use in autonomous
    public void gyroTurn(float degrees) {
        //Sets the current robot.heading as the initial heading for reference when turning
        float gyroHeadingInitial = heading;
        //Turns the correct direction until the angle has been reached
        if (degrees <= 0) {
            while (heading > degrees + gyroHeadingInitial && program.opModeIsActive()) {
                ldrive(AUTONOMOUS_DRIVE_SPEED);
                rdrive(-AUTONOMOUS_DRIVE_SPEED);
                updateGyro();
            }
        } else {
            while (heading < degrees + gyroHeadingInitial && program.opModeIsActive()) {
                ldrive(-AUTONOMOUS_DRIVE_SPEED);
                rdrive(AUTONOMOUS_DRIVE_SPEED);
                updateGyro();
            }
        }

        //Stops wheels
        ldrive(0);
        rdrive(0);
    }
}