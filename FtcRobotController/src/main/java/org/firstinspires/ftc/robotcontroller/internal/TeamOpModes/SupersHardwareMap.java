package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SupersHardwareMap {
    //Setting up empty hardware map to be replaced later on
    HardwareMap hwMap;
                                                                                                        //HOW TO CONFIGURE THE PHONES:
    //Declaring motor variables                                                                 format: "config name" = M(otor)#, C(ontroller)#
    public DcMotor fleft; //Front left drive motor                                                      "fleft" = M1, C1 with encoder
    public DcMotor fright; //Front right drive motor                                                    "fright" = M2, C1 with encoder
    public DcMotor bleft; //Back left drive motor                                                       "bleft" = M1, C2
    public DcMotor bright; //Back right drive motor                                                     "bright" = M2, C2
    public DcMotor flicker; //Flicker motor                                                             "flicker" = M1, C3 with encoder
    public DcMotor intake; //Intake motor                                                               "intake" = M2, C3

    //Declaring sensor variables                                                                format: "config name" = (category), P(ort)#
    public BNO055IMU imu; //Gyro sensor                                                                 "imu" = I2C (Adafruit IMU), 0
    public OpticalDistanceSensor ods; //Optical distance sensor for stopping in front of the beacon     "color" = I2C (Color Sensor), 1
    public OpticalDistanceSensor ods2; //Optical distance sensor for finding line                       "ods2" = Analog Input (Optical Distance Sensor), 2
    public ColorSensor color; //Color sensor for detecting beacon color                                 "ods" = Analog Input (Optical Distance Sensor), 0

    //Declaring public constants(change to user preference/measurements)
    public static final double WHEEL_DIAMETER = 4 + 7/8;//Unit: inches
    public static final double AUTONOMOUS_DRIVE_SPEED = 0.25f;
    public static final double TELEOP_DRIVE_SPEED = 1f;
    public static final double INTAKE_SPEED = -1f;
    public static final double FLICKER_SPEED = 0.6f;
    public static final double BEACON_DISTANCE = 0.3;
    public static final double FLOOR_REFLECTIVITY = .17;
    public static final double LINE_REFLECTIVITY = .95;
    public static final double MIDDLE_REFLECTIVITY = .56;

    //Setting up variables used in program, made some public so that they are accessible by other programs
    //Some such as "program" don't need to be accessed by other programs, so they are kept local
    public float heading;
    public float lastHeading;
    public float startingAngle;
    float rawGyro;
    float gyroAdd = 0;
    public ElapsedTime timer = new ElapsedTime();
    public boolean notreversed;
    public boolean autonomous;
    LinearOpMode program;

    //Teleop constructor
    public SupersHardwareMap(boolean norev, boolean auto) {
        notreversed = norev;                 //Determines whether the program is notreversed side or red side, set to notreversed side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
    }

    //Autonomous constructor
    public SupersHardwareMap(boolean norev, boolean auto, LinearOpMode op) {
        notreversed = norev;                 //Determines whether the program is notreversed side or red side, set to notreversed side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
        program = op;                       //Allows access to user program
    }

    //Sets up hardware map, reverses motors, etc.
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Setting up the hardwaremap for the motor variables
        fleft = hwMap.dcMotor.get("fright");
        fright = hwMap.dcMotor.get("fleft");
        bleft = hwMap.dcMotor.get("bright");
        bright = hwMap.dcMotor.get("bleft");
        flicker = hwMap.dcMotor.get("flicker");
        intake = hwMap.dcMotor.get("intake");

        //Only sets up sensors in autonomous, otherwise they are not needed
        if(autonomous) {
            imu = hwMap.get(BNO055IMU.class, "imu");
            ods = hwMap.opticalDistanceSensor.get("ods");
            ods2 = hwMap.opticalDistanceSensor.get("ods2");
            color = hwMap.colorSensor.get("color");
            color.enableLed(false);
        }

        //Reversing right motors so that all wheels go the same way
        fright.setDirection(DcMotor.Direction.REVERSE);
        bright.setDirection(DcMotor.Direction.REVERSE);
        //May have to reverse flicker or intake

        //Sets motors to the mode that runs them at a constant power (not enough drive motor encoders to make them run at a constant speed, but that would be preferable for autonomous)
        fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            startingAngle = heading;
            //Waits for the program to start
            program.waitForStart();
        }
    }

    //Moves or brakes the left drive wheels
    public void ldrive(double leftspeed) {
            if (notreversed) {                           //Moves the drive wheels normally if notreversed side
                fleft.setPower(leftspeed);
                bleft.setPower(leftspeed);
            } else {                              //Moves the drive wheels in the opposite direction and switches motors to drive backwards if on red side(or going backward in teleop)
                fright.setPower(-leftspeed);
                bright.setPower(-leftspeed);
            }
    }

    //Moves or brakes the right drive wheels
    public void rdrive(double rightspeed) {
        if (notreversed) {                           //Moves the drive wheels normally if notreversed side
            fright.setPower(rightspeed);
            bright.setPower(rightspeed);
        } else {                              //Moves the drive wheels in the opposite direction and switches motors to drive backwards if on red side(or going backward in teleop)
            fleft.setPower(-rightspeed);
            bleft.setPower(-rightspeed);
        }
    }

    //Waits a certain amount of time, useful for waiting without having to manually reset the timer and all that every time
    public void delay(double seconds) {
        if(autonomous) {
            timer.reset();
            while (timer.seconds() < seconds && program.opModeIsActive()) {
            }
        }
        else {
            timer.reset();
            while (timer.seconds() < seconds){}
        }
    }

    //Runs the flicker a specified number of rotations at the default flicker speed times the specified power coefficient(negative to go backwards)
    public void moveFlicker(double rotations, double powerCoefficient) {
        //Sets the flicker to use the encoder
        flicker.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets target position for encoder
        //May have to change based on gear reduction, multiply by 1/2 for 20 and 3/2 for 60, 40 is standard
        //1440 is one rotation for tetrix, 1120 is one rotation for AndyMark
        flicker.setTargetPosition((int) java.lang.Math.floor(rotations * 1440) + flicker.getCurrentPosition());

        //Sets the power
        flicker.setPower(powerCoefficient * FLICKER_SPEED);

        //Runs the flicker until the target position is reached
        if(autonomous)
            while(Math.abs(flicker.getTargetPosition() - flicker.getCurrentPosition()) > 10 && program.opModeIsActive()) {
                program.telemetry.addData("Flicker target:", flicker.getTargetPosition());
                program.telemetry.addData("Flicker current:", flicker.getCurrentPosition());
                program.telemetry.update();
            }
        else
            while(Math.abs(flicker.getTargetPosition() - flicker.getCurrentPosition()) > 10){}

        //Stops the flicker after the target position is reached
        flicker.setPower(0);
    }

    //TEST OUT ENCODER VALUES TO SEE WHAT NEEDS TO BE REVERSED
    //Drives the robot a specified number of inches at the default autonomous speed times the specified power coefficient(negative to go backwards)
    //Only use in autonomous
    public void driveInches(double inches, double powerCoefficient) {
        //Reverses the amount of inches to go (effectively reversing the encoder values) and speed if the robot is on red mode(backwards)
        if(!notreversed) {
            powerCoefficient = - powerCoefficient;
            inches = - inches;
        }

        //Sets mode to run to position so that the encoders are used
        /*fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //Sets the target position based on the amount of inches to be covered and the starting position
        fleft.setTargetPosition((int) java.lang.Math.floor((inches / (WHEEL_DIAMETER * 3.1416)) * 1120) + fleft.getCurrentPosition()); //Change 1120 based on motor type
        //Right motors are reversed, so negative encoder values should hopefully compensate for that
        //fright.setTargetPosition((int) - java.lang.Math.floor((inches / (WHEEL_DIAMETER * 3.1416)) * 1120) + fright.getCurrentPosition()); //Change 1120 based on motor type

        //Sets motor power based on the default autonomous speed and the power coefficient parameter
        fleft.setPower(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);
        fright.setPower(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);
        bleft.setPower(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);
        bright.setPower(powerCoefficient * AUTONOMOUS_DRIVE_SPEED);

        //If this is too laggy, it may work better to use just one encoder
        //Keeps moving the wheels until they are within 10 encoder ticks of the target value
        while(Math.abs(fleft.getTargetPosition() - fleft.getCurrentPosition()) > 20 /* || Math.abs(fright.getTargetPosition() - fright.getCurrentPosition()) > 20*/ && program.opModeIsActive()) {
           /* //Turns off right or left wheels individually if they reach the target position first
            if(Math.abs(fleft.getTargetPosition() - fleft.getCurrentPosition()) <= 20 && fleft.getPower() != 0) {
                fleft.setPower(0);
                bleft.setPower(0);
            }
            if(Math.abs(fright.getTargetPosition() - fright.getCurrentPosition()) <= 20 && fright.getPower() != 0) {
                fright.setPower(0);
                bright.setPower(0);
            }
            */
            //Sends telemetry for debugging
            program.telemetry.addData("fleft target", fleft.getTargetPosition());
            program.telemetry.addData("fleft current", fleft.getCurrentPosition());
            program.telemetry.addData("fright target", fright.getTargetPosition());
            program.telemetry.addData("fright current", fright.getCurrentPosition());
            program.telemetry.update();
        }

        //Stops wheels after target position has been reached and sets motors back to their regular mode that doesn't use encoders
        fleft.setPower(0);
        fright.setPower(0);
        bleft.setPower(0);
        bright.setPower(0);

        /*fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
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
        program.telemetry.addData("gyro heading:", heading);
        program.telemetry.update();
    }

    //Turns the robot a specified number of degrees, clockwise = negative angle
    //Only use in autonomous
    public void gyroTurn(double degrees) {
        updateGyro();
        //Sets the current robot.heading as the initial heading for reference when turning
        float gyroHeadingInitial = heading;
        //Turns the correct direction until the angle has been reached
        if (degrees <= 0) {
            while (heading > degrees + gyroHeadingInitial && program.opModeIsActive()) {
                ldrive(1.75 * AUTONOMOUS_DRIVE_SPEED);
                rdrive(- 1.75 * AUTONOMOUS_DRIVE_SPEED);
                updateGyro();
            }
        } else {
            while (heading < degrees + gyroHeadingInitial && program.opModeIsActive()) {
                rdrive(1.75 * AUTONOMOUS_DRIVE_SPEED);
                ldrive(- 1.75 * AUTONOMOUS_DRIVE_SPEED);
                updateGyro();
            }
        }

        //Stops wheels
        ldrive(0);
        rdrive(0);
    }

    //Drives up to the line in front of the beacon, follows it, then hits the beacon
    //For the time being, only use in autonomous
    public void hitBeacon(boolean colorisblue) {
        //Drives until line is found
        while(ods2.getLightDetected() < MIDDLE_REFLECTIVITY && program.opModeIsActive()) {
            ldrive(0.75 * AUTONOMOUS_DRIVE_SPEED);
            rdrive(0.75 * AUTONOMOUS_DRIVE_SPEED);
        }

        //Brakes
        ldrive(0);
        rdrive(0);

        delay(0.15);

        //Backs up before turning so that the robot is more lined up with the center of the beacon
        driveInches(-4, -1);

        delay(0.15);

        //Optional thing to replace line following(light sensor would be exactly in the middle of the robot for this)
        //Turns 90 from the starting angle to face the beacon
        updateGyro();
        if(colorisblue)
            gyroTurn(startingAngle - heading - 85);
        else
            gyroTurn(startingAngle - heading + 85);

        //Drives until close enough, and gets slower as it goes(replace with line following, go straight for testing
        while(ods.getLightDetected() < BEACON_DISTANCE && program.opModeIsActive()) {
            double speed = (AUTONOMOUS_DRIVE_SPEED * .4 + 0.07) -  (AUTONOMOUS_DRIVE_SPEED * .45) * (ods.getLightDetected() / BEACON_DISTANCE);
            ldrive(speed);
            rdrive(speed);
        }

        //Brakes
        ldrive(0);
        rdrive(0);

        //Waits half a second to give the color sensor some time
        delay(.5);

        //Turns/rocks left or right depending on the color
        if((colorisblue && color.blue() > color.red()) || (!colorisblue && color.blue() < color.red())) {
            rdrive(1 * AUTONOMOUS_DRIVE_SPEED);
            ldrive(-0.5 * AUTONOMOUS_DRIVE_SPEED);
        }
        else {
            ldrive(AUTONOMOUS_DRIVE_SPEED);
            rdrive(-0.5 * AUTONOMOUS_DRIVE_SPEED);
        }

        //Waits for half a second, then brakes
        delay(1);
        ldrive(0);
        rdrive(0);

        //Drives backwards
        delay(0.15);
        driveInches(-8, -1);

        //Turns towards next beacon
        /*if(colorisblue)
            gyroTurn(startingAngle - + 85ing + 85);
        else
            gyroTurn(startingAngle - - 85ing - 85);*/
    }
}