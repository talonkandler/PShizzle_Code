package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    //Teleop constructor
    public SupersHardwareMap(boolean blu, boolean auto){
        blue = blu;                         //Determines whether the program is blue side or red side, set to blue side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
    }

    //Autonomous
    public SupersHardwareMap(boolean blu, boolean auto, LinearOpMode op){
        blue = blu;                         //Determines whether the program is blue side or red side, set to blue side for teleop, or red side to go backwards
        autonomous = auto;                  //Says whether program is autonomous or teleop
        program = op;                       //Allows access to user program
    }

    //Moves the drive wheels
    //Too complicated?????? if so, turn into driveright and driveleft
    public void drive(double leftspeed, double rightspeed){
        if(leftspeed != 0 || rightspeed != 0) {
            if (blue) {                           //Moves the drive wheels normally if blue side
                if (leftspeed != 0) {
                    fleft.setPower(leftspeed);
                    bleft.setPower(leftspeed);
                }
                if (rightspeed != 0) {
                    fright.setPower(rightspeed);
                    bright.setPower(rightspeed);
                }
            } else {                              //Moves the drive wheels in the opposite direction and switches motors to drive backwards
                if (leftspeed != 0) {
                    fright.setPower(-leftspeed);
                    bright.setPower(-leftspeed);
                }
                if (rightspeed != 0) {
                    fleft.setPower(-rightspeed);
                    bleft.setPower(-rightspeed);
                }
            }
        }
        else {
            fleft.setPower(0);
            bleft.setPower(0);
            fright.setPower(0);
            bright.setPower(0);
        }
    }

    //Runs the flicker a specified number of rotations at the default flicker speed times the specified power coefficient(negative to go backwards)
    public void moveFlicker(double rotations, double powerCoefficient){
        //May have to change based on gear reduction, multiply by 1/2 for 20 and 3/2 for 60, 40 is standard
        int encoderInput = (int) java.lang.Math.floor(rotations * 1120) ;

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
    public void driveInches(double inches, double powerCoefficient){
        //Figures out what value to give the encoder based on the amount of inches to be covered
        int encoderInput = (int) java.lang.Math.floor((inches / (WHEEL_DIAMETER * 3.1416)) * 1120); //Change 1120 based on motor type

        //Resets encoders and sets the power and position to be used
        fright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fright.setTargetPosition(encoderInput);
        fleft.setTargetPosition(encoderInput);

        //Sets motor power
        drive(powerCoefficient * AUTONOMOUS_DRIVE_SPEED, powerCoefficient * AUTONOMOUS_DRIVE_SPEED);

        fright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();

        //If this is too laggy, it may work better to cut out the second encoder
        while(Math.abs(fright.getCurrentPosition()) < Math.abs(fright.getTargetPosition()) || (fleft.getCurrentPosition()) < Math.abs(fleft.getTargetPosition()) && timer.seconds() < 5 && program.opModeIsActive()) {
            if(Math.abs(fright.getCurrentPosition()) >= Math.abs(fright.getTargetPosition())) {
                if(blue) {
                    fright.setPower(0);
                    bright.setPower(0);
                }
                else {
                    fleft.setPower(0);
                    bleft.setPower(0);
                }

            }
            if(Math.abs(fleft.getCurrentPosition()) >= Math.abs(fleft.getTargetPosition())) {
                if(blue) {
                    fleft.setPower(0);
                    bleft.setPower(0);
                }
                else {
                    fright.setPower(0);
                    bright.setPower(0);
                }
            }
        }

        //Stops wheels and sets motors back to their regular mode
        drive(0,0);
        fleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Waits a certain amount of time
    public void delay(double seconds) {
        timer.reset();
        while (timer.seconds() < seconds);
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

        //Reversing right motors so that all wheels go the same way
        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);
        //May have to reverse flicker or intake

        //If this is called in an autonomous program, it waits for the program to start before proceeding
        try{
        if(autonomous)
            program.waitForStart();
        } catch(java.lang.InterruptedException exc) {}
    }
}