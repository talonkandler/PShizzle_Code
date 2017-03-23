package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TwoFlickerHardwareMap {
    //Setting up empty hardware map to be replaced later on
    HardwareMap hwMap =  null;

    //Declaing motor variables                                  Wiring will depend on which end is the front
    public DcMotor fleft = null; //Front left drive motor       fleft = M(otor)1, C(ontroller)1
    public DcMotor fright = null; //Front right drive motor     fright = M2, C1
    public DcMotor bleft = null; //Back left drive motor        bleft = M1, C2
    public DcMotor bright = null; //Back right drive motor      bright = M2, C2

    public DcMotor lflicker = null; //Left flicker              lflicker = M1, C3
    public DcMotor rflicker = null; //Right Flicker             rflicker = M2, C3

    public Servo balldumper = null; //Ball positioning servo    balldumper = S(ervo)1, C1

    //Change wheelDiameter depending on the diameter of the drive wheels
    public static final double WHEEL_DIAMETER = 5;//? inches

    //Set driveSpeed depending on speed preference
    public static final double AUTONOMOUS_DRIVE_SPEED = 0.5f;
    public static final double TELEOP_DRIVE_SPEED = 0.5f;
    public static final double INTAKE_SPEED = -1f;
    public static final double FLICKER_SPEED = 0.8f;
    boolean dumpLeft = true;

    //Constructor
    public TwoFlickerHardwareMap(){}

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Assigning values to the previously declared motor, beacon, and sensor variables
        fleft = hwMap.dcMotor.get("fright");    //Name motors/servos/sensors in phone configuration based on the variable name
        fright = hwMap.dcMotor.get("fleft");
        bleft = hwMap.dcMotor.get("bright");
        bright = hwMap.dcMotor.get("bleft");

        lflicker = hwMap.dcMotor.get("lflicker");
        rflicker = hwMap.dcMotor.get("rflicker");

        balldumper = hwMap.servo.get("balldumper");

        //Reversing right motors so that all wheels go the same way
        fleft.setDirection(DcMotor.Direction.REVERSE);
        bleft.setDirection(DcMotor.Direction.REVERSE);
        rflicker.setDirection(DcMotor.Direction.REVERSE);

        balldumper.setPosition(0.5);
    }

    public void dumpBall(){
        if(dumpLeft) {
            balldumper.setPosition(0);
            balldumper.setPosition(0.5);
            dumpLeft = false;
        }
        else {
            balldumper.setPosition(1);
            balldumper.setPosition(0.5);
            dumpLeft = true;
        }
    }
}