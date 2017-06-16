package org.firstinspires.ftc.robotcontroller.internal.PreviousSeasonOpModes.FirstResQOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "O_Drive_Shoot", group = "Autonomous")
public class O_Drive_Shoot extends LinearOpMode {
    //Setting the hardware map as a global variable
    SupersHardwareMap robot;

    public void runOpMode(){
        //Sets up the hardware map
        robot = new SupersHardwareMap(false);
        robot.init(hardwareMap);
        waitForStart();

        //Drives forward and shoots twice then turns towards the beacon line
        driveInches(18, 1.5);
        robot.timer.reset();
        while(robot.timer.seconds() < 1 && opModeIsActive()){}
        moveFlicker(1, 1);
        robot.timer.reset();
        while(robot.timer.seconds() < 1 && opModeIsActive()){}
        moveFlicker(1, 1);
    }

    //Runs the flicker a specified number of rotations at the default flicker speed times the specified power coefficient(negative to go backwards)
    public void moveFlicker(double rotations, double powerCoefficient) {
        //Sets target position for encoder
        //May have to change based on gear reduction, multiply by 1/2 for 20 and 3/2 for 60, 40 is standard
        //1440 is one rotation for tetrix, 1120 is one rotation for AndyMark
        robot.flicker.setTargetPosition((int) java.lang.Math.floor(rotations * 3/2 * 1120) + robot.flicker.getCurrentPosition());

        //Sets the power
        robot.flicker.setPower(-powerCoefficient * robot.FLICKER_SPEED);

        //Runs the flicker until slightly before the target position is reached, but once it brakes it will be in the right place
        while(Math.abs(robot.flicker.getTargetPosition() - robot.flicker.getCurrentPosition()) > 120 && opModeIsActive()) {
            telemetry.addData("Flicker target:", robot.flicker.getTargetPosition());
            telemetry.addData("Flicker current:", robot.flicker.getCurrentPosition());
            telemetry.update();
        }

        //Stops the flicker after the target position is reached
        robot.flicker.setPower(0);
    }
    //TEST OUT ENCODER VALUES TO SEE WHAT NEEDS TO BE REVERSED
    //Drives the robot a specified number of inches at the default autonomous speed times the specified power coefficient(negative to go backwards)
    //Only use in autonomous
    public void driveInches(int inches, double powerCoefficient) {
        //Reverses the amount of inches to go (effectively reversing the encoder values) and speed if the robot is on red mode(backwards)
        if(!robot.notreversed) {
            powerCoefficient = - powerCoefficient;
            inches = - inches;
        }

        //Sets mode to run to position so that the encoders are used
        /*robot.fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        //Sets the target position based on the amount of inches to be covered and the starting position
        robot.fleft.setTargetPosition((int) java.lang.Math.floor((inches / (robot.WHEEL_DIAMETER * 3.1416)) * 1120) + robot.fleft.getCurrentPosition()); //Change 1120 based on motor type
        //Right motors are reversed, so negative encoder values should hopefully compensate for that
        //robot.fright.setTargetPosition((int) - java.lang.Math.floor((inches / (WHEEL_DIAMETER * 3.1416)) * 1120) + robot.fright.getCurrentPosition()); //Change 1120 based on motor type

        //Sets motor power based on the default autonomous speed and the power coefficient parameter
        robot.fleft.setPower(powerCoefficient * robot.AUTONOMOUS_DRIVE_SPEED);
        robot.fright.setPower(powerCoefficient * robot.AUTONOMOUS_DRIVE_SPEED);
        robot.bleft.setPower(powerCoefficient * robot.AUTONOMOUS_DRIVE_SPEED);
        robot.bright.setPower(powerCoefficient * robot.AUTONOMOUS_DRIVE_SPEED);

        //If this is too laggy, it may work better to use just one encoder
        //Keeps moving the wheels until they are within 10 encoder ticks of the target value
        while(Math.abs(robot.fleft.getTargetPosition() - robot.fleft.getCurrentPosition()) > 20 /* || Math.abs(robot.fright.getTargetPosition() - robot.fright.getCurrentPosition()) > 20*/ && opModeIsActive()) {
           /* //Turns off right or left wheels individually if they reach the target position first
            if(Math.abs(robot.fleft.getTargetPosition() - robot.fleft.getCurrentPosition()) <= 20 && robot.fleft.getPower() != 0) {
                robot.fleft.setPower(0);
                robot.bleft.setPower(0);
            }
            if(Math.abs(robot.fright.getTargetPosition() - robot.fright.getCurrentPosition()) <= 20 && robot.fright.getPower() != 0) {
                robot.fright.setPower(0);
                robot.bright.setPower(0);
            }
            */
            //Sends telemetry for debugging
            telemetry.addData("robot.fleft target", robot.fleft.getTargetPosition());
            telemetry.addData("robot.fleft current", robot.fleft.getCurrentPosition());
            telemetry.addData("robot.fright target", robot.fright.getTargetPosition());
            telemetry.addData("robot.fright current", robot.fright.getCurrentPosition());
            telemetry.update();
        }

        //Stops wheels after target position has been reached and sets motors back to their regular mode that doesn't use encoders
        robot.fleft.setPower(0);
        robot.fright.setPower(0);
        robot.bleft.setPower(0);
        robot.bright.setPower(0);
    }
}
