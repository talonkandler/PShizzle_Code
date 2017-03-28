package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Blue_Shoot_Beacons_Cap(test)", group = "Autonomous")
public class Blue_Shoot_Beacons_Middle extends LinearOpMode {
    public void runOpMode(){
        //Sets up the hardware map
        SupersHardwareMap robot = new SupersHardwareMap(false, true, this);
        robot.init(hardwareMap);
        waitForStart();

        /*//Drives forward and shoots twice then turns towards the beacon line
        robot.driveInches(13, 2);
        robot.timer.reset();
        while(robot.timer.seconds() < 0.25){}
        robot.moveFlicker(1, 1);
        robot.timer.reset();
        while(robot.timer.seconds() < 1){}
        robot.moveFlicker(1, 1);
        robot.gyroTurn(-50);*/

        //Drives towards line and follows it, hits beacon, turns towards next beacon and repeats
        robot.hitBeacon(true);
        robot.gyroTurn(robot.startingAngle - robot.heading);
        robot.driveInches(10, 1);
        robot.hitBeacon(true);

        //Turns towards the cap ball, shoves it, and parks on center platform
        robot.gyroTurn(-122);

        robot.color.enableLed(true);

        while(robot.color.blue() < 2) {
            robot.ldrive(1.5 * robot.AUTONOMOUS_DRIVE_SPEED);
            robot.rdrive(1.5 * robot.AUTONOMOUS_DRIVE_SPEED);
        }
        robot.ldrive(0);
        robot.rdrive(0);
        robot.driveInches(5, 1);
    }
}