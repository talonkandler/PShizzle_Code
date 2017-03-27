package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Red_Shoot_Beacons_Cap(test)", group = "Autonomous")
public class Red_Shoot_Beacons_Middle extends LinearOpMode {
    public void runOpMode(){
        //Sets up the hardware map
        SupersHardwareMap robot = new SupersHardwareMap(false, true, this);
        robot.init(hardwareMap);

        //Drives forward and shoots twice then turns towards the beacon line
        robot.driveInches(10, 1);
        robot.moveFlicker(1, 1);
        robot.delay(1);
        robot.moveFlicker(1, 1);
        robot.gyroTurn(50);

        //Drives towards line and follows it, hits beacon, turns towards next beacon and repeats
        robot.hitBeacon(false);
        robot.gyroTurn(robot.startingAngle - robot.heading + 8);
        robot.driveInches(10, 1);
        robot.hitBeacon(false);

        //Turns towards the cap ball, drives into it, turns to fling it off to the side, then turns back and parks on center platform
        robot.gyroTurn(145);
        while(robot.ods.getLightDetected() < .9) {
            robot.ldrive(robot.AUTONOMOUS_DRIVE_SPEED);
            robot.rdrive(robot.AUTONOMOUS_DRIVE_SPEED);
        }
        robot.ldrive(0);
        robot.rdrive(0);
        robot.gyroTurn(20);
        robot.delay(0.5);
        robot.gyroTurn(-20);
        robot.driveInches(5, 1);
    }
}
