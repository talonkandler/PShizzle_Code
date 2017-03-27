package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "beacon", group = "autonomous")
public class Blue_Shoot_Beacons_Corner extends LinearOpMode{
        public void runOpMode() {
            SupersHardwareMap robot = new SupersHardwareMap(false, true, this);
            robot.init(hardwareMap);
            /*robot.driveInches(10, 1);
            robot.gyroTurn(50);*/
            robot.hitBeacon(false);
            robot.gyroTurn(robot.startingAngle - robot.heading - 15);
            robot.driveInches(10, 1);
            robot.hitBeacon(false);
            robot.gyroTurn(145);
            while(robot.ods.getLightDetected() < .8) {
                robot.ldrive(robot.AUTONOMOUS_DRIVE_SPEED);
                robot.rdrive(robot.AUTONOMOUS_DRIVE_SPEED);
            }
            robot.ldrive(0);
            robot.rdrive(0);
            robot.driveInches(5, 1);
        }
}

