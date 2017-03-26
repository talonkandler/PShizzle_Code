package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "beacon", group = "autonomous")
public class Blue_Shoot_Beacons_Corner extends LinearOpMode{
        public void runOpMode() {
            SupersHardwareMap robot = new SupersHardwareMap(false, true, this);
            robot.init(hardwareMap);
            robot.hitBeacon(false);
        }
}

