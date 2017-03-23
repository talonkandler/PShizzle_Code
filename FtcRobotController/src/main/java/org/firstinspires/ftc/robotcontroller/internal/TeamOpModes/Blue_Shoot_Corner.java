package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Blue_Shoot_Corner extends LinearOpMode {
    SupersHardwareMap robot = new SupersHardwareMap(true, true, this);
    @Override
    public void runOpMode() {
        //Position in front of center vortex
        //Shoots
        robot.moveFlicker(1, 1);
        //  Runs intake
        robot.timer.reset();
        robot.intake.setPower(robot.INTAKE_SPEED);
        while(robot.timer.seconds() < 3){}
        robot.intake.setPower(0);
        //Shoots
        robot.moveFlicker(1, 1);

        //Drive forward and turn towards corner, then drive onto corner
        robot.driveInches(18, 1);
        //robot.gyroTurn(135);
        robot.driveInches(36, 1);
    }
}
