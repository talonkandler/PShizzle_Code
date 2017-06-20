package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Omni Field-Centric Drive", group = "omni")
public class Omni_Field_Centric extends OpMode {

    Omni_Hardware_Map robot;
    float fieldCentricAngle = 0; //Controls the field centric angle, for lack of a better term
    float desiredAngle = 0; //Angle the robot should be at
    boolean isTurning = false; //Whether the robot is being turned with the triggers

    @Override
    public void init (){
        robot = new Omni_Hardware_Map(hardwareMap, telemetry, true);
    }

    @Override
    public void loop (){
        //Updates the gyro value
        robot.updateGyro();

        //Going into ultra turbo mode when both triggers are pressed
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            robot.dp = 1;
        else
            robot.dp = 0.3f;

        //Sets the field centric angle to the direction of the robot
        if(gamepad1.start)
            fieldCentricAngle = robot.heading;

        //Determines whether the robot is turning then if it just stopped turning it sets the desired angle so that the robot can determine whether it is drifting rotationally
        if(gamepad1.right_trigger < .05 && gamepad1.left_trigger < .05) {
            if (isTurning)
                desiredAngle = robot.heading;
            isTurning = false;
        }
        else
            isTurning = true;

        //Translating the joystick values into a polar vector
        //atan2 finds the theta of the new polar coordinate in radians (joystick theta)
        float jTheta = (float) Math.atan2(- gamepad1.right_stick_y, gamepad1.right_stick_x) - (float) Math.PI / 2;
        //Makes values from -180 to 180 instead of -270 to 90
        if(jTheta <= -Math.PI)
            jTheta = 2 * (float) Math.PI + jTheta;
        //Power portion of polar coordinate (joystick power)
        float jp = (float) Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);
        //Curtails excessive power values
        if(jp > 1)
            jp = 1;
        //Angle robot should go in relation to its front
        float theta = (jTheta - (float) Math.toRadians(robot.heading) - fieldCentricAngle);

        //May want to find more efficient way to do this if it is too slow or not working
        //Finding ratios of powers
        float frbl = (float) (Math.sin(theta) + Math.cos(theta));
        float flbr = (float) (-Math.sin(theta) + Math.cos(theta));

        //Changing ratios into motor powers
        if(Math.abs(flbr) > Math.abs(frbl)) {
            if(frbl > flbr)
                jp = - jp;
            frbl = frbl * (jp/ flbr);
            flbr = jp;
        }
        else if(Math.abs(frbl) > Math.abs(flbr)) {
            if(flbr > frbl)
                jp = - jp;
            flbr = flbr * (jp/ frbl);
            frbl = jp;
        }
        else {
            flbr = jp * (Math.abs(flbr)/flbr);
            frbl = jp * (Math.abs(frbl)/frbl);
        }

        //Sets the target angle to the robot's direction of motion
        if(gamepad1.a)
            desiredAngle = (float) Math.toDegrees(jTheta);

        //Factors in turing if necessary
        if(isTurning) {
            robot.drive(
                    flbr + gamepad1.right_trigger + gamepad1.left_trigger * -1,
                    frbl + gamepad1.right_trigger * -1 + gamepad1.left_trigger,
                    flbr + gamepad1.right_trigger + gamepad1.left_trigger * -1,
                    frbl + gamepad1.right_trigger * -1 + gamepad1.left_trigger
            );
        }
        //Translates normally but turns if robot is misaligned (1/50 can be adjusted, it just means that robot will devote all power to turning when deviation is 50 degrees or more)
        else {
            float deviation = (1/50) * (robot.heading - desiredAngle);
            robot.drive(
                    flbr + deviation,
                    frbl + deviation * -1,
                    flbr + deviation,
                    frbl + deviation * -1
            );
        }

        telemetry.addData("Ultra Turbo Mode Activated", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Joystick Direction", Math.toDegrees(jTheta));
        telemetry.addData("Joystick Magnitude", gamepad1.right_stick_y);
        telemetry.addData("Gyro Heading", robot.heading);
        telemetry.update();
    }
}