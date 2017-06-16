package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Omni Field-Centric Drive", group = "omni")
public class Omni_Field_Centric extends OpMode {

    Omni_Hardware_Map robot;

    @Override
    public void init (){
        robot = new Omni_Hardware_Map(hardwareMap, telemetry, true);
    }

    @Override
    public void loop (){

        //Going into ultra turbo mode when both triggers are pressed
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            robot.dp = 1;
        else
            robot.dp = 0.3f;

        //Translating the joystick values into a polar vector
        //atan2 finds the theta of the new polar coordinate in radians (joystick theta)
        float jTheta = (float) Math.atan2(- gamepad1.right_stick_y, gamepad1.right_stick_x);
        //Power portion of polar coordinate (joystick power)
        float jp = (float) Math.sqrt(gamepad1.right_stick_x * gamepad1.right_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);
        //Angle robot should go in relation to its front
        float theta = jTheta - (float) Math.toRadians(robot.heading);

        //May want to find more efficient way to do this if it is too slow or not working
        //Finding ratios of powers
        float flbr = (float) (Math.sin(theta) + Math.cos(theta))/2;
        float frbl = (float) (-Math.sin(theta) + Math.cos(theta))/2;

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

        robot.drive(
                flbr + gamepad1.right_trigger + gamepad1.left_trigger * -1,
                frbl + gamepad1.right_trigger * -1 + gamepad1.left_trigger,
                flbr + gamepad1.right_trigger + gamepad1.left_trigger * -1,
                frbl + gamepad1.right_trigger * -1 + gamepad1.left_trigger
        );

        telemetry.addData("Joystick X Axis", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y Axis", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Ultra Turbo Mode Activated", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.addData("Gyro Heading", robot.heading);
        telemetry.update();
    }
}