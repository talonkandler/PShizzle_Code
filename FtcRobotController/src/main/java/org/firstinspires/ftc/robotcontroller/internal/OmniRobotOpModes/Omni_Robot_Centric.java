package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Omni Robot-Centric Drive", group = "omni")
public class Omni_Robot_Centric extends OpMode {

    Omni_Hardware_Map robot;

    //Array for concerning(fl = 0, fr = 1, bl = 2, br = 3)
    float[] concerning = new float[4];

    @Override
    public void init (){
        robot = new Omni_Hardware_Map(hardwareMap, telemetry, false);
    }

    @Override
    public void loop (){
        //Reseting concerning
        concerning[0] = 1;
        concerning[1] = 1;
        concerning[2] = 1;
        concerning[3] = 1;

        //Use tophat corners for concerning
        if(gamepad1.dpad_up && gamepad1.dpad_left)
            concerning[0] = 0;
        else if(gamepad1.dpad_up && gamepad1.dpad_right)
            concerning[1] = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_left)
            concerning[2] = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_right)
            concerning[3] = 0;

        //Hold both bumpers for ultra-turbo mode
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            robot.dp = 1f;
        else
            robot.dp = 0.3f;

        //Arcade drive with right joystick, turn with triggers(clockwise-right, counterclockwise-left)
        robot.drive(
                concerning[0] * (gamepad1.right_trigger + gamepad1.left_trigger * -1) + -gamepad1.right_stick_y + gamepad1.right_stick_x,
                concerning[1] * (gamepad1.right_trigger * -1 + gamepad1.left_trigger) + -gamepad1.right_stick_y + gamepad1.right_stick_x * -1,
                concerning[2] * (gamepad1.right_trigger + gamepad1.left_trigger * -1) + -gamepad1.right_stick_y + gamepad1.right_stick_x * -1,
                concerning[3] * (gamepad1.right_trigger * -1 + gamepad1.left_trigger) + -gamepad1.right_stick_y + gamepad1.right_stick_x
            );

        //Telemetry
        telemetry.addData("Joystick X Axis:", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y Axis:", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger:", gamepad1.left_trigger);
        telemetry.addData("Right Trigger:", gamepad1.right_trigger);
        telemetry.addData("Pivoting:", gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left);
        telemetry.addData("Ultra Turbo Mode Activated:", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.update();
    }
}