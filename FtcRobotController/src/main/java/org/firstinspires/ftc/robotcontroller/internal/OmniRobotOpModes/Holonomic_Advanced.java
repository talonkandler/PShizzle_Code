package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Holonomic_Drive_Advanced", group = "omni")
public class Holonomic_Advanced extends OpMode {
    DcMotor fleft;
    DcMotor fright;
    DcMotor bleft;
    DcMotor bright;

    float driveCoefficient = .3f;

    int fleftOn = 1;
    int frightOn = 1;
    int bleftOn = 1;
    int brightOn = 1;

    @Override
    public void init (){
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");

        fright.setDirection(DcMotorSimple.Direction.REVERSE);
        bright.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop (){
        //I think right and left are switched so I fixed it the lazy way but whatevs
        if(gamepad1.dpad_up && gamepad1.dpad_right)
            fleftOn = 0;
        else if(gamepad1.dpad_up && gamepad1.dpad_left)
            frightOn = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_right)
            bleftOn = 0;
        else if(gamepad1.dpad_down && gamepad1.dpad_left)
            brightOn = 0;

        if(gamepad1.right_bumper && gamepad1.left_bumper)
            driveCoefficient = 1;
        else
            driveCoefficient = 0.3f;

        if(gamepad1.right_trigger > 0.05) {
            fleft.setPower( fleftOn * ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            fright.setPower( frightOn * ClipValue( driveCoefficient * (gamepad1.right_trigger)));
            bleft.setPower( bleftOn * ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            bright.setPower( brightOn * ClipValue( driveCoefficient * (gamepad1.right_trigger)));
        }

        else if(gamepad1.left_trigger > 0.05) {
            fleft.setPower( fleftOn * ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            fright.setPower( frightOn * ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
            bleft.setPower( bleftOn * ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            bright.setPower( brightOn * ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
        }

        else {
            fleft.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
            fright.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
            bleft.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
            bright.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
        }

        telemetry.addData("Joystick X Axis:", gamepad1.right_stick_x);
        telemetry.addData("Joystick Y Axis:", gamepad1.right_stick_y);
        telemetry.addData("Left Trigger:", gamepad1.left_trigger);
        telemetry.addData("Right Trigger:", gamepad1.right_trigger);
        telemetry.addData("Pivoting:", gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left);
        telemetry.addData("Ultra Turbo Mode Activated:", gamepad1.right_bumper && gamepad1.left_bumper);
        telemetry.update();

        fleftOn = 1;
        frightOn = 1;
        bleftOn = 1;
        brightOn = 1;
    }

    float ClipValue(float value) {
        if(value > driveCoefficient || value < - driveCoefficient)
            return ((Math.abs(value) / value) * driveCoefficient);
        else
            return value;
    }
}