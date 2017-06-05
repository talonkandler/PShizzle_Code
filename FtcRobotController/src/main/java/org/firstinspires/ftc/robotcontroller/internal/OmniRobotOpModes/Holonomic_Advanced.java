package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Holonomic_Drive_Advanced", group = "omni")
public class Holonomic_Advanced extends OpMode {
    DcMotor fleft;
    DcMotor fright;
    DcMotor bleft;
    DcMotor bright;
    CRServo gear1;
    CRServo gear2;

    float driveCoefficient = .3f;

    float fleftOn = 1;
    float frightOn = 1;
    float bleftOn = 1;
    float brightOn = 1;

    @Override
    public void init (){
        fleft = hardwareMap.dcMotor.get("fleft");
        fright = hardwareMap.dcMotor.get("fright");
        bleft = hardwareMap.dcMotor.get("bleft");
        bright = hardwareMap.dcMotor.get("bright");
        gear1 = hardwareMap.crservo.get("gear1");

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
            fleftOn = ( fleftOn * ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            frightOn = ( frightOn * ClipValue( driveCoefficient * (gamepad1.right_trigger)));
            bleftOn = ( bleftOn * ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            brightOn = ( brightOn * ClipValue( driveCoefficient * (gamepad1.right_trigger)));
        }

        else if(gamepad1.left_trigger > 0.05) {
            fleftOn = ( fleftOn * ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            frightOn = ( frightOn * ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
            bleftOn = ( bleftOn * ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            brightOn = ( brightOn * ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
        }
        else {
            fleftOn = 0;
            frightOn = 0;
            bleftOn = 0;
            brightOn = 0;
        }

        fleft.setPower( ClipValue( fleftOn + driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
        fright.setPower( ClipValue( frightOn + driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
        bleft.setPower( ClipValue( bleftOn + driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
        bright.setPower( ClipValue( brightOn + driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));

        if(fleft.getPower() != 0 || fright.getPower() != 0) {
            gear1.setPower(1);
            gear2.setPower(1);
        }
        else {
            gear1.setPower(0.5);
            gear2.setPower(0.5);
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