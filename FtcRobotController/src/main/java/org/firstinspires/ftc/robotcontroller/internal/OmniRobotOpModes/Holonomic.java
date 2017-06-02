package org.firstinspires.ftc.robotcontroller.internal.OmniRobotOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Holonomic_Drive", group = "omni")
public class Holonomic extends OpMode {
    DcMotor fleft;
    DcMotor fright;
    DcMotor bleft;
    DcMotor bright;

    float driveCoefficient = .3f;

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
        if(gamepad1.right_bumper && gamepad1.left_bumper)
            driveCoefficient = 1;
        else
            driveCoefficient = 0.3f;

        if(gamepad1.right_trigger > 0.05) {
            fleft.setPower( ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            fright.setPower( ClipValue( driveCoefficient * (gamepad1.right_trigger)));
            bleft.setPower( ClipValue( driveCoefficient * (gamepad1.right_trigger * -1)));
            bright.setPower( ClipValue( driveCoefficient * (gamepad1.right_trigger)));
        }

        else if(gamepad1.left_trigger > 0.05) {
            fleft.setPower( ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            fright.setPower( ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
            bleft.setPower( ClipValue( driveCoefficient * (gamepad1.left_trigger)));
            bright.setPower( ClipValue( driveCoefficient * (gamepad1.left_trigger * -1)));
        }

        else {
            fleft.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
            fright.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
            bleft.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x * -1)));
            bright.setPower( ClipValue( driveCoefficient * (-gamepad1.right_stick_y - gamepad1.right_stick_x)));
        }
    }

    float ClipValue(float value) {
        if(value > driveCoefficient || value < - driveCoefficient)
            return ((Math.abs(value) / value) * driveCoefficient);
        else
            return value;
    }
}