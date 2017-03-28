package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

//Necessary imports
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Adding this program to the phones
@TeleOp(name = "Special TeleOp", group = "TeleOp")

public class Special_Tele_Op extends OpMode {
    //Setting up robot class
    SupersHardwareMap robot = new SupersHardwareMap(true, false);

    //Setting up global variables
    double driveSpeed = robot.TELEOP_DRIVE_SPEED;
    //Increase threshold value if joysticks aren't resetting to exactly 0
    double threshold = 0.05f;

    //Setting up hardware maps, reversing motors, etc
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    //Runs all of the functions
    @Override
    public void loop() {
        runWheels();
        reverseAndSpeed();
        flickers();
        runIntake();
        rock();
        Telemetry();
    }

    //Drives right drive motors based on right joystick and left drive motors based on left joystick
    public void runWheels() {
        //Threshold ensures that the motors wont move when joystick is released even if the joysticks don't reset exactly to 0
            if(gamepad2.left_stick_y != 0) {
                robot.notreversed = true;
                robot.ldrive(-gamepad2.left_stick_y * robot.TELEOP_DRIVE_SPEED);
            }
            else if(gamepad1.right_stick_y != 0){
                robot.notreversed = false;
                robot.rdrive(-gamepad1.right_stick_y * driveSpeed);
            }
            else {
                if(!gamepad1.right_bumper && !gamepad1.left_bumper) {
                robot.fleft.setPower(0);
                robot.bleft.setPower(0);
                }
            }

            if (gamepad2.right_stick_y != 0) {
                robot.notreversed = true;
                robot.rdrive(-gamepad2.right_stick_y * robot.TELEOP_DRIVE_SPEED);
            } else if (gamepad1.left_stick_y != 0) {
                robot.notreversed = false;
                robot.ldrive(-gamepad1.left_stick_y * driveSpeed);
            } else {
                if(!gamepad1.right_bumper && !gamepad1.left_bumper) {
                    robot.fright.setPower(0);
                    robot.bright.setPower(0);
                }
            }
    }

    //Changes the front of the robot for driving purposes when dpad up or down is pressed
    public void reverseAndSpeed(){
        //Makes the controller drive the robot with the intake as the front if the dpad is pressed up
        if(gamepad1.dpad_up)
            robot.notreversed = true;
            //Makes the controller drive the robot with the intake as the back if the dpad is pressed down
        else if(gamepad1.dpad_down)
            robot.notreversed = false;
        //Slows down the drive speed when the left trigger is pressed
        driveSpeed = robot.TELEOP_DRIVE_SPEED - 0.8 * Math.abs(gamepad1.left_trigger);
    }

    //Rotates flicker backwards when left trigger is held, shoots when right trigger is held (move to second gamepad for drive practice)
    public void flickers(){
        if(gamepad1.right_trigger > 0.8)
            robot.moveFlicker(1, 1);
        else
            robot.flicker.setPower(0);
    }

    //When joystick is pushed up, the intake motor pushes balls out, when it is pushed down, the intake motor pulls balls in
    public void runIntake() {
        if(Math.abs(gamepad2.right_trigger) > 0.1)
            robot.intake.setPower(gamepad2.right_trigger * robot.INTAKE_SPEED);
        else if(Math.abs(gamepad2.left_trigger) > 0.1)
            robot.intake.setPower(gamepad2.left_trigger * -robot.INTAKE_SPEED);
        else
            robot.intake.setPower(0);
    }

    public void rock() {
        if(gamepad1.right_bumper) {
            robot.fleft.setPower(-0.5);
            robot.bleft.setPower(-0.5);
            robot.fright.setPower(0.25);
            robot.bright.setPower(0.25);
        }
        else if(gamepad1.left_bumper) {
            robot.fleft.setPower(0.25);
            robot.bleft.setPower(0.25);
            robot.fright.setPower(-0.5);
            robot.bright.setPower(-0.5);
        }
    }

    //Tells how to control robot for drivers, debugging info can be added here
    public void Telemetry(){
        telemetry.addData("Instructions:","(C = controller)");
        telemetry.addData("Tank Drive:", "C1, L/R joysticks");
        telemetry.addData("Set Front(Reverse):", "C1, up and down on dpad");
        telemetry.addData("Adjust Flicker position:", "C2, L/R triggers");
        telemetry.addData("Shoot:", "C2, a button");
        telemetry.addData("Pull in balls:", "C2, L joystick down");
        telemetry.addData("Push out balls:", "C2, L joystick up");
    }
}