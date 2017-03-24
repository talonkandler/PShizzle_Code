package org.firstinspires.ftc.robotcontroller.internal.TeamOpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Test", group = "Sensor")
public class Sensor_Test extends OpMode {

    SupersHardwareMap robot = new SupersHardwareMap(true, false);
    public void init() {robot.init(hardwareMap);}

    public void loop() {
        telemetry.addData("Color sensor blue:", robot.color.blue());
        telemetry.addData("Color sensor red:", robot.color.red());
        telemetry.addData("ODS Reading:", robot.ods.getLightDetected());
        telemetry.addData("Gyro heading:", robot.heading);
    }

}