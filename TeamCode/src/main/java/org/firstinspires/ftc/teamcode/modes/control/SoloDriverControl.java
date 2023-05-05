package org.firstinspires.ftc.teamcode.modes.control;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.SoloDrivingExtension;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

// Human-controlled mode where one user controls the robot, generally for testing
@TeleOp(name = "[✔] Solo-Driver Mode", group = "competition")
public class SoloDriverControl extends OpMode {

    private SoloDrivingExtension drive;

    @Override
    public void init() {
        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap)
                .setDefaults();
        PlainMecanumDrive drive = new PlainMecanumDrive(hardwareMap);
        this.drive = new SoloDrivingExtension(robot, drive, gamepad1, telemetry);
    }

    @Override
    public void loop() {
        drive.loop();
        telemetry.update();
    }

}
