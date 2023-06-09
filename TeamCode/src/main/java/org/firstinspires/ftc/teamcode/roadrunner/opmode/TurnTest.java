package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "[♻] Turn Test", group = "roadrunner")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        PlainMecanumDrive drive = new PlainMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}
