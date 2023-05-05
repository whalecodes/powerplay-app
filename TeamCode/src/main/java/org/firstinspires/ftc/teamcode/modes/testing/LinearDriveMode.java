package org.firstinspires.ftc.teamcode.modes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;

@TeleOp(name = "[⚒] Linear-Drive Control", group = "practice")
public class LinearDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);

        telemetry.addLine("This mode is not designed for functional use.");
        telemetry.addLine("Only use this when testing the linear control of this robot.");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {

            telemetry.clear();
            double drive = gamepad1.left_stick_y;

            if (drive != 0) {

                robot.driveByPower(drive);
                telemetry.addLine("The robot is DRIVING at " + drive + " power!");
                telemetry.update();
                continue;

            }

            double strafe = gamepad1.left_stick_x;

            if (strafe != 0) {

                robot.strafeByPower(strafe);
                telemetry.addLine("The robot is STRAFING at " + strafe + " power!");
                telemetry.update();
                continue;

            }

            double turn = gamepad1.right_stick_x;

            if (turn != 0) {

                robot.turnByPower(turn);
                telemetry.addLine("The robot is TURNING at " + turn + " power!");
                telemetry.update();

            }

        }

    }

}
