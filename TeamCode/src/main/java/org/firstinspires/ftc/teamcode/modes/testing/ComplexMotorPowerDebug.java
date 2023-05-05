package org.firstinspires.ftc.teamcode.modes.testing;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.PowerPlayRobot;

@TeleOp(name = "[⚒] Complex Motor Power Debug", group = "practice")
public class ComplexMotorPowerDebug extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()) {

            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            long start = System.currentTimeMillis();

            // distance of joystick from center
            double drivePower = Math.hypot(strafe, drive) * (1.5);
            // angle of joystick from center
            double driveAngle = Math.atan2(drive, strafe) + Math.PI / 4;

            // sideways component of drive + strafe turn amount
            double frontLeftPower = drivePower * Math.cos(driveAngle) + turn;
            // forward component of drive + strafe turn
            double frontRightPower = drivePower * Math.sin(driveAngle) + turn;
            // forward component of drive - strafe turn
            double backLeftPower = drivePower * Math.sin(driveAngle) - turn;
            // strafe component of drive - strafe turn
            double backRightPower = drivePower * Math.cos(driveAngle) - turn;

            if (abs(frontLeftPower) > 1 || abs(backLeftPower) > 1 || abs(frontRightPower) > 1 || abs(backRightPower) > 1 ) {
                // normalize max power to be 1, and all other powers to be proportional to it
                double max;
                max = Math.max(abs(frontLeftPower), abs(backLeftPower));
                max = Math.max(abs(frontRightPower), max);
                max = Math.max(abs(backRightPower), max);

                telemetry.addData("drive", drive);
                telemetry.addData("strafe", strafe);
                telemetry.addData("turn", turn);

                telemetry.addData("front left", frontLeftPower / max);
                telemetry.addData("front right", frontRightPower / max);
                telemetry.addData("back left", backLeftPower / max);
                telemetry.addData("back right", backRightPower / max);

                telemetry.addData("calc time", System.currentTimeMillis() - start);
                telemetry.update();

            }

        }

    }

}
