package org.firstinspires.ftc.teamcode.modes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlayRobot;

@TeleOp(name = "[⚒] Motor Debug Mode", group = "practice")
public class MotorDebugMode extends LinearOpMode {

    public static double MOTOR_POWER = 0.4;

    @Override
    public void runOpMode() {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Motor</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Left</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Front Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Right</font>");
            telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Rear&nbsp;&nbsp;Left</font>");
            telemetry.addLine();

            if (gamepad1.x) {
                robot.getFrontLeftWheel().setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
                telemetry.addLine("Motor Direction: " + robot.getFrontLeftWheel().getDirection().name());
                telemetry.addLine("Motor Position: " + robot.getFrontLeftWheel().getCurrentPosition());
            } else if (gamepad1.y) {
                robot.getFrontRightWheel().setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
                telemetry.addLine("Motor Direction: " + robot.getFrontRightWheel().getDirection().name());
                telemetry.addLine("Motor Position: " + robot.getFrontRightWheel().getCurrentPosition());
            } else if (gamepad1.b) {
                robot.getBackRightWheel().setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Back Right");
                telemetry.addLine("Motor Direction: " + robot.getBackRightWheel().getDirection().name());
                telemetry.addLine("Motor Position: " + robot.getBackRightWheel().getCurrentPosition());
            } else if (gamepad1.a) {
                robot.getBackLeftWheel().setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Back Left");
                telemetry.addLine("Motor Direction: " + robot.getBackLeftWheel().getDirection().name());
                telemetry.addLine("Motor Position: " + robot.getBackLeftWheel().getCurrentPosition());
            } else {
                robot.setMotorPower(0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
