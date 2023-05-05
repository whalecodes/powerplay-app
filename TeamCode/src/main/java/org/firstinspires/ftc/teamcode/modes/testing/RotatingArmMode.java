package org.firstinspires.ftc.teamcode.modes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.RotatingArmExtension;

@Config("[⚒] Rotating Arm Mode")
@TeleOp(name = "[⚒] Rotating Arm Mode", group = "practice")
public class RotatingArmMode extends LinearOpMode {

    public static int TARGET = 180;
    public static int SECONDS = 3;

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);
        RotatingArmExtension arm = new RotatingArmExtension(robot.getArmRotator());

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {

                arm.rotateTo(TARGET, SECONDS);
                while (arm.target != arm.position) {
                    arm.update();
                    telemetry.addData("pos", arm.position);
                    telemetry.addData("target", arm.target);
                    telemetry.addData("velocity", arm.velocity);
                    telemetry.update();
                }

            }

        }

    }

}
