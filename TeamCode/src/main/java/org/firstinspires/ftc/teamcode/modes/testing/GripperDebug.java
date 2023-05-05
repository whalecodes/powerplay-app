package org.firstinspires.ftc.teamcode.modes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;

@Config("Gripper Debug Mode")
@TeleOp(name = "[⚒] Gripper Debug Mode", group = "practice")
public class GripperDebug extends LinearOpMode {

    public static double ONE_POS = 0;
    public static double ONE_MIN = 0;
    public static double ONE_MAX= 1;

    public static double TWO_POS = 1;
    public static double TWO_MIN = 0;
    public static double TWO_MAX = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);
        Servo one = robot.getGripperOne();
        Servo two = robot.getGripperTwo();

        waitForStart();

        while (opModeIsActive()) {

            one.scaleRange(ONE_MIN, ONE_MAX);
            two.scaleRange(TWO_MIN, TWO_MAX);
            one.setPosition(ONE_POS);
            two.setPosition(TWO_POS);

        }

    }

}
