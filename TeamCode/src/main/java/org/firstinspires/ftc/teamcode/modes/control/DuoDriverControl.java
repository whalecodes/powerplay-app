package org.firstinspires.ftc.teamcode.modes.control;

import static org.firstinspires.ftc.teamcode.custom.object.EnhancedGamepad.Button.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.object.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

@TeleOp(name = "[✔] Duo-Driver Mode", group = "competition")
public class DuoDriverControl extends OpMode {

    private PowerPlayRobot robot;

    private LinearSliderExtension slider;
    private PlainMecanumDrive drive;

    private EnhancedGamepad driveGamepad;
    private EnhancedGamepad armGamepad;

    @Override
    public void init() {

        robot = new PowerPlayRobot(hardwareMap)
                .setDefaults();
        drive = new PlainMecanumDrive(hardwareMap);
        slider = new LinearSliderExtension(robot.getLinearSlider());

        driveGamepad = new EnhancedGamepad(gamepad1)
                .onMoveAnyStick((xLeft, yLeft, xRight, yRight)-> {
                    drive.setWeightedDrivePower(-yLeft, -xLeft, -xRight);
                });

        armGamepad = new EnhancedGamepad(gamepad2)
                .onPress(KEY_UP, ()-> robot.rotateArmToDegree(180))
                .onPress(KEY_DOWN, ()-> robot.rotateArmToDegree(0))
                .onPress(KEY_LEFT, ()-> robot.closeGripper())
                .onPress(KEY_RIGHT, ()-> robot.openGripper())
                .onLeftStickMove((x, y)-> {
                    slider.raise((int)(75*y));
                });

    }

    @Override
    public void loop() {

        // driving



        // arm control

    }

}
