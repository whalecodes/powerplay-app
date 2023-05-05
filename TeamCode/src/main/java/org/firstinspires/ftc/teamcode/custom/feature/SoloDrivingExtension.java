package org.firstinspires.ftc.teamcode.custom.feature;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

/**
 * Effectively a TeleOp split into a separate class so that it can be reused without copy-pasting code.
 */
@Config("Solo-Driving Extension")
public class SoloDrivingExtension {

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_ROTATION = 90;

    private final PowerPlayRobot robot;
    private final PlainMecanumDrive drive;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    private final LinearSliderExtension slider;

    public SoloDrivingExtension(PowerPlayRobot robot, PlainMecanumDrive drive, Gamepad gamepad, Telemetry telemetry) {
        this.robot = robot;
        this.drive = drive;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.slider = new LinearSliderExtension(robot.getLinearSlider());

        this.drive.setPoseEstimate(new Pose2d(START_X, START_Y, Math.toRadians(START_ROTATION)));

    }

    /*
     * Control Manual for One Driver
     *
     * Left Stick — driving (up and down) and strafing (left and right)
     * Right Stick — turning (left and right)
     *
     * Left Button — release gripper
     * Right Button — close gripper
     *
     * Right Trigger — lower arm
     * Left Trigger — raise arm
     */

    public void loop() {

        // driving

        drive.setWeightedDrivePower(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        );

        drive.update();

        Pose2d loc = drive.getPoseEstimate();
        telemetry.addLine("loc: " + loc.getX() + ", " + loc.getY() + ", " + Math.toDegrees(loc.getHeading()));

        // gripper

        boolean closeGripper = gamepad.b;
        boolean openGripper = gamepad.x;

        if (closeGripper)
            robot.closeGripper();
        else if (openGripper)
            robot.openGripper();

        // linearSlider

        int raiseArm = (int) (gamepad.right_trigger * 80);
        int lowerArm = (int) (gamepad.left_trigger * 80);
        int linearSliderTargetOffset = raiseArm - lowerArm;

        slider.setTarget(slider.getTarget() + linearSliderTargetOffset);

        telemetry.addLine("left: " + raiseArm+ ", right: " + lowerArm);

        slider.update();

        telemetry.addLine("linear slider offset: " + linearSliderTargetOffset);
        telemetry.addLine("linear slider target: " + slider.getTarget());
        telemetry.addLine("linear slider position: " + slider.getPosition());

        // arm rotation

        boolean rotateArmFront = gamepad.y;
        boolean rotateArmBack = gamepad.a;

        if (rotateArmFront) robot.rotateArmToDegree(180);
        if (rotateArmBack) robot.rotateArmToDegree(0);

        telemetry.addLine("arm position: " + robot.getArmPosition());

    }

}
