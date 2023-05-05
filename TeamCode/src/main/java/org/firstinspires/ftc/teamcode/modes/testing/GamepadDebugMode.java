package org.firstinspires.ftc.teamcode.modes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "[⚒] Gamepad Debug Mode", group = "practice")
public class GamepadDebugMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad gamepad = gamepad1;

        telemetry.addLine("This opmode does not interact with robot hardware.");
        telemetry.addLine("Instead, it uses telemetry to show what buttons are being activated.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.clear();

            telemetry.addData("x", gamepad.x);
            telemetry.addData("y", gamepad.y);
            telemetry.addData("b", gamepad.b);
            telemetry.addData("a", gamepad.a);

            telemetry.addData("left stick button", gamepad.left_stick_button);
            telemetry.addData("right stick button", gamepad.right_stick_button);
            telemetry.addData("left stick x", gamepad.left_stick_x);
            telemetry.addData("left stick y", gamepad.left_stick_y);
            telemetry.addData("right stick x", gamepad.right_stick_x);
            telemetry.addData("right stick y", gamepad.right_stick_y);

            telemetry.addData("left bumper", gamepad.left_bumper);
            telemetry.addData("right bumper", gamepad.right_bumper);
            telemetry.addData("left trigger", gamepad.left_trigger);
            telemetry.addData("right trigger", gamepad.right_trigger);

            telemetry.addData("dpad left", gamepad.dpad_left);
            telemetry.addData("dpad right", gamepad.dpad_right);
            telemetry.addData("dpad up", gamepad.dpad_up);
            telemetry.addData("dpad down", gamepad.dpad_down);

            telemetry.update();

        }

    }

}
