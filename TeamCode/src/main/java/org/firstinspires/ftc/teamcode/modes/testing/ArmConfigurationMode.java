package org.firstinspires.ftc.teamcode.modes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension;

@Config("Linear Slider Configuration")
@TeleOp(name = "[⚒] Arm Height Configuration", group = "practice")
public class ArmConfigurationMode extends OpMode {

    public static int target = 0;

    private LinearSliderExtension slider;

    @Override
    public void init() {
        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);
        slider = new LinearSliderExtension(robot.getLinearSlider());
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        slider.setTarget(target);
        slider.update();

        int position = slider.getPosition();

        telemetry.addData("target", target);
        telemetry.addData("position", position);
        telemetry.addData("error", target - position);
        telemetry.addData("power", slider.power);
    }

}
