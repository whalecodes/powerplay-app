package org.firstinspires.ftc.teamcode.modes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.apache.commons.math3.analysis.function.Power;
import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.object.EnhancedMotor;
import org.firstinspires.ftc.teamcode.roadrunner.EnhancedMecanumDrive;

@TeleOp(name = "[⚒] Swappable Drive Mode", group = "practice")
public class SwappableDriveMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        
        EnhancedMecanumDrive drive = new EnhancedMecanumDrive(hardwareMap);

        boolean prevState = false;
        boolean isRobotCentric = false;

        waitForStart();

        while (opModeIsActive()) {

            boolean currState = gamepad1.left_bumper;
            if (currState && !prevState) {
                isRobotCentric = !isRobotCentric;
            }
            prevState = currState;

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double heading = gamepad2.right_stick_x;

            if (isRobotCentric) {
                drive.setRobotCentricDrivePower(x, y, heading);
            } else {
                drive.setFieldCentricDrivePower(x, y, heading);
            }

            telemetry.addData("is robot centric?", isRobotCentric);
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading", heading);
            telemetry.update();

        }

    }

}
