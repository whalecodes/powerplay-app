package org.firstinspires.ftc.teamcode.modes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;

@Autonomous(name = "[⚒] Motor Power Mode", group = "practice")
public class MotorPowerMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);

        waitForStart();

        robot.setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.setMotorPower(1);
        Thread.sleep(3000);
        robot.setMotorPower(0);

        telemetry.addData("left front", robot.getFrontLeftWheel().getCurrentPosition());
        telemetry.addData("left back", robot.getBackLeftWheel().getCurrentPosition());
        telemetry.addData("right front", robot.getFrontRightWheel().getCurrentPosition());
        telemetry.addData("right back", robot.getBackRightWheel().getCurrentPosition());

        double max = Math.max(robot.getFrontLeftWheel().getCurrentPosition(), robot.getBackLeftWheel().getCurrentPosition());
        max = Math.max(robot.getFrontRightWheel().getCurrentPosition(), max);
        max = Math.max(robot.getBackRightWheel().getCurrentPosition(), max);


        telemetry.addData("left front multiplier", robot.getFrontLeftWheel().getCurrentPosition()/max);
        telemetry.addData("left back multiplier", robot.getBackLeftWheel().getCurrentPosition()/max);
        telemetry.addData("right front multiplier", robot.getFrontRightWheel().getCurrentPosition()/max);
        telemetry.addData("right back multiplier", robot.getBackRightWheel().getCurrentPosition()/max);
        telemetry.update();

    }

}
