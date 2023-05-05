package org.firstinspires.ftc.teamcode.modes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.ParkingDetectionExtension;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

@Config("8 Parking Autonomous Settings")
@Autonomous(name = "[✔] Parking", group = "competition")
public class ParkingAutonomous extends LinearOpMode {

    public static int FORWARD = 26;
    public static int STRAFE = 28;

    @Override
    public void runOpMode() throws InterruptedException {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);
        PlainMecanumDrive drive = new PlainMecanumDrive(hardwareMap);

        ParkingDetectionExtension parkingDetection = new ParkingDetectionExtension(robot, hardwareMap, telemetry);

        Trajectory toCenter = drive.trajectoryBuilder(new Pose2d())
                .forward(FORWARD)
                .build();

        while (!isStarted()) {
            parkingDetection.init_loop();
        }

        ParkingDetectionExtension.ParkingPosition parkingPosition = parkingDetection.getParkingPosition();
        if (parkingPosition == null)
            parkingPosition = ParkingDetectionExtension.ParkingPosition.CENTER;

        robot.closeGripper();

        robot.setLinearSliderPower(-1);
        Thread.sleep(100);
        robot.setLinearSliderPower(0);

        drive.followTrajectory(toCenter);

        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.CENTER) return;

        TrajectoryBuilder toParkBuilder = drive.trajectoryBuilder(toCenter.end());

        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.LEFT)
            toParkBuilder.strafeLeft(STRAFE);
        else if (parkingPosition == ParkingDetectionExtension.ParkingPosition.RIGHT)
            toParkBuilder.strafeRight(STRAFE);

        drive.followTrajectory(toParkBuilder.build());

    }

}
