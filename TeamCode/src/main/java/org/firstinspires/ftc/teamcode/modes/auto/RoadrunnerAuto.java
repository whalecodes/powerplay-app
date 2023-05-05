package org.firstinspires.ftc.teamcode.modes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension;
import org.firstinspires.ftc.teamcode.custom.feature.ParkingDetectionExtension;
import org.firstinspires.ftc.teamcode.roadrunner.EnhancedMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequence;

public abstract class RoadrunnerAuto extends LinearOpMode {

    protected PowerPlayRobot robot;
    protected LinearSliderExtension slider;
    protected ParkingDetectionExtension vision;
    protected EnhancedMecanumDrive drive;

    @Override
    public void runOpMode() {

        PhotonCore.enable();

        robot = new PowerPlayRobot(hardwareMap);
        slider = new LinearSliderExtension(robot.getLinearSlider());
        vision = new ParkingDetectionExtension(robot, hardwareMap, telemetry);

        drive = new EnhancedMecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose());

        TrajectorySequence path = onStart();

        while (!isStarted()) {
            vision.init_loop();
        }

        vision.stop();

        ParkingDetectionExtension.ParkingPosition parkingPosition = vision.getParkingPosition();

        telemetry.addLine("Beginning the sequence...");
        telemetry.update();

        long start = System.currentTimeMillis();

        robot.rotateArmToDegree(179);
        drive.followTrajectorySequenceAsync(path);

        long totalLoop = 0;
        long totalLoops = 0;

        while (opModeIsActive() && drive.isBusy()) {
            long driveLoopStart = System.currentTimeMillis();
            drive.update();
            long driveLoopEnd = System.currentTimeMillis();
            long sliderLoopStart = System.currentTimeMillis();
            slider.update();
            long sliderLoopEnd = System.currentTimeMillis();
            //Pose2d loc = drive.getPoseEstimate();
            //telemetry.addLine("Location: " + loc.getX() + ", " + loc.getY() + ", " + Math.toDegrees(loc.getHeading()));
            telemetry.addData("drive loop", driveLoopEnd - driveLoopStart);
            telemetry.addData("slider loop", sliderLoopEnd - sliderLoopStart);

            totalLoop += driveLoopEnd - driveLoopStart + sliderLoopEnd - sliderLoopStart;
            totalLoops ++;

            double avgLoop = totalLoop / (double) totalLoops;

            telemetry.addData("avg loop", avgLoop);

            telemetry.update();
        }

        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.LEFT) drive.followTrajectorySequence(onParkLeft());
        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.CENTER || parkingPosition == null) drive.followTrajectorySequenceAsync(onParkCenter());
        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.RIGHT) drive.followTrajectorySequenceAsync(onParkRight());

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            slider.update();
        }

        long end = System.currentTimeMillis();
        long difference = end - start;
        long duration = difference/1000;

        telemetry.addLine("Completed the sequence in " + duration + " seconds.");
        telemetry.update();

    }

    public abstract TrajectorySequence onStart();

    public abstract Pose2d getStartPose();

    public abstract TrajectorySequence onParkLeft();

    public abstract TrajectorySequence onParkCenter();

    public abstract TrajectorySequence onParkRight();

}
