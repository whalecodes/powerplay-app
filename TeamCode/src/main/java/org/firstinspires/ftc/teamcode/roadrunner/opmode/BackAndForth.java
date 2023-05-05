package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.EnhancedMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(name = "[♻] Back and Forth", group = "roadrunner")
public class BackAndForth extends LinearOpMode {

    public static double DISTANCE = 50;
    public static boolean DO_ENHANCED = false;

    @Override
    public void runOpMode() throws InterruptedException {

        PhotonCore.enable();

        PlainMecanumDrive plainDrive = new PlainMecanumDrive(hardwareMap);
        EnhancedMecanumDrive enhancedDrive = new EnhancedMecanumDrive(hardwareMap);

        Trajectory trajectoryForwardPlain = plainDrive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackwardPlain = plainDrive.trajectoryBuilder(trajectoryForwardPlain.end())
                .back(DISTANCE)
                .build();

        Trajectory trajectoryForwardEnhanced = enhancedDrive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        Trajectory trajectoryBackwardEnhanced = enhancedDrive.trajectoryBuilder(trajectoryForwardPlain.end())
                .back(DISTANCE)
                .build();

        waitForStart();

        if (DO_ENHANCED) {

            while (opModeIsActive()) {

                long totalTime = 0;
                long totalHz = 0;
                long totalLoops = 0;
                enhancedDrive.followTrajectoryAsync(trajectoryForwardPlain);
                while (enhancedDrive.isBusy()) {
                    long start = System.currentTimeMillis();
                    enhancedDrive.update();
                    long end = System.currentTimeMillis();
                    long loopTime = end - start;
                    totalLoops += loopTime;
                    double hz = 1000 / (double) loopTime;
                    totalHz += hz;
                    telemetry.addData("loop time", loopTime);
                    telemetry.addData("avg loop", (double) totalTime/totalLoops);
                    telemetry.addData("hz", hz);
                    telemetry.addData("avg hz", (double) totalHz/totalLoops);
                    telemetry.update();
                }
                enhancedDrive.followTrajectoryAsync(trajectoryBackwardPlain);
                while (enhancedDrive.isBusy()) {
                    long start = System.currentTimeMillis();
                    enhancedDrive.update();
                    long end = System.currentTimeMillis();
                    long loopTime = end - start;
                    totalLoops += loopTime;
                    long hz = 1000/loopTime;
                    totalHz += hz;
                    telemetry.addData("loop time", loopTime);
                    telemetry.addData("avg loop", (double) totalTime/totalLoops);
                    telemetry.addData("hz", hz);
                    telemetry.addData("avg hz", (double) totalHz/totalLoops);
                    telemetry.update();
                }

            }

        } else {

            while (opModeIsActive()) {

                long totalTime = 0;
                long totalHz = 0;
                long totalLoops = 0;
                plainDrive.followTrajectoryAsync(trajectoryForwardEnhanced);
                while (plainDrive.isBusy()) {
                    long start = System.currentTimeMillis();
                    plainDrive.update();
                    long end = System.currentTimeMillis();
                    long loopTime = end - start;
                    totalLoops += loopTime;
                    long hz = 1000/loopTime;
                    totalHz += hz;
                    telemetry.addData("loop time", loopTime);
                    telemetry.addData("avg loop time", (double) totalTime/totalLoops);
                    telemetry.addData("hz", hz);
                    telemetry.addData("avg hz", (double) totalHz/totalLoops);
                    telemetry.update();
                }
                plainDrive.followTrajectoryAsync(trajectoryBackwardEnhanced);
                while (plainDrive.isBusy()) {
                    long start = System.currentTimeMillis();
                    plainDrive.update();
                    long end = System.currentTimeMillis();
                    long loopTime = end - start;
                    totalLoops += loopTime;
                    long hz = 1000/loopTime;
                    totalHz += hz;
                    telemetry.addData("loop time", loopTime);
                    telemetry.addData("avg loop time", (double) totalTime/totalLoops);
                    telemetry.addData("hz", hz);
                    telemetry.addData("avg hz", (double) totalHz/totalLoops);
                    telemetry.update();
                }

            }

        }

    }
}