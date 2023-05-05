package org.firstinspires.ftc.teamcode.modes.auto.archive;

import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FIVE_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FOUR_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.GROUND;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.HIGH;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.MEDIUM;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.THREE_CONE_STACK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension;
import org.firstinspires.ftc.teamcode.custom.feature.ParkingDetectionExtension;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequence;

@Disabled
@Config("1+3 Turning High Cycle Right")
@Autonomous(name = "[✘] 1+3 Turning High Cycle Right", group = "competition")
public class TurningHighCycleRightAutonomous extends LinearOpMode {

    public static double CONE_DROP_DELAY = 0.25;
    public static double CONE_PICKUP_ACTION_DELAY = 0.15;

    @Override
    public void runOpMode() {

        PowerPlayRobot robot = new PowerPlayRobot(hardwareMap);
        LinearSliderExtension slider = new LinearSliderExtension(robot.getLinearSlider());
        ParkingDetectionExtension vision = new ParkingDetectionExtension(robot, hardwareMap, telemetry);

        PlainMecanumDrive drive = new PlainMecanumDrive(hardwareMap);
        drive.setPoseEstimate(getStartPose());

        telemetry.addLine("Creating cone scoring trajectory, hold tight...");
        telemetry.update();

        TrajectorySequence path = drive.trajectorySequenceBuilder(getStartPose())
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                    robot.closeGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    slider.setTarget(HIGH);
                })

                .lineTo(getCenterVec())
                .turn(Math.toRadians(45))
                .forward(POLE_NUDGE_DISTANCE)
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(MEDIUM);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .back(POLE_NUDGE_DISTANCE)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(0);
                })
                .turn(Math.toRadians(45))

                // start collect cycle 1
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(0);
                    slider.setTarget(FIVE_CONE_STACK);
                })
                .lineTo(getConeStackVec())
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(180))
                .lineTo(getCenterVec())
                // end collect cycle 1

                // start deposit cycle  1
                .turn(Math.toRadians(-45))
                .forward(POLE_NUDGE_DISTANCE)
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(FOUR_CONE_STACK);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(robot::openGripper)
                .waitSeconds(0.1)
                .back(POLE_NUDGE_DISTANCE)
                .addTemporalMarker(()-> robot.rotateArmToDegree(0))
                .turn(Math.toRadians(45))
                // end deposit cycle 1

                // start collect cycle 2
                .lineTo(getConeStackVec())
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(180))
                .lineTo(getCenterVec())
                // end collect cycle 2

                // start deposit cycle  2
                .turn(Math.toRadians(-45))
                .forward(POLE_NUDGE_DISTANCE)
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(THREE_CONE_STACK);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(robot::openGripper)
                .waitSeconds(0.1)
                .back(POLE_NUDGE_DISTANCE)
                .addTemporalMarker(()-> robot.rotateArmToDegree(0))
                .turn(Math.toRadians(45))
                // end deposit cycle 2

                // start collect cycle 3
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(0);
                    slider.setTarget(FOUR_CONE_STACK);
                })
                .lineTo(getConeStackVec())
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(MEDIUM))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(180))
                .lineTo(getCenterVec())
                // end collect cycle 3

                // start deposit cycle  3
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .turn(Math.toRadians(-45))
                .forward(POLE_NUDGE_DISTANCE)
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(FOUR_CONE_STACK);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(robot::openGripper)
                .waitSeconds(0.1)
                .back(POLE_NUDGE_DISTANCE)
                .addTemporalMarker(()-> robot.rotateArmToDegree(0))
                .turn(Math.toRadians(45))
                // end deposit cycle 3

                .addTemporalMarker(()-> slider.setTarget(GROUND))

                .build();

        while (!isStarted()) {
            vision.init_loop();
        }

        telemetry.addLine("Creating parking trajectory, hold tight...");
        telemetry.update();

        long start = System.currentTimeMillis();

        robot.rotateArmToDegree(178);
        drive.followTrajectorySequenceAsync(path);

        long time = System.currentTimeMillis();

        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            slider.update();
            Pose2d loc = drive.getPoseEstimate();
            telemetry.addLine("Location: " + loc.getX() + ", " + loc.getY() + ", " + Math.toDegrees(loc.getHeading()));
            telemetry.addLine(System.currentTimeMillis() - time + " ms since last loop.");
            telemetry.update();
        }

        while (opModeIsActive()) {
            drive.update();
            slider.update();
        }

        long end = System.currentTimeMillis();
        long difference = end - start;
        long duration = difference/1000;

        telemetry.addLine("Completed the sequence in " + duration + " seconds.");
        telemetry.update();

    }

    public static double START_X = 36;
    public static double START_Y = -62;

    public static double CENTER_X = 35;
    public static double CENTER_Y = -15;

    public static double CONE_STACK_X = 57.5;
    public static double CONE_STACK_Y = -12;

    public static double LEFT_PARK_X = 13;
    public static double CENTER_PARK_X = 36;
    public static double RIGHT_PARK_X = 56.5;

    public static double PARK_Y = -13;

    public static double POLE_NUDGE_DISTANCE = 9.5;

    public static Pose2d getStartPose() {
        return new Pose2d(START_X, START_Y, Math.toRadians(90));
    }

    public static Vector2d getCenterVec() {
        return new Vector2d(CENTER_X, CENTER_Y);
    }

    public static Vector2d getConeStackVec() {
        return new Vector2d(CONE_STACK_X, CONE_STACK_Y);
    }

    public static Vector2d getLeftParkVec() {
        return new Vector2d(LEFT_PARK_X, PARK_Y);
    }

    public static Vector2d getCenterParkVec() {
        return new Vector2d(CENTER_PARK_X, PARK_Y);
    }

    public static Vector2d getRightParkVec() {
        return new Vector2d(RIGHT_PARK_X, PARK_Y);
    }

}
