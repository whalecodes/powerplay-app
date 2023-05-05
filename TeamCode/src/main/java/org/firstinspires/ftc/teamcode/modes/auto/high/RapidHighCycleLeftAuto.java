package org.firstinspires.ftc.teamcode.modes.auto.high;

import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FIVE_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FOUR_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.GROUND;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.HIGH;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.THREE_CONE_STACK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PowerPlayRobot;
import org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension;
import org.firstinspires.ftc.teamcode.custom.feature.ParkingDetectionExtension;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstant;
import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequence;

@Config("3 1+3 High Cycle Left")
@Autonomous(name = "[✔] 1+3 High Cycle Left", group = "competition")
public class RapidHighCycleLeftAuto extends LinearOpMode {

    public static double CONE_DROP_DELAY = 0.9;
    public static double CONE_PICKUP_ACTION_DELAY = 0.25;
    public static double ROTATE_ARM_TO_CONE_DELAY = 0.15;

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
                .waitSeconds(0.2)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getCenterVec())
                .turn(Math.toRadians(90))
                .lineTo(getDepositVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    slider.setTarget(HIGH);
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(FIVE_CONE_STACK);
                })

                // cycle 1
                .lineTo(getConeStackVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    slider.setTarget(HIGH);
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(FOUR_CONE_STACK);
                })

                // cycle 2
                .lineTo(getConeStackVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    slider.setTarget(HIGH);
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(THREE_CONE_STACK);
                })

                // cycle 3
                .lineTo(getConeStackVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(HIGH))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec(),
                        PlainMecanumDrive.getVelocityConstraint(DriveConstant.MAX_VEL, DriveConstant.MAX_ANG_VEL, DriveConstant.TRACK_WIDTH),
                        PlainMecanumDrive.getAccelerationConstraint(DriveConstant.MAX_ACCEL))
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    slider.setTarget(HIGH);
                })
                .waitSeconds(0.15)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })

                .addTemporalMarker(()-> {
                    slider.setTarget(GROUND);
                })

                .build();

        Trajectory toLeft = drive.trajectoryBuilder(path.end()).lineTo(getLeftParkVec()).build();
        Trajectory toMiddle = drive.trajectoryBuilder(path.end()).lineTo(getMiddleParkVec()).build();
        Trajectory toRight = drive.trajectoryBuilder(path.end()).lineTo(getRightParkVec()).build();


        while (!isStarted()) {
            vision.init_loop();
        }

        vision.stop();

        ParkingDetectionExtension.ParkingPosition parkingPosition = vision.getParkingPosition();

        telemetry.addLine("Beginning the sequence...");
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
            telemetry.addData("time since last loop", System.currentTimeMillis() - time);
            time = System.currentTimeMillis();
            telemetry.update();
        }

        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.LEFT) drive.followTrajectoryAsync(toLeft);
        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.CENTER || parkingPosition == null) drive.followTrajectoryAsync(toMiddle);
        if (parkingPosition == ParkingDetectionExtension.ParkingPosition.RIGHT) drive.followTrajectoryAsync(toRight);

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

    public static double START_X = 36;
    public static double START_Y = -62;

    public static double CENTER_X = 36;
    public static double CENTER_Y = -13;

    public static double DEPOSIT_X = 46;
    public static double DEPOSIT_Y = -9.5;

    public static double CONE_STACK_X = 10;
    public static double CONE_STACK_Y = -11;

    public static double LEFT_PARK_X = 13;
    public static double CENTER_PARK_X = 36;
    public static double RIGHT_PARK_X = 56.5;

    public static double PARK_Y = -13;

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

    public static Vector2d getMiddleParkVec() {
        return new Vector2d(CENTER_PARK_X, PARK_Y);
    }

    public static Vector2d getRightParkVec() {
        return new Vector2d(RIGHT_PARK_X, PARK_Y);
    }

    public static Vector2d getDepositVec() {
        return new Vector2d(DEPOSIT_X, DEPOSIT_Y);
    }

}
