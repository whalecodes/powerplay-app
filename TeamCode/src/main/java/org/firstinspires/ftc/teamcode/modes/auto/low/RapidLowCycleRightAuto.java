package org.firstinspires.ftc.teamcode.modes.auto.low;

import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FIVE_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.FOUR_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.LOW;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.THREE_CONE_STACK;
import static org.firstinspires.ftc.teamcode.custom.feature.LinearSliderExtension.ArmHeight.TWO_CONE_STACK;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.modes.auto.RoadrunnerAuto;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequence;

@Config("3 1+5 Low Cycle Right")
@Autonomous(name = "[✔] 1+5 Low Cycle Right", group = "competition")
public class RapidLowCycleRightAuto extends RoadrunnerAuto {

    public static double CONE_DROP_DELAY = 0.1;
    public static double CONE_PICKUP_ACTION_DELAY = 0.15;
    public static double ROTATE_ARM_TO_CONE_DELAY = 0.1;

    public static double START_X = 36;
    public static double START_Y = -62;

    public static double CENTER_X = 35;
    public static double CENTER_Y = -15;

    public static double DEPOSIT_X = 45.5;
    public static double DEPOSIT_Y = -13.65;

    public static double CONE_STACK_X = 59;
    public static double CONE_STACK_Y = -10.5;

    public static double LEFT_PARK_X = 13;
    public static double CENTER_PARK_X = 36;
    public static double RIGHT_PARK_X = 56.5;

    public static double PARK_Y = -13;

    private TrajectorySequence path;

    @Override
    public TrajectorySequence onStart() {
        this.path = drive.trajectorySequenceBuilder(getStartPose())
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                    robot.closeGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .lineTo(getCenterVec())
                .turn(Math.toRadians(-90))
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> robot.rotateArmToDegree(180))
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> slider.setTarget(FIVE_CONE_STACK))

                // cycle 1
                .lineTo(getConeStackVec())
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(FOUR_CONE_STACK);
                })

                // cycle 2
                .lineTo(getConeStackVec())
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(THREE_CONE_STACK);
                })

                // cycle 3
                .lineTo(getConeStackVec())
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(TWO_CONE_STACK);
                })

                // cycle 4
                .lineTo(getConeStackVec())
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                })
                // cycle 5
                .lineTo(getConeStackVec())
                .addTemporalMarker(robot::closeGripper)
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(CONE_PICKUP_ACTION_DELAY)
                .addTemporalMarker(()-> robot.rotateArmToDegree(90))
                .lineTo(getDepositVec())
                .waitSeconds(CONE_DROP_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                    robot.openGripper();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(()-> slider.setTarget(LOW))
                .waitSeconds(0.1)
                .addTemporalMarker(()-> {
                    robot.rotateArmToDegree(180);
                })
                .waitSeconds(ROTATE_ARM_TO_CONE_DELAY)
                .addTemporalMarker(()-> {
                    slider.setTarget(0);
                })

                .build();
        return path;
    }

    public Pose2d getStartPose() {
        return new Pose2d(START_X, START_Y, Math.toRadians(90));
    }

    @Override
    public TrajectorySequence onParkLeft() {
        return drive.trajectorySequenceBuilder(path.end()).lineTo(getLeftParkVec()).build();
    }

    @Override
    public TrajectorySequence onParkCenter() {
        return drive.trajectorySequenceBuilder(path.end()).lineTo(getMiddleParkVec()).build();
    }

    @Override
    public TrajectorySequence onParkRight() {
        return drive.trajectorySequenceBuilder(path.end()).lineTo(getRightParkVec()).build();
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
