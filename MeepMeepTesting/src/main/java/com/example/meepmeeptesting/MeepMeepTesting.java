package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main2(String... args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity newCheck = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(getStartPose())
                        .splineTo(new Vector2d(33, -11.5), 90)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(newCheck)
                .start();

    }


    public static void main(String... args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity newCheck = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(getStartPose())
                        .forward(50)
                        .turn(Math.toRadians(45))
                        .forward(4)
                        .back(4)
                        .turn(Math.toRadians(-135))
                        .forward(20)
                        .back(20)
                        .turn(Math.toRadians(-45))
                        .back(4)
                        .forward(4)
                        .turn(Math.toRadians(45))
                        .build());

        RoadRunnerBotEntity onePlusFiveCycleRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(getStartPose())

                        .waitSeconds(0.1)

                        .lineTo(getCenterVec())
                        .turn(Math.toRadians(45))

                        .forward(POLE_NUDGE_DISTANCE)
                        .back(POLE_NUDGE_DISTANCE)

                        .turn(-Math.toRadians(135))
                        .lineTo(getConeStackPose())
                        .forward(STACK_NUDGE_DISTANCE)
                        .waitSeconds(1)
                        .back(STACK_NUDGE_DISTANCE)
                        .lineTo(getCenterVec())

                        .lineTo(getLeftParkPose())
                        .waitSeconds(0.2)
                        .lineTo(getCenterParkPose())
                        .waitSeconds(0.2)
                        .lineTo(getRightParkPose())

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(onePlusFiveCycleRight)
                .start();

    }

    public static int START_X = 36;
    public static int START_Y = -62;
    public static int START_DEG = 90;

    public static Pose2d getStartPose() {
        return new Pose2d(START_X, START_Y, Math.toRadians(START_DEG));
    }

    public static int CENTER_X = 35;
    public static int CENTER_Y = -12;

    public static Vector2d getCenterVec() {
        return new Vector2d(CENTER_X, CENTER_Y);
    }

    public static int CONE_STACK_X = 56;
    public static int CONE_STACK_Y = -12;

    public static Vector2d getConeStackPose() {
        return new Vector2d(CONE_STACK_X, CONE_STACK_Y);
    }

    public static int LEFT_PARK_X = 13;
    public static int CENTER_PARK_X = 36;
    public static int RIGHT_PARK_X = 57;

    public static int PARK_Y = -12;

    public static Vector2d getLeftParkPose() {
        return new Vector2d(LEFT_PARK_X, PARK_Y);
    }

    public static Vector2d getCenterParkPose() {
        return new Vector2d(CENTER_PARK_X, PARK_Y);
    }

    public static Vector2d getRightParkPose() {
        return new Vector2d(RIGHT_PARK_X, PARK_Y);
    }

    public static int POLE_NUDGE_DISTANCE = 5;
    public static int STACK_NUDGE_DISTANCE = 1;

    public static void mainOld(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Vector2d start = new Vector2d(36, -62);

        Vector2d center = new Vector2d(35, -12);
        Pose2d centerDeposit = new Pose2d(center, Math.toRadians(315));
        Pose2d depositReady = new Pose2d(23, -12, Math.toRadians(90));
        Pose2d collectReady = new Pose2d(56, -12, Math.toRadians(0));

        RoadRunnerBotEntity newCheck = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(start, Math.toRadians(90)))
                        .forward(50)
                        .turn(Math.toRadians(45))
                        .forward(4)
                        .back(4)
                        .turn(Math.toRadians(-135))
                        .forward(20)
                        .back(20)
                        .turn(Math.toRadians(-45))
                        .back(4)
                        .forward(4)
                        .turn(Math.toRadians(45))
                        .build());

        RoadRunnerBotEntity onePlusFiveCycleRight = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(new Pose2d(start, Math.toRadians(90)))
                        //.waitSeconds(3)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)

                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(0.5)
                        .back(1)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)


                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(0.5)
                        .back(1)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)


                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(0.5)
                        .back(1)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)


                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(0.5)
                        .back(1)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)


                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(0.5)
                        .back(1)
                        .lineToLinearHeading(centerDeposit)
                        .back(5)
                        .forward(5)

                        .lineToLinearHeading(new Pose2d(13, -12, Math.toRadians(90)))
                        //.waitSeconds(0.2)
                        //.lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(90)))
                        //.waitSeconds(0.2)
                        //.lineToLinearHeading(new Pose2d(57, -12, Math.toRadians(90)))

                        .build());

        RoadRunnerBotEntity onePlusThree = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(start, Math.toRadians(90)))
                                .lineTo(center)
                                .lineTo(depositReady.vec())
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)

                                .lineToLinearHeading(collectReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)
                                .lineToLinearHeading(depositReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)

                                .lineToLinearHeading(collectReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)
                                .lineToLinearHeading(depositReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)

                                .lineToLinearHeading(collectReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)
                                .lineToLinearHeading(depositReady)
                                .forward(1)
                                .waitSeconds(1)
                                .back(1)

                                .build()
                );


        RoadRunnerBotEntity onePlusThreeCycle = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13)
                .followTrajectorySequence(drive-> drive.trajectorySequenceBuilder(depositReady)

                        .lineToLinearHeading(collectReady)
                        .forward(1)
                        .waitSeconds(1)
                        .back(1)
                        .lineToLinearHeading(depositReady)
                        .forward(1)
                        .waitSeconds(1)
                        .back(1)

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(newCheck)
                .start();
    }
}