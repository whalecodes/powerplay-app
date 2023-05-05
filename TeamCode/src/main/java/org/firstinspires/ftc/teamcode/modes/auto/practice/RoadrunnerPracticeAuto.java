package org.firstinspires.ftc.teamcode.modes.auto.practice;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.PlainMecanumDrive;

@Autonomous(name = "practice auto", group = "a")
public class RoadrunnerPracticeAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        PlainMecanumDrive drive = new PlainMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));



    }

}
