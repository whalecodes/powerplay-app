package org.firstinspires.ftc.teamcode.roadrunner;

import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.kA;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.kStatic;
import static org.firstinspires.ftc.teamcode.roadrunner.DriveConstant.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.custom.object.EnhancedMotor;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.roadrunner.trajectory.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.roadrunner.utilities.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Roadrunner's Mecanum Drive with performance improvements
@Config("1 Enhanced Mecanum")
public class EnhancedMecanumDrive extends MecanumDrive {

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(5, 0, 0.5);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(5, 0, 0.5);

    public static final double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    private final TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private final List<DcMotorEx> motors;

    private final IMU imu;
    private final VoltageSensor batteryVoltageSensor;

    public EnhancedMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        TrajectoryFollower follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.2, 0.2, Math.toRadians(3)), 0.75);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstant.LOGO_FACING_DIR, DriveConstant.USB_FACING_DIR));
        imu.initialize(parameters);

        leftFront = EnhancedMotor.from(hardwareMap, "FL");
        leftRear = EnhancedMotor.from(hardwareMap,"BL");
        rightRear = EnhancedMotor.from(hardwareMap,"BR");
        rightFront = EnhancedMotor.from(hardwareMap,"FR");

        /*
        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");
         */

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor
        );

    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    // new
    public void setWeightedDrivePower(double x, double y, double heading) {
        Pose2d vel = new Pose2d(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(x)
                    + VY_WEIGHT * Math.abs(y)
                    + OMEGA_WEIGHT * Math.abs(heading);

            vel = new Pose2d(
                    VX_WEIGHT * x,
                    VY_WEIGHT * y,
                    OMEGA_WEIGHT * heading
            ).div(denom);
        }

        setDrivePower(vel);
    }

    public void setRobotCentricDrivePower(double x, double y, double heading) {

        double leftFrontPower = y + x + heading;
        double leftRearPower = y - x + heading;
        double rightFrontPower = y - x - heading;
        double rightRearPower = y + x - heading;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(heading), 1);

        leftFrontPower /= max;
        leftRearPower /= max;
        rightFrontPower /= max;
        rightRearPower /= max;


        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

    }

    public void setFieldCentricDrivePower(double x, double y, double heading) {

        double botHeading = getPoseEstimate().getHeading();

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(heading), 1);

        double leftFrontPower = (rotY + rotX + heading) / denominator;
        double leftRearPower = (rotY - rotX + heading) / denominator;
        double rightFrontPower = (rotY - rotX - heading) / denominator;
        double rightRearPower = (rotY + rotX - heading) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

    }

    private final List<Double> wheelPositions = new ArrayList<>(4);

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        wheelPositions.clear();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    private final List<Double> wheelVelocities = new ArrayList<>(4);

    @Override
    public List<Double> getWheelVelocities() {
        wheelVelocities.clear();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

}