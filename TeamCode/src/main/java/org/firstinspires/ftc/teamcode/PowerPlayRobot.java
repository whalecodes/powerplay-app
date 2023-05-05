package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// Robot Configuration & Hardware Accessor for 2022-2023 PowerPlay
public class PowerPlayRobot {

    // Hardware Names
    public final String WEBCAM_NAME = "Webcam";

    public final String FRONT_LEFT_WHEEL = "FL";
    public final String FRONT_RIGHT_WHEEL = "FR";
    public final String BACK_LEFT_WHEEL = "BL";
    public final String BACK_RIGHT_WHEEL = "BR";

    public final String LINEAR_SLIDER = "arm";
    public final String GRIPPER_ONE = "S1";
    public final String GRIPPER_TWO = "S2";
    public final String ARM_ROTATOR = "S4"; //

    // Hardware Definitions
    private final DcMotor frontLeftWheel;
    private final DcMotor frontRightWheel;
    private final DcMotor backLeftWheel;
    private final DcMotor backRightWheel;

    private final DcMotor linearSlider;
    private final Servo gripperOne;
    private final Servo gripperTwo;
    private final Servo armRotator;

    public PowerPlayRobot(HardwareMap map) {

        frontLeftWheel = map.dcMotor.get(FRONT_LEFT_WHEEL);
        frontRightWheel = map.dcMotor.get(FRONT_RIGHT_WHEEL);
        backLeftWheel = map.dcMotor.get(BACK_LEFT_WHEEL);
        backRightWheel = map.dcMotor.get(BACK_RIGHT_WHEEL);

        linearSlider = map.dcMotor.get(LINEAR_SLIDER);
        gripperOne = map.servo.get(GRIPPER_ONE);
        gripperTwo = map.servo.get(GRIPPER_TWO);
        armRotator = map.servo.get(ARM_ROTATOR);

        backLeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        gripperOne.scaleRange(0.1, 1);
        gripperTwo.scaleRange(0, 0.9);

    }

    public PowerPlayRobot setDefaults() {
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorZeroPowerMode(DcMotor.ZeroPowerBehavior.BRAKE);
        return this;
    }

    public DcMotor getFrontLeftWheel()  {
        return frontLeftWheel;
    }

    public DcMotor getFrontRightWheel() {
        return frontRightWheel;
    }

    public DcMotor getBackLeftWheel() {
        return backLeftWheel;
    }

    public DcMotor getBackRightWheel() {
        return backRightWheel;
    }

    public DcMotor getLinearSlider() {
        return linearSlider;
    }

    public Servo getArmRotator() {
        return armRotator;
    }

    public void setMotorRunMode(DcMotor.RunMode mode) {
        frontLeftWheel.setMode(mode);
        frontRightWheel.setMode(mode);
        backLeftWheel.setMode(mode);
        backRightWheel.setMode(mode);
    }

    public void setMotorZeroPowerMode(DcMotor.ZeroPowerBehavior mode) {
        frontLeftWheel.setZeroPowerBehavior(mode);
        frontRightWheel.setZeroPowerBehavior(mode);
        backLeftWheel.setZeroPowerBehavior(mode);
        backRightWheel.setZeroPowerBehavior(mode);
    }

    public void setMotorPower(double power) {
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    // arm position

    public void rotateArmToDegree(int degree) {
        if (degree < 0 || degree > 180) return;
        double rotation = degree/180D;
        setArmRotation(rotation);
    }

    public void setArmRotation(double rotation) {
        armRotator.setPosition(rotation);
    }

    public double getArmPosition() {
        return armRotator.getPosition();
    }

    public void setLinearSliderPower(double power) {
        linearSlider.setPower( (power == 0) ? 0.1 : power); // gravity prevention permanently built in
    }

    // motor control

    /**
     * Provide power to the robot so that it can strafe, drive forward, and turn all at once or separately.
     * This method was salvaged/refactored from previous years.
     *
     * positive = forward/right
     * negative = backward, left
     *
     * @param drive how quickly forward/backward the robot should drive.
     * @param strafe how quickly the robot should strafe, and whether it should do so left or right.
     * @param turn how quickly the robot should turn, and whether it should do so left or right.
     */
    public double[] setComplexDrivePower(double drive, double strafe, double turn) {

        // distance of joystick from center
        double drivePower = Math.hypot(strafe, drive) * (1.5);
        // angle of joystick from center
        double driveAngle = Math.atan2(drive, strafe) + Math.PI / 4;

        // sideways component of drive + strafe turn amount
        double frontLeftPower = drivePower * Math.cos(driveAngle) + turn;
        // forward component of drive + strafe turn
        double frontRightPower = drivePower * Math.sin(driveAngle) + turn;
        // forward component of drive - strafe turn
        double backLeftPower = drivePower * Math.sin(driveAngle) - turn;
        // strafe component of drive - strafe turn
        double backRightPower = drivePower * Math.cos(driveAngle) - turn;

        double[] powers = new double[4];
        powers[0] = frontRightPower;
        powers[1] = frontLeftPower;
        powers[2] = backLeftPower;
        powers[3] = backRightPower;

        if (abs(frontLeftPower) > 1 || abs(backLeftPower) > 1 || abs(frontRightPower) > 1 || abs(backRightPower) > 1 ) {
            // normalize max power to be 1, and all other powers to be proportional to it
            double max;
            max = Math.max(abs(frontLeftPower), abs(backLeftPower));
            max = Math.max(abs(frontRightPower), max);
            max = Math.max(abs(backRightPower), max);

            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftWheel.setPower(frontLeftPower);
        frontRightWheel.setPower(frontRightPower);
        backLeftWheel.setPower(backLeftPower);
        backRightWheel.setPower(backRightPower);

        return powers;
    }

    public void driveByPower(double power) {
        frontLeftWheel.setPower(power);
        frontRightWheel.setPower(power);
        backLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void strafeByPower(double power) {
        frontRightWheel.setPower(-power);
        backLeftWheel.setPower(-power);
        frontLeftWheel.setPower(power);
        backRightWheel.setPower(power);
    }

    public void turnByPower(double power) {
        frontRightWheel.setPower(power);
        backRightWheel.setPower(power);
        frontLeftWheel.setPower(-power);
        backLeftWheel.setPower(-power);
    }

    // gripper

    public Servo getGripperOne() {
        return gripperOne;
    }

    public Servo getGripperTwo() {
        return gripperTwo;
    }

    public void closeGripper() {
        gripperTwo.setPosition(0);
        gripperOne.setPosition(1);
    }

    public void openGripper() {
        gripperOne.setPosition(0);
        gripperTwo.setPosition(1);
    }

}
