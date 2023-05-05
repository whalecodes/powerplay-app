package org.firstinspires.ftc.teamcode.custom.feature;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.filter.KalmanFilter;

/**
 * DcMotor wrapper which runs to an encoder position using a PIDF controller.
 */
@Config("Linear Slider PIDF")
public class LinearSliderExtension {

    public enum ArmHeight {

        GROUND(50),
        LOW(2000),
        MEDIUM(3200),
        HIGH(4300),

        FIVE_CONE_STACK(600),
        FOUR_CONE_STACK(400),
        THREE_CONE_STACK(250),
        TWO_CONE_STACK(100);

        private final int target;

        ArmHeight(int target) {
            this.target = target;
        }

        public int getTarget() {
            return target;
        }

        public static final ArmHeight[] POLES = {
                GROUND, LOW, MEDIUM, HIGH
        };

    }

    public static double kP = 0.01,
            kI = 0,
            kD = 0.0003,
            kF = 0.1;

    private final PIDController controller;
    private DcMotor motor;

    public double power = 0;
    public int position = 0;
    private int target = 0;

    public LinearSliderExtension(DcMotor motor) {
        this.controller = new PIDController(kP, kI, kD);
        this.motor = motor;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        controller.setPID(kP, kI, kD);

        position = motor.getCurrentPosition();
        power = controller.calculate(position, target);

        motor.setPower(power + kF);
    }

    public void setTarget(ArmHeight height) {
        setTarget(height.getTarget());
    }

    public void setTarget(int target) {
        if (target <= -4500) return;
        if (target >= 9000) return;

        this.target = target;
    }

    public void raise(int amount) {
        target += amount;
    }

    public void lower(int amount) {
        target -= amount;
    }

    public int getTarget() {
        return target;
    }

    public int getPosition() {
        return position;
    }

    public void setMotor(DcMotor motor) {
        this.motor = motor;
    }

}
