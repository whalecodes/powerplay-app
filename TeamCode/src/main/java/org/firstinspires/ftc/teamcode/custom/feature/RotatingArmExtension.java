package org.firstinspires.ftc.teamcode.custom.feature;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// doesn't work
public class RotatingArmExtension {

    private final Servo servo;

    public double target = 1;
    public double velocity = 0;

    public double position = 1;

    private long lastUpdate = System.currentTimeMillis();

    public RotatingArmExtension(Servo servo) {
        this.servo = servo;
    }

    public void rotateTo(int degrees, double seconds) {
        target = degrees/180D;
        velocity = (position - target)/(seconds*1000);
        lastUpdate = System.currentTimeMillis();
    }

    public void update() {

        if (position == target) return;

        double requiredChange = velocity * (System.currentTimeMillis()-lastUpdate);
        lastUpdate = System.currentTimeMillis();

        position += requiredChange;

        // prevent overshoot
        if ((velocity > 0 && position > target)
                || (velocity < 0 && position < target)) {
            position = target;
            velocity = 0;
        }
        servo.setPosition(position);

    }

}
