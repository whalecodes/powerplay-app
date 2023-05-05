package org.firstinspires.ftc.teamcode.custom.feature;

import android.sax.StartElementListener;

import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Just writes the current voltage to telemetry.
 */
public class VoltageTrackingExtension {

    private final VoltageSensor sensor;
    private final Telemetry telemetry;

    public VoltageTrackingExtension(Telemetry telemetry, HardwareMap map) {
        sensor = map.voltageSensor.iterator().next();
        this.telemetry = telemetry;
    }

    public void update() {
        telemetry.addData("voltage", sensor.getVoltage());
    }

}
