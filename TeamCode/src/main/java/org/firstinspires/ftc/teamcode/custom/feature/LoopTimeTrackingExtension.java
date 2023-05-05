package org.firstinspires.ftc.teamcode.custom.feature;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LoopTimeTrackingExtension {

    private final Telemetry telemetry;

    long lastLoop = 0;
    long totalLoopTime = 0;
    long totalLoops = 0;

    public LoopTimeTrackingExtension(Telemetry telemetry) {
        this.telemetry = telemetry;
        lastLoop = System.currentTimeMillis();
    }

    public void loop() {

        long now = System.currentTimeMillis();
        long time = now - lastLoop;

        telemetry.addData("last loop", time);
        totalLoops ++;
        totalLoopTime += time;
        telemetry.addData("avg loop",((double) totalLoopTime)/totalLoops);

    }

}
