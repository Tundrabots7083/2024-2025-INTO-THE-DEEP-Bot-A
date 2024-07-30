package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

/**
 * Action that waits for the specified amount of time before completing execution.
 */
public class WaitAction extends ActionExBase {
    private final long millis;
    private boolean initialized = false;
    private long waitTime;

    /**
     * Instantiates an action to wait for the specified number of milliseconds before it finishes
     * running.
     *
     * @param millis the number of milliseconds to run for
     */
    public WaitAction(long millis) {
        this.millis = millis;
    }

    /**
     * Runs the action for the requested number of milliseconds.
     *
     * @param telemetryPacket the telemetry to use for output
     * @return <code>false</code> once the action has run for the specified number of milliseconds;
     * <code>true</code> if the action has not been running for the specified number of milliseconds
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            waitTime = System.currentTimeMillis() + millis;
            initialized = true;
        }
        return waitTime < System.currentTimeMillis();
    }
}
