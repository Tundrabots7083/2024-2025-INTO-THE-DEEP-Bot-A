package org.firstinspires.ftc.teamcode.ftc7083.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Action that waits for the specified amount of time before completing execution.
 */
public class WaitAction extends ActionExBase {
    private final long millis;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean initialized = false;

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
     *         <code>true</code> if the action has not been running for the specified number of milliseconds
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            timer.reset();
            initialized = true;
        }
        return timer.time() < millis;
    }
}
