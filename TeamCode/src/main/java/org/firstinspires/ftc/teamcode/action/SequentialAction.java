package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/**
 * Runs the set of actions sequentially. This action only completes when the last action finishes
 * running.
 */
public class SequentialAction extends ActionExBase {
    private final Action[] actions;
    private int currentAction = 0;

    /**
     * Instantiates an action that runs the set of actions sequential.
     *
     * @param actions the set of actions to run sequentially.
     */
    public SequentialAction(Action... actions) {
        this.actions = actions;
    }

    /**
     * Runs the actions sequentially, only completing when every action in the set has completed
     * running.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if all actions have completed running; <code>true</code> if any
     * action is still running.
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean isActionRunning = false;
        if (currentAction < actions.length) {
            isActionRunning = actions[currentAction].run(telemetryPacket);
            if (!isActionRunning) {
                currentAction++;
            }
        }

        return isActionRunning;
    }
}
