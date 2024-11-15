package org.firstinspires.ftc.teamcode.ftc7083.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.Arrays;

/**
 * Runs an array of actions in parallel. The action completes running only when all actions have
 * completed.
 */
public class ParallelAction extends ActionExBase {
    private final Action[] actions;
    private final boolean[] actionsRunning;

    /**
     * Instantiates a ParallelAction that runs a set of actions in parallel.
     *
     * @param actions the actions to run in parallel
     */
    public ParallelAction(Action... actions) {
        this.actions = actions;
        actionsRunning = new boolean[actions.length];
        Arrays.fill(actionsRunning, true);
    }

    /**
     * Runs each action in the set in parallel until every action has finished running.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if all actions have finished running; <code>true</code> if any
     *         of the actions are still running.
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean isRunning = false;
        for (int i = 0; i < actionsRunning.length; i++) {
            if (actionsRunning[i]) {
                actionsRunning[i]  = actions[i].run(telemetryPacket);
                if (actionsRunning[i]) {
                    isRunning = true;
                }
            }
        }

        return isRunning;
    }
}
