package org.firstinspires.ftc.teamcode.action;

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
    private boolean[] actionsFinished;

    /**
     * Instantiates a ParallelAction that runs a set of actions in parallel.
     *
     * @param actions the actions to run in parallel
     */
    public ParallelAction(Action... actions) {
        this.actions = actions;
        actionsFinished = new boolean[actions.length];
        Arrays.fill(actionsFinished, false);
    }

    /**
     * Runs each action in the set in parallel until every action has finished running.
     *
     * @param telemetryPacket the telemetry to use for output.
     * @return <code>false</code> if all actions have finished running; <code>true</code> if any
     * of the actions is still running.
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        boolean isFinished = true;
        for (int i = 0; i < actionsFinished.length; i++) {
            if (!actionsFinished[i]) {
                actionsFinished[i] = actions[i].run(telemetryPacket);
                if (!actionsFinished[i]) {
                    isFinished = false;
                }
            }
        }

        return isFinished;
    }
}
