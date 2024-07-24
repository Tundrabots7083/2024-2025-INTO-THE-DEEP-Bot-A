package org.firstinspires.ftc.teamcode.fsm;

import androidx.annotation.NonNull;

/**
 * Class that runs an action.
 */
public class ActionExecutor {
    private final Action action;
    private boolean initialized = false;
    private boolean finished = false;

    /**
     * Creates an executor for a given action.
     *
     * @param action the action to execute.
     */
    protected ActionExecutor(Action action) {
        this.action = action;
    }

    /**
     * Initialize the action.
     */
    void initialize() {
        if (!initialized) {
            action.initialize();
            initialized = true;
        }
    }

    /**
     * Execute the action.
     */
    void execute() {
        if (action != null) {
            if (!initialized) {
                initialize();
            }

            action.execute();
        }
        if (action == null || action.isFinished()) {
            end();
        }
    }

    /**
     * Cancel the action.
     */
    void cancel() {
        finished = true;
        if (action != null) {
            action.cancel();
        }
    }

    /**
     * End the action.
     */
    void end() {
        finished = true;
        if (action != null) {
            action.end();
        }
    }

    /**
     * Returns an indication as to whether the action has completed execution.
     *
     * @return <code>true</code> if the action has completed execution;
     * <code>false</code> if it is still running.
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Gets the action that is to be executed.
     *
     * @return the action that is to be executed.
     */
    public Action getAction() {
        return action;
    }

    /**
     * Gets a string representation of the ActionExecutor;
     *
     * @return a string representation of the ActionExecutor
     */
    @NonNull
    @Override
    public String toString() {
        return "ActionExecutor{" +
                "action=" + action +
                ", initialized=" + initialized +
                ", finished=" + finished +
                '}';
    }
}
