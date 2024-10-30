package org.firstinspires.ftc.teamcode.ftc7083.fsm;

/**
 * Action that occurs between transitions of a FSM.
 */
public interface Action {

    /**
     * Called once the action is to tick, but before the <code>execute</code> method is called.
     */
    default void initialize() {
    }

    /**
     * Called repeatedly until the action is finished or is cancelled.
     */
    void execute();

    /**
     * Cancels the execution of an action.
     */
    default void cancel() {
    }

    /**
     * Called when the action completes. This occurs when <code>cancel</code> is called, or when
     * <code>isFinished</code> returns <code>true</code>.
     */
    default void end() {
    }

    boolean isFinished();
}
