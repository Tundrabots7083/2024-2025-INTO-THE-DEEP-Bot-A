package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.roadrunner.Action;

/**
 * An implementation of the ActionEx interface with all methods but the <code>run</code> method
 * implemented.
 */
public abstract class ActionExBase implements ActionEx {
    /**
     * Decorates this action with a set of actions to run parallel to it, ending when the last action ends.
     *
     * @param actions the set of actions to run in parallel with this action
     * @return an action that runs this action and the set of actions provided in parallel.
     */
    @Override
    public ActionEx alongWith(Action... actions) {
        Action[] parallelActions = new Action[actions.length + 1];
        parallelActions[0] = this;
        System.arraycopy(actions, 0, parallelActions, 1, actions.length);
        return new ParallelAction(parallelActions);
    }

    /**
     * Decorates this action with a set of actions to run after it in sequence. Often more
     * convenient/less-verbose than constructing a new SequentialAction explicitly.
     *
     * @param actions the actions to run next
     * @return an action that runs this action and then the set of actions sequentially.
     */
    public ActionEx andThen(Action... actions) {
        Action[] sequentialActions = new Action[actions.length + 1];
        sequentialActions[0] = this;
        System.arraycopy(actions, 0, sequentialActions, 1, actions.length);
        return new SequentialAction(sequentialActions);
    }

    /**
     * Returns an action that delays the execution of this action for the specified number of
     * milliseconds.
     *
     * @param millis the number of milliseconds to wait until executing this action
     * @return an action that delays the execution of this action
     */
    @Override
    public ActionEx delayFor(long millis) {
        return new SequentialAction(new WaitAction(millis), this);
    }

    /**
     * Decorates this action with a timeout.
     *
     * @param millis the maximum number of milliseconds the action may run before being interrupted
     * @return this action
     */
    @Override
    public ActionEx withTimeout(long millis) {
        return new TimeoutAction(this, millis);
    }

    /**
     * Decorates this action with a wait.
     *
     * @param millis the number of milliseconds to wait after this action completes
     * @return this action
     */
    @Override
    public ActionEx withWait(long millis) {
        return new SequentialAction(this, new WaitAction(millis));
    }
}
