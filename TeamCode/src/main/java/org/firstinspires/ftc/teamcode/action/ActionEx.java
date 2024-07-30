package org.firstinspires.ftc.teamcode.action;

import com.acmerobotics.roadrunner.Action;

/**
 * Extended RoadRunner action, which includes some useful additional functions.
 */
public interface ActionEx extends Action {

    /**
     * Decorates this action with a set of actions to run parallel to it, ending when the last action ends.
     *
     * @param actions the set of actions to run in parallel with this action
     * @return an action that runs this action and the set of actions provided in parallel.
     */
    ActionEx alongWith(Action... actions);

    /**
     * Decorates this action with a set of actions to run after it in sequence. Often more
     * convenient/less-verbose than constructing a new SequentialAction explicitly.
     *
     * @param actions the actions to run next
     * @return an action that runs this action and then the set of actions sequentially.
     */
    ActionEx andThen(Action... actions);

    /**
     * Returns an action that delays the execution of this action for the specified number of
     * milliseconds.
     *
     * @param millis the number of milliseconds to wait until executing this action
     * @return an action that delays the execution of this action
     */
    ActionEx delayFor(long millis);

    /**
     * Returns an Action that runs this action for a maximum of <code>millis</code> milliseconds.
     *
     * @param millis the maximum amount of time this action may run.
     * @return an Action that runs this action for a maximum of <code>millis</code> milliseconds
     */
    ActionEx withTimeout(long millis);

    /**
     * Returns an Action that waits for the requested number of milliseconds before completing the
     * action itself.
     *
     * @param millis the number of milliseconds to wait after completion of this action
     * @return an action that waits after completion of this action
     */
    ActionEx withWait(long millis);
}
