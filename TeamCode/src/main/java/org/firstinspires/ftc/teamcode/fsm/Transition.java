package org.firstinspires.ftc.teamcode.fsm;

import androidx.annotation.NonNull;

public class Transition<State, Event> {

    private final State initialState;
    private final State targetState;
    private final Event event;
    private final ActionExecutor actionExecutor;

    public Transition(State initialState, State targetState, Event event) {
        this(initialState, targetState, event, null);
    }

    public Transition(State initialState, State targetState, Event event, Action action) {
        this.initialState = initialState;
        this.targetState = targetState;
        this.event = event;
        this.actionExecutor = new ActionExecutor(action);
    }

    public State getInitialState() {
        return initialState;
    }

    public State getTargetState() {
        return targetState;
    }

    public Event getEvent() {
        return event;
    }

    public Action getAction() {
        return actionExecutor.getAction();
    }

    public ActionExecutor getActionExecutor() {
        return actionExecutor;
    }

    /**
     * Gets a string representation of the transition.
     *
     * @return a string representation of the transition
     */
    @NonNull
    @Override
    public String toString() {
        return "Transition{" +
                "initialState=" + initialState +
                ", targetState=" + targetState +
                ", event=" + event +
                ", actionExecutor=" + actionExecutor +
                '}';
    }
}
