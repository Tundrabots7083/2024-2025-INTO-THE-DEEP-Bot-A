package org.firstinspires.ftc.teamcode.ftc7083.fsm;

import androidx.annotation.NonNull;

import java.util.HashMap;
import java.util.Map;

/**
 * Simple finite state machine that allows the transitioning from one state to another given
 * an event trigger.
 * <p>
 * TODO: possible enhancements
 *       - Add an action that occurs during a transition between states
 *
 * @param <State> FSM states
 * @param <Event> triggers for the state change
 */
public class FSM<State, Event> {
    private final State initialState;
    private final Map<State, Map<Event, Transition<State, Event>>> transitions = new HashMap<>();
    private State currentState;
    private Event currentEvent = null;
    private Transition<State, Event> currentTransition = null;

    /**
     * Instantiates a new FSM. This takes an initial state for the FSM.
     *
     * @param initialState the initial state for the FSM.
     */
    public FSM(State initialState) {
        this.initialState = initialState;
        this.currentState = initialState;
    }

    /**
     * Adds a new transition for the FSM.
     *
     * @param currentState the current state from which to transition
     * @param targetState  the next state in the transition, or <code>null</code> if there is none
     * @param event        the event that triggers the transition
     * @return this FSM
     */
    public FSM<State, Event> addTransition(State currentState, State targetState, Event event) {
        return addTransition(currentState, targetState, event, null);
    }

    /**
     * Adds a new transition for the FSM.
     *
     * @param currentState the current state from which to transition
     * @param targetState  the next state in the transition, or <code>null</code> if there is none
     * @param event        the event that triggers the transition
     * @param action       the action to execute during the transition
     * @return this FSM
     */
    public FSM<State, Event> addTransition(State currentState, State targetState, Event event, Action action) {
        Transition<State, Event> transition = new Transition<State, Event>(currentState, targetState, event, action);
        Map<Event, Transition<State, Event>> stateTransitions = this.transitions.computeIfAbsent(currentState, k -> new HashMap<>());
        stateTransitions.put(event, transition);

        return this;
    }

    /**
     * Bulk add of a set of transitions to the FSM.
     *
     * @param currentState the state from which to transition
     * @param transitions  the events and the resulting state transitions
     * @return this FSM.
     */
    public FSM<State, Event> addTransitions(State currentState, Map<Event, Transition<State, Event>> transitions) {
        Map<Event, Transition<State, Event>> stateTransitions = this.transitions.computeIfAbsent(currentState, k -> new HashMap<>());
        stateTransitions.putAll(transitions);
        return this;
    }

    /**
     * Transitions the FSM to the next state.
     *
     * @param event the event that triggers the transition.
     */
    public void transition(Event event) {
        currentEvent = event;
        Map<Event, Transition<State, Event>> stateTransitions = transitions.get(currentState);
        currentTransition = stateTransitions != null ? stateTransitions.get(event) : null;
        currentState = currentTransition != null ? currentTransition.getTargetState() : null;
    }

    /**
     * Transitions to the next state as identified by the action executor.
     *
     * @param actionExecutor the action executor running the current action.
     */
    protected void transition(ActionExecutor actionExecutor) {
        State targetState = currentTransition.getTargetState();
        Map<Event, Transition<State, Event>> transitionStates = transitions.get(currentState);
        if (transitionStates != null) {
            Transition<State, Event> transition = transitionStates.get(currentTransition.getEvent());
            if (transition != null) {
                transition(transition.getEvent());
            }
        }
        currentState = targetState;
    }

    /**
     * Executes the current action in the FSM, and transitions to the next state when done.
     */
    public void execute() {
        if (currentTransition != null) {
            ActionExecutor actionExecutor = currentTransition.getActionExecutor();
            if (!actionExecutor.isFinished()) {
                actionExecutor.execute();
            } else {
                transition(actionExecutor);
            }
        }
    }

    /**
     * Resets the FSM to the initial state.
     */
    public void reset() {
        currentState = initialState;
        currentEvent = null;
    }

    /**
     * Gets the current state of the FSM.
     *
     * @return the current state of the FSM
     */
    public State getState() {
        return currentState;
    }

    /**
     * Gets the current action that is being executed.
     *
     * @return the current action that is being executed, or <code>null</code> if there is not
     * current action.
     */
    public Action getCurrentAction() {
        return currentTransition != null ? currentTransition.getAction() : null;
    }

    /**
     * Get the event that triggered the current state transition.
     *
     * @return the event that triggered the current state transition, or <code> null</code> if
     * there hasn't been a state transition started yet.
     */
    public Event getLastEvent() {
        return currentEvent;
    }

    /**
     * Returns indication as to whether the FSM is running
     *
     * @return <code>true</code> if the FSM is running; <code>false</code> if it is not.
     */
    public boolean isRunning() {
        Map<Event, Transition<State, Event>> transitionStates = transitions.get(currentState);
        return transitionStates != null || !getCurrentAction().isFinished();
    }

    @NonNull
    @Override
    public String toString() {
        return "FSM{" +
                "initialState=" + initialState +
                ", transitions=" + transitions +
                ", currentState=" + currentState +
                ", currentEvent=" + currentEvent +
                ", currentTransition=" + currentTransition +
                '}';
    }
}
