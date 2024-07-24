package org.firstinspires.ftc.teamcode.test;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.fsm.Action;
import org.firstinspires.ftc.teamcode.fsm.FSM;
import org.junit.Test;

public class FSMTest {
    @Test
    public void testFSM() {
        FSM<State, Event> fsm = new FSM<>(State.NOT_STARTED);
        fsm.addTransition(State.NOT_STARTED, State.STARTED, Event.START, new MyAction("Starting"))
                .addTransition(State.STARTED, State.COMPLETED, Event.FINISH, new MyAction("Completed"))
                .addTransition(State.STARTED, State.CANCELLED, Event.CANCEL, new MyAction("Cancelled"))
                .addTransition(State.STARTED, State.FAILED, Event.FAIL, new MyAction("Failed"));

        do {
            switch (fsm.getState()) {
                case NOT_STARTED:
                    fsm.transition(Event.START);
                    break;
                case STARTED:
                    fsm.transition(Event.FINISH);
                    break;
            }
            fsm.execute();
        } while (fsm.isRunning());
    }

    enum State {
        NOT_STARTED,
        STARTED,
        COMPLETED,
        FAILED,
        CANCELLED,
    }

    enum Event {
        START,
        FINISH,
        CANCEL,
        FAIL
    }

    private static class MyAction implements Action {
        public final String message;

        public MyAction(String message) {
            this.message = message;
        }

        @Override
        public void execute() {
            System.out.println(message);
        }

        @Override
        public boolean isFinished() {
            return true;
        }

        @NonNull
        @Override
        public String toString() {
            return "MyAction{" +
                    "message='" + message + '\'' +
                    '}';
        }
    }
}
