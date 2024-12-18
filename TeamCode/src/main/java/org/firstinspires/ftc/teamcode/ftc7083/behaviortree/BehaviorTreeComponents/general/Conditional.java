package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general;

public class Conditional extends Node {
    private Condition condition;

    public Conditional(Condition condition) {
        this.condition = condition;
    }

    @Override
    public Status execute(BlackBoardSingleton blackBoard) {
        boolean result = condition.check(blackBoard);
        System.out.println("Conditional check result: " + result);
        if (result == true) {
            return Status.SUCCESS;
        } else {
            return Status.FAILURE;
        }

        // return condition.check() ? Status.SUCCESS : Status.FAILURE;
    }
}
