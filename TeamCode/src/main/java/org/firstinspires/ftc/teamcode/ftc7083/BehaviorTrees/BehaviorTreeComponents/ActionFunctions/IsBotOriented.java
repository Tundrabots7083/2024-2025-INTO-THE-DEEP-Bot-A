package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Condition;

public class IsBotOriented implements Condition {

    public boolean check(BlackBoardSingleton blackBoard) {
        return blackBoard.getValue("BotIsSquaredWithSample") != null &&
                blackBoard.getValue("BotIsInIntakeRange") != null &&
                (boolean) blackBoard.getValue("BotIsSquaredWithSample") &&
                (boolean) blackBoard.getValue("BotIsInIntakeRange");
    }
}
