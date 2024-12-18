package org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.ActionFunctions;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.behaviortree.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

public class ExtendArmToSubmersibleSample implements ActionFunction {

    IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    Telemetry telemetry;

    protected Status lastStatus = Status.FAILURE;
    protected int runCount = 0;

    public ExtendArmToSubmersibleSample (Telemetry telemetry, IntakeAndScoringSubsystem intakeAndScoringSubsystem) {
        this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
        this.telemetry = telemetry;
    }

    public Status perform(BlackBoardSingleton blackBoard) {
        Status status = Status.RUNNING;
        double xDistance;

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }



        if(blackBoard.getValue("xDistanceToSample") != null) {
            xDistance = (double)blackBoard.getValue("xDistanceToSample");
        } else {
            status = Status.FAILURE;
            return status;
        }

        intakeAndScoringSubsystem.moveToPosition(xDistance,1);
        intakeAndScoringSubsystem.execute();

        if(intakeAndScoringSubsystem.isAtTarget()) {
            status = Status.SUCCESS;
        }

        runCount++;
        lastStatus = status;

        return status;
    }
}
