package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.Selected;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageVisionDetectorSingleton;


public class DetectTeamProp implements ActionFunction {
    private FirstVisionProcessor visionProcessor;
    private CenterStageVisionDetectorSingleton visionDetector;
    private LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;

    public DetectTeamProp(LinearOpMode opMode) {
        this.opMode=opMode;


        this.visionDetector =  CenterStageVisionDetectorSingleton.getInstance(opMode);

        this.visionProcessor = visionDetector.getVisionProcessor();
    }

    public Status perform(GlobalStoreSingleton globalStore) {
        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        Selected teamPropPosition = visionProcessor.getSelection();
        opMode.telemetry.addData("DetectTeamProp ",teamPropPosition.name());
        opMode.telemetry.update();

        globalStore.setValue("TeamPropPosition", teamPropPosition);
        return Status.SUCCESS;
    }
}
