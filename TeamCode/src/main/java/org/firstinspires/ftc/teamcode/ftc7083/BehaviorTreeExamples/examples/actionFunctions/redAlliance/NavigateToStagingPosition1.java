package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels.CenterStageWorldModel;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.models.PIDNCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;
import org.firstinspires.ftc.teamcode.models.StagingPosition;

public class NavigateToStagingPosition1 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateToStagingPosition1(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        setRelativePositionTargetParameters(globalStore);

        setPIDCoeficients(globalStore);
        setErrorTolerances(globalStore);

        setNavigationType(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;
    }
    private void setRelativePositionTargetParameters(GlobalStoreSingleton globalStore){
        CenterStageWorldModel worldModel = (CenterStageWorldModel)globalStore.getValue("WorldModel");
        StagingPosition stagingPosition = (StagingPosition) globalStore.getValue("CurrentStagingPosition1");

        RelativePosition stagingRelativePosition = worldModel.getValue(stagingPosition.toString()).relativePosition;

        globalStore.setValue("CurrentTarget", stagingRelativePosition);
    }
    private void setNavigationType(GlobalStoreSingleton globalStore){

        globalStore.setValue("NavigationType", NavigationType.RELATIVE);
    }
    private void setPIDCoeficients(GlobalStoreSingleton globalStore){
        PIDNCoeficients PIDNCoeficients = new PIDNCoeficients();
        PIDNCoeficients.HKd=0;
        PIDNCoeficients.HKi=0;
        PIDNCoeficients.HKp=0.025;

        PIDNCoeficients.RKd=0;
        PIDNCoeficients.RKi=0;
        PIDNCoeficients.RKp=0.033;

        PIDNCoeficients.YKd=0;
        PIDNCoeficients.YKi=0;
        PIDNCoeficients.YKp=0.022;

        globalStore.setValue("PIDCoeficients", PIDNCoeficients);
    }

    private void setErrorTolerances(GlobalStoreSingleton globalStore){
        ErrorTolerances errorTolerances = new ErrorTolerances();
        errorTolerances.headingErrorTolerance=0.75;
        errorTolerances.rangeErrorTolerance=0.75;
        errorTolerances.yawErrorTolerance=0.75;

        globalStore.setValue("ErrorTolerances", errorTolerances);

    }
}
