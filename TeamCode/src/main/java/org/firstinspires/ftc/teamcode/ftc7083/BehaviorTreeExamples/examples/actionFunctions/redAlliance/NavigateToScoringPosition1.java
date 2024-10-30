package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDNCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class NavigateToScoringPosition1 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateToScoringPosition1(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        RelativePosition currentTarget = new RelativePosition(7,61.3,4,-3);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",7);
        globalStore.setValue("YawStep",-0.25);

        setPIDCoeficients(globalStore);
        setErrorTolerances(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;
    }
    private void setPIDCoeficients(GlobalStoreSingleton globalStore){
        PIDNCoeficients PIDNCoeficients = new PIDNCoeficients();
        PIDNCoeficients.HKd=0.0;
        PIDNCoeficients.HKi=0.12;
        PIDNCoeficients.HKp=0.028;

        PIDNCoeficients.RKd=0;
        PIDNCoeficients.RKi=0.12;
        PIDNCoeficients.RKp=0.033;

        PIDNCoeficients.YKd=0;
        PIDNCoeficients.YKi=0.1;
        PIDNCoeficients.YKp=0.027;

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
