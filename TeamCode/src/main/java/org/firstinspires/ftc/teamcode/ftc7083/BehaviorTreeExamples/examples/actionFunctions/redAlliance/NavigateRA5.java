package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.ErrorTolerances;
import org.firstinspires.ftc.teamcode.models.PIDNCoeficients;
import org.firstinspires.ftc.teamcode.models.RelativePosition;

public class NavigateRA5 extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA5(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            opMode.sleep(30);//allow the system time to change the referenceTargetId

            return lastStatus;
        }

        RelativePosition currentTarget = new RelativePosition(6,20.9,-14,7);
        globalStore.setValue("CurrentTarget", currentTarget);

        globalStore.setValue("ReferenceAprilTagId",6);
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
        PIDNCoeficients.RKp=0.023;

        PIDNCoeficients.YKd=0;
        PIDNCoeficients.YKi=0.1;
        PIDNCoeficients.YKp=0.02;

        globalStore.setValue("PIDCoeficients", PIDNCoeficients);
    }

    private void setErrorTolerances(GlobalStoreSingleton globalStore){
        ErrorTolerances errorTolerances = new ErrorTolerances();
        errorTolerances.headingErrorTolerance=0.5;
        errorTolerances.rangeErrorTolerance=0.25;
        errorTolerances.yawErrorTolerance=0.5;

        globalStore.setValue("ErrorTolerances", errorTolerances);

    }
}
