package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;

public class LocalizeByOdometry  implements ActionFunction {

    private LinearOpMode opMode;
    public LocalizeByOdometry(LinearOpMode opMode) {
        this.opMode = opMode;
        //initialize mech drive
        //initialize localizer
        //set localizer to mech drive
    }

    public Status perform(GlobalStoreSingleton globalStore) {
        return Status.SUCCESS;
    }
}
