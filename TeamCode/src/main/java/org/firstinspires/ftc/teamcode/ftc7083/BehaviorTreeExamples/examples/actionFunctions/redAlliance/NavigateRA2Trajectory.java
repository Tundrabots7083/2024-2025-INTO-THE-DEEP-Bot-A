package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

public class NavigateRA2Trajectory extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    public NavigateRA2Trajectory(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        setTrajectory(globalStore);
        setNavigationType(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;

    }

    private void setTrajectory(GlobalStoreSingleton globalStore){
        double DISTANCE = 50;

        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);
        Trajectory currentTrajectory = standardTrajectoryBuilder.trajectoryBuilder(new Pose2d())
                .back(DISTANCE)
                .build();

        globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }
    private void setNavigationType(GlobalStoreSingleton globalStore){

        globalStore.setValue("NavigationType", NavigationType.ABSOLUTE);
    }
}
