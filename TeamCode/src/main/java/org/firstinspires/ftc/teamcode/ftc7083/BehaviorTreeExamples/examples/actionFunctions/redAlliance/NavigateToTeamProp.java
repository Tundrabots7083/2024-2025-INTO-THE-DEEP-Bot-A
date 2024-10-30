package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.models.Selected;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

public class NavigateToTeamProp extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;
    StandardTrajectoryBuilder standardTrajectoryBuilder;
    Pose2d robotStartingPose;
    public NavigateToTeamProp(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

       robotStartingPose = new Pose2d(-36, -64.5, Math.toRadians(90));
    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }

        standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

       // setTeamPropTrajectory(globalStore);
        setMiddleTeamPropTrajectory(globalStore);

        setNavigationType(globalStore);

        globalStore.setValue("RobotStartingPose",robotStartingPose);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;
    }

    private void setTeamPropTrajectory(GlobalStoreSingleton globalStore){
        Selected teamPropPosition = (Selected)globalStore.getValue("TeamPropPosition");


        standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);


        switch (teamPropPosition) {
            case LEFT:
                opMode.telemetry.addLine("Left Spike Mark");
                opMode.telemetry.update();
                setLeftTeamPropTrajectory(globalStore);
                break;
            case MIDDLE:
                opMode.telemetry.addLine("Middle Spike Mark");
                opMode.telemetry.update();
                setMiddleTeamPropTrajectory(globalStore);
                break;
            case RIGHT:
                opMode.telemetry.addLine("Right Spike Mark");
                opMode.telemetry.update();
                setRightTeamPropTrajectory(globalStore);
                break;

            case NONE:
                opMode.telemetry.addLine("Team Prop Not Detected");
                opMode.telemetry.update();
                setRightTeamPropTrajectory(globalStore);//assume right position when nothing is detected
                break;
        }
    }

    private void setLeftTeamPropTrajectory(GlobalStoreSingleton globalStore){
        Trajectory currentTrajectory = standardTrajectoryBuilder.trajectoryBuilder(robotStartingPose)
                .forward(10)
                .build();
        globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }
    private void setMiddleTeamPropTrajectory(GlobalStoreSingleton globalStore){
        // Move to center spike mark

        Trajectory currentTrajectory = standardTrajectoryBuilder.trajectoryBuilder(robotStartingPose)
                .forward(24)
                .build();
        globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }
    private void setRightTeamPropTrajectory(GlobalStoreSingleton globalStore){
        Trajectory currentTrajectory = standardTrajectoryBuilder.trajectoryBuilder(robotStartingPose)
                .forward(5)
                .build();
        globalStore.setValue("CurrentTrajectory", currentTrajectory);
    }


    private void setNavigationType(GlobalStoreSingleton globalStore){

        globalStore.setValue("NavigationType", NavigationType.ABSOLUTE);
    }
}
