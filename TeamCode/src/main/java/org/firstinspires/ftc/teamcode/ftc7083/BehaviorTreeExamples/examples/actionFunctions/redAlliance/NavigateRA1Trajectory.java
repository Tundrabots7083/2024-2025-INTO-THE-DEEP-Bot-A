package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.Navigate;
import org.firstinspires.ftc.teamcode.models.NavigationType;
import org.firstinspires.ftc.teamcode.subsystems.StandardTrajectoryBuilder;

public class NavigateRA1Trajectory  extends Navigate {
    LinearOpMode opMode;
    protected Status lastStatus = Status.FAILURE;

    private int counter=0;
    public NavigateRA1Trajectory(LinearOpMode opMode){
        super(opMode);
        this.opMode = opMode;

    }

    public Status perform(GlobalStoreSingleton globalStore) {

        if(lastStatus == Status.SUCCESS){
            return lastStatus;
        }
  /*
        return Status.FAILURE;
     */
        setTrajectory(globalStore);
        setNavigationType(globalStore);

        Status status = super.perform(globalStore);

        lastStatus = status;

        return status;


    }

    private void setTrajectory(GlobalStoreSingleton globalStore){
        StandardTrajectoryBuilder standardTrajectoryBuilder=new StandardTrajectoryBuilder(globalStore);

      Pose2d lastRobotPose = (Pose2d)globalStore.getValue("LastRobotPose");

      if(lastRobotPose==null){
          lastRobotPose = new Pose2d(0,0,Math.toRadians(0));

      }
        double DISTANCE = 15;

        opMode.telemetry.addData("NavigateRA1Trajectory-", "lastRobotPose %s counter %d",lastRobotPose.toString(), counter);
        opMode.telemetry.update();
        counter++;

        Trajectory currentTrajectory1  = standardTrajectoryBuilder.trajectoryBuilder(lastRobotPose)
                .lineTo(new Vector2d(15, 1))
                //.forward(DISTANCE)
                .build();

        Trajectory currentTrajectory  = standardTrajectoryBuilder.trajectoryBuilder(lastRobotPose)
                .splineToConstantHeading(new Vector2d(-40, -35), Math.toRadians(0))
                .build();


        //opMode.telemetry.addData("NavigateRA1Trajectory currentTrajectory", currentTrajectory.toString());
        //opMode.telemetry.update();
        globalStore.setValue("CurrentTrajectory", currentTrajectory1);
    }
    private void setNavigationType(GlobalStoreSingleton globalStore){

        globalStore.setValue("NavigationType", NavigationType.ABSOLUTE);
    }
}
