package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees.BoardScoring;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;

@Disabled
@Autonomous(name="BT Scoring Board", group="vision")
public class RedAudienceAutonomousBoadScoringOpMode extends LinearOpMode
{
    BoardScoring behaviorTree = null;
    //  RedAudienceAutonomousWithDeadWheels behaviorTree = null;

    int loopCount = 0;

    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "runOpMode started");
        telemetry.update();
        initialize(this);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "runOpMode while started");
            telemetry.update();

            Status result = this.behaviorTree.tick();

            telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "Behavior tree result: %s",result);
            telemetry.update();

            telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "loop count: %d",this.loopCount);
            telemetry.update();

            loopCount++;

            if(result == Status.SUCCESS || result == Status.FAILURE){
                telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "runOpMode success");
                telemetry.update();
                requestOpModeStop();


            }
        }
    }

    private void initialize(LinearOpMode opMode){

        this.behaviorTree = new BoardScoring(opMode);

    }

}

