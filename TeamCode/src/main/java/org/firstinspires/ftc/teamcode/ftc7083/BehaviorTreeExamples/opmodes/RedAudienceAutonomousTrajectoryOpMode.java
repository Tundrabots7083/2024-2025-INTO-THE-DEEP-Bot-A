package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees.RedAudienceAutonomousWithDeadWheels;

@Disabled
@Autonomous(name="BT Drive Trajectory", group="vision")
public class RedAudienceAutonomousTrajectoryOpMode  extends LinearOpMode
{
    RedAudienceAutonomousWithDeadWheels behaviorTree = null;

    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("RedAudienceAutonomousTrajectoryOpMode", "runOpMode started");
        telemetry.update();
        initialize(this);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("RedAudienceAutonomousTrajectoryOpMode", "runOpMode while started");
            telemetry.update();
            Status result = this.behaviorTree.run();

            telemetry.addData("RedAudienceAutonomousWithDeadWheels", "Behavior tree result: %s",result);
            telemetry.update();

            if(result == Status.SUCCESS){
                telemetry.addData("RedAudienceAutonomousTrajectoryOpMode", "runOpMode success");
                telemetry.update();
                requestOpModeStop();
            }
        }
    }

    private void initialize(LinearOpMode opMode){

        this.behaviorTree = new RedAudienceAutonomousWithDeadWheels(opMode);
    }

}
