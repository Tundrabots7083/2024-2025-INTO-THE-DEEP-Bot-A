package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees.RedAudienceAutonomous;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;

@Disabled
@Autonomous(name="BT Drive To AprilTag", group="vision")
public class RedAudienceAutonomousOpMode extends LinearOpMode
{
   RedAudienceAutonomous behaviorTree = null;
    //  RedAudienceAutonomousWithDeadWheels behaviorTree = null;

    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode started");
       // telemetry.addData("BTAutoDriveToAprilTagOpMode hw", this.hardwareMap.toString());
        telemetry.update();
        initialize(this);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode while started");
            telemetry.update();

            Status result = this.behaviorTree.tick();

            telemetry.addData("RedAudienceAutonomous", "Behavior tree result: %s",result);
            telemetry.update();

            if(result == Status.SUCCESS){
                telemetry.addData("BTAutoDriveToAprilTagOpMode", "runOpMode success");
                telemetry.update();
                requestOpModeStop();
            }
        }
    }

    private void initialize(LinearOpMode opMode){

        this.behaviorTree = new RedAudienceAutonomous(opMode);
        //this.behaviorTree = new RedAudienceAutonomousWithDeadWheels(opMode);


    }

}
