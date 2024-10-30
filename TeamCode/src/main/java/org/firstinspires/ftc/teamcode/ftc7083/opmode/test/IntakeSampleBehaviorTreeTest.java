package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTree.IntakeSampleBehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;

@Autonomous (name = "Intake Sample BT", group = "Test")
public class IntakeSampleBehaviorTreeTest extends LinearOpMode {

    IntakeSampleBehaviorTree behaviorTree = null;

    int loopCount = 0;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("RedAudienceAutonomousBoadScoringOpMode", "runOpMode started");
        telemetry.update();
        initialize(this);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("[IntakeSampleBT]", "runOpMode while started");
            telemetry.update();

            Status result = this.behaviorTree.tick();

            telemetry.addData("[IntakeSampleBT]", "Behavior tree result: %s", result);
            telemetry.update();

            telemetry.addData("[IntakeSampleBT]", "loop count: %d", this.loopCount);
            telemetry.update();

            loopCount++;

            if (result == Status.SUCCESS || result == Status.FAILURE) {
                telemetry.addData("[IntakeSampleBT]", "runOpMode success");
                telemetry.update();
                requestOpModeStop();

            }
        }
    }

    private void initialize(LinearOpMode opMode){
        this.behaviorTree = new IntakeSampleBehaviorTree(hardwareMap,telemetry);
    }

}