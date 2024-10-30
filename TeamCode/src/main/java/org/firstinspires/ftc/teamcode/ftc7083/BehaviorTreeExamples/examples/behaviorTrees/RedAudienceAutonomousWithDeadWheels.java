package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels.CenterStageWorldModel;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Action;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.DetectAprilTags;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance.NavigateRA1Trajectory;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance.NavigateToTeamProp;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

import java.util.Arrays;

public class RedAudienceAutonomousWithDeadWheels
{
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton globalStore;
    private WorldModel worldModel;
    protected LinearOpMode opMode;

    public RedAudienceAutonomousWithDeadWheels(LinearOpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData("RedAudienceAutonomousWithDeadWheels", "RedAudienceAutonomousWithDeadWheels started");
        opMode.telemetry.update();
        Init();
    }

    private void Init() {
        this.worldModel = new CenterStageWorldModel();

        this.globalStore = GlobalStoreSingleton.getInstance(opMode);
        this.initializeGlobalStore();

        this.root = new Sequence(
                Arrays.asList(
                        //new Action(new DetectTeamProp(this.opMode),this.opMode),
                        new Action(new DetectAprilTags(this.opMode),this.opMode),
                        new Action(new NavigateToTeamProp(this.opMode),this.opMode),
                        //new Action(new ScoreSpikePixel(this.opMode),this.opMode),
                        new Action(new NavigateRA1Trajectory(this.opMode),this.opMode)

                ),this.opMode
        );

        this.tree = new BehaviorTree(root, globalStore);
    }
    private void initializeGlobalStore(){
        DriveTrainConfig driveTrainConfig = new DriveTrainConfig();
        driveTrainConfig.speedGain =0.033;
        driveTrainConfig.strafeGain=0.02;
        driveTrainConfig.turnGain=0.04;

        this.globalStore.setValue("DriveTrainConfig", driveTrainConfig);
        this.globalStore.setValue("WorldModel", this.worldModel);

        this.globalStore.setValue("StartPose",new Pose2d());//to be set to the actual starting pose
    }
    public Status run() {
        // Run the behavior tree
        Status result = tree.tick();
        opMode.telemetry.addData("RedAudienceAutonomousWithDeadWheels", "Run - Behavior tree result: %s",result);
        opMode.telemetry.update();

        return result;
    }
}
