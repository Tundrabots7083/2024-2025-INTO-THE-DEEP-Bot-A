package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels.CenterStageWorldModel;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Action;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Sequence;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.DetectAprilTags;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.LocalizeByAprilTag;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance.NavigateToStagingPosition1;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.models.DriveTrainConfig;
import org.firstinspires.ftc.teamcode.models.StagingPosition;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageVisionDetectorSingleton;

import java.util.Arrays;

public class BoardScoring {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton globalStore;
    private WorldModel worldModel;
    protected LinearOpMode opMode;

    public BoardScoring(LinearOpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData("BoardScoring", "BoardScoring started");
        opMode.telemetry.update();
        Init();
    }

    private void Init() {
        this.worldModel = new CenterStageWorldModel();

        CenterStageVisionDetectorSingleton.reset();

        this.globalStore = GlobalStoreSingleton.getInstance(opMode);
        this.initializeGlobalStore();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectAprilTags(this.opMode),this.opMode),
                        new Action(new LocalizeByAprilTag(this.opMode),this.opMode),
                        new Action(new NavigateToStagingPosition1(this.opMode),this.opMode)//,
                    //    new Action(new NavigateToScoringPosition1(this.opMode),this.opMode),
                    //    new Action(new ScoreBoardPixel(this.opMode),this.opMode)
                ),this.opMode);

        this.tree = new BehaviorTree(root, globalStore);
    }
    private void initializeGlobalStore(){
        DriveTrainConfig driveTrainConfig = new DriveTrainConfig();
        driveTrainConfig.speedGain =0.033;
        driveTrainConfig.strafeGain=0.02;
        driveTrainConfig.turnGain=0.04;

        this.globalStore.setValue("DriveTrainConfig", driveTrainConfig);
        this.globalStore.setValue("YawStep", 0.25);
        this.globalStore.setValue("WorldModel", this.worldModel);

        this.globalStore.setValue("ReferenceAprilTagId",7);
        this.globalStore.setValue("YawStep",-0.25);

        this.globalStore.setValue("CurrentStagingPosition1", StagingPosition.RED_FIVE_CENTER);
        this.globalStore.setValue("CurrentStagingPosition2", StagingPosition.RED_FIVE_RIGHT);

    }
    public Status tick() {
        // Run the behavior tree
        Status result = tree.tick();
        opMode.telemetry.addData("BoardScoring", "Run - Behavior tree result: %s",result);
        opMode.telemetry.update();

        return result;
    }
}

