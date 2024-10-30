package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.examples.worldModels.CenterStageWorldModel;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Action;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Sequence;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.DetectAprilTags;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.LocalizeByAprilTag;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageVisionDetectorSingleton;

import java.util.Arrays;

public class AprilTagDetectionAndLocalization {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    private WorldModel worldModel;
    protected LinearOpMode opMode;

    public AprilTagDetectionAndLocalization(LinearOpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData("RedAudienceAutonomous", "RedAudienceAutonomous started");
        opMode.telemetry.update();
        Init();
    }

    private void Init() {
        this.worldModel = new CenterStageWorldModel();

        CenterStageVisionDetectorSingleton.reset();

        this.blackBoard = BlackBoardSingleton.getInstance(opMode.telemetry);
        this.initializeGlobalStore();

        this.root = new Sequence(
                Arrays.asList(
                        new Action(new DetectAprilTags(this.opMode),this.opMode),
                        new Action(new LocalizeByAprilTag(this.opMode),this.opMode)
                ),this.opMode);

        this.tree = new BehaviorTree(root, blackBoard);
    }
    private void initializeGlobalStore(){
        this.blackBoard.setValue("WorldModel", this.worldModel);
    }
    public Status tick() {
        // Run the behavior tree
        Status result = tree.tick();
        opMode.telemetry.addData("RedAudienceAutonomous", "Run - Behavior tree result: %s",result);
        opMode.telemetry.update();

        return result;
    }
}

