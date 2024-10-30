package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.behaviorTrees;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.DetectAprilTags;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.LocalizeByAprilTag;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions.redAlliance.NavigateRA1Trajectory;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Action;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BehaviorTree;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Node;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;

import java.util.Arrays;

public class RedAudienceAutonomous {
    private BehaviorTree tree;
    private Node root;
    private BlackBoardSingleton blackBoard;
    private WorldModel worldModel;
    protected LinearOpMode opMode;

    public RedAudienceAutonomous(LinearOpMode opMode) {
        this.opMode = opMode;

        opMode.telemetry.addData("RedAudienceAutonomous", "RedAudienceAutonomous started");
        opMode.telemetry.update();
        Init();
    }

    private void Init() {
        this.worldModel = new CenterStageWorldModel();

        CenterStageVisionDetectorSingleton.reset();
        CenterStageORMechanumDriveSingleton.reset();
        BlackBoardSingleton.reset();

        this.blackBoard = GlobalStoreSingleton.getInstance(opMode);
        this.initializeGlobalStore();

        this.root = new Sequence(
                Arrays.asList(
                      //  new Action(new DetectTeamProp(this.opMode),this.opMode),
                        new Action(new DetectAprilTags(this.opMode),this.opMode),
                        new Action(new LocalizeByAprilTag(this.opMode),this.opMode),
                        new Action(new NavigateRA1Trajectory(this.opMode),this.opMode)//,
       // new Action(new NavigateToStagingPosition1(this.opMode),this.opMode)//,

//                      //LocalizeByAprilTag
                 //       new Action(new NavigateToTeamProp(this.opMode),this.opMode)//,
                        //new Action(new ScoreSpikePixel(this.opMode),this.opMode),
                       // new Action(new NavigateRA1Trajectory(this.opMode),this.opMode)

                ),this.opMode);


 //       this.root = new Sequence(
   //             Arrays.asList(
     //                   new Selector(
       //                         Arrays.asList(
         //                               new Action(new DetectAprilTags(this.opMode),this.opMode),
           //                             new Action(new SearchTarget(this.opMode),this.opMode)
             //                   )
               //         ,this.opMode),
                     //   new Action(new DetectTeamProp(this.opMode),this.opMode),

                 //       new Action(new NavigateRA1(this.opMode),this.opMode)//,
                      //  new Action(new LocalizeByAprilTag(this.opMode),this.opMode)//,
                        //new Action(new ScoreSpikePixel(this.opMode),this.opMode),
                     /*   new Action(new NavigateRA2(this.opMode),this.opMode),
                      //  new Action(new NavigateRA3(this.opMode),this.opMode),
                        new Action(new NavigateRA4(this.opMode),this.opMode),
                        new Action(new NavigateRA5(this.opMode),this.opMode),
                     //   new Action(new ScoreBoardPixel(this.opMode),this.opMode),
                        new Action(new NavigateRA6(this.opMode),this.opMode)*/

       //         ),this.opMode
  //      );

        this.tree = new BehaviorTree(root, blackBoard);
    }
    private void initializeGlobalStore(){
        DriveTrainConfig driveTrainConfig = new DriveTrainConfig();
        driveTrainConfig.speedGain =0.033;
        driveTrainConfig.strafeGain=0.02;
        driveTrainConfig.turnGain=0.04;

        this.blackBoard.setValue("DriveTrainConfig", driveTrainConfig);
        this.blackBoard.setValue("YawStep", 0.25);
        this.blackBoard.setValue("WorldModel", this.worldModel);

        this.blackBoard.setValue("ReferenceAprilTagId",7);
        this.blackBoard.setValue("YawStep",-0.25);

        Pose2d robotStartingPose = new Pose2d(-45, -43, Math.toRadians(0));
        this.blackBoard.setValue("RobotStartingPose",robotStartingPose);
    }
    public Status tick() {
        // Run the behavior tree
        Status result = tree.tick();
        opMode.telemetry.addData("RedAudienceAutonomous", "Run - Behavior tree result: %s",result);
        opMode.telemetry.update();

        return result;
    }
}
