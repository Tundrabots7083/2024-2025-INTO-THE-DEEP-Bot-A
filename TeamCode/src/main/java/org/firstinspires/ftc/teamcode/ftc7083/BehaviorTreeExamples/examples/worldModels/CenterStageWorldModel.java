package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.worldModels;

/*import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.ftc7083.models.RelativePosition;
import org.firstinspires.ftc.teamcode.ftc7083.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObjectSize;*/
public class CenterStageWorldModel extends org.firstinspires.ftc.teamcode.models.worldModel.WorldModel {
    public CenterStageWorldModel() {
        this.Init();
    }

    private void Init() {
   /*     //blue alliance backdrop board
        WorldObject blueAllianceBackdrop = new WorldObject("BlueAllianceBackdrop","BlueBackdrop", new Pose2d(60.75, 49.5), new WorldObjectSize(48, 35.0));

        this.setValue(blueAllianceBackdrop);

        //////////////////Board Tags
        //blue alliance backdrop board tags
        WorldObject blueAllianceLeft = new WorldObject("BlueAllianceLeft","1", new Pose2d(62, 42, 180), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceCenter = new WorldObject("BlueAllianceCenter","2", new Pose2d(62, 36, 180), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceRight = new WorldObject("BlueAllianceRight","3", new Pose2d(62, 30, 180), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceLeft);
        this.setValue(blueAllianceCenter);
        this.setValue(blueAllianceRight);

        //red alliance backdrop board tags
        WorldObject redAllianceLeft = new WorldObject("RedAllianceLeft","4", new Pose2d(62, -30.0,180), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceCenter = new WorldObject("RedAllianceCenter","5", new Pose2d(62, -36.0, 180), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceRight = new WorldObject("RedAllianceRight","6", new Pose2d(62, -42.0, 180), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceLeft);
        this.setValue(redAllianceCenter);
        this.setValue(redAllianceRight);

    //////////////////////////////////////////////////////////////

        //blue alliance backdrop board
        WorldObject redAllianceBackdrop = new WorldObject("RedAllianceBackdrop","RedBackdrop", new Pose2d(60.75, -12.75), new WorldObjectSize(48, 35.0));

        this.setValue(redAllianceBackdrop);

        /////////////////////////////Audience wall tags
        //red alliance wall target tags
        WorldObject redAllianceWallTargetLarge = new WorldObject("RedAllianceWallTargetLarge","7", new Pose2d(-72.0, -41.5, 0), new WorldObjectSize(5.0, 5.0));
        WorldObject redAllianceWallTargetSmall = new WorldObject("RedAllianceWallTargetSmall","8", new Pose2d(-72.0, -36.0, 0), new WorldObjectSize(2.0, 2.0));

        this.setValue(redAllianceWallTargetLarge);
        this.setValue(redAllianceWallTargetSmall);

        //blue alliance wall target tags
        WorldObject blueAllianceWallTargetLarge = new WorldObject("BlueAllianceWallTargetLarge","10", new Pose2d(-72.0, 41.5, 0), new WorldObjectSize(5.0, 5.0));
        WorldObject blueAllianceWallTargetSmall = new WorldObject("BlueAllianceWallTargetSmall","9", new Pose2d(-72.0, 36, 0), new WorldObjectSize(2.0, 2.0));

        this.setValue(blueAllianceWallTargetLarge);
        this.setValue(blueAllianceWallTargetSmall);
/////////////////////////////////////////////////////////////////////


/////////////////////Spike position///////////////////////
        //blue alliance audience spike positions
        WorldObject blueAllianceAudienceSpikeLeft = new WorldObject("BlueAllianceAudienceSpikeLeft","BlueAllianceAudienceSpikeLeft", new Pose2d(-24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceAudienceSpikeCenter = new WorldObject("BlueAllianceAudienceSpikeCenter","BlueAllianceAudienceSpikeCenter", new Pose2d(-36, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceAudienceSpikeRight = new WorldObject("BlueAllianceAudienceSpikeRight","BlueAllianceAudienceSpikeRight", new Pose2d(-48, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceAudienceSpikeLeft);
        this.setValue(blueAllianceAudienceSpikeCenter);
        this.setValue(blueAllianceAudienceSpikeRight);


        //blue alliance backstage spike positions
        WorldObject blueAllianceBackstageSpikeLeft = new WorldObject("BlueAllianceBackstageSpikeLeft","BlueAllianceBackstageSpikeLeft", new Pose2d(24, 30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceBackstageSpikeCenter = new WorldObject("BlueAllianceBackstageSpikeCenter","BlueAllianceBackstageSpikeCenter", new Pose2d(12, 24), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceBackstageSpikeRight = new WorldObject("BlueAllianceBackstageSpikeRight","BlueAllianceBackstageSpikeRight", new Pose2d(0, 30), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceBackstageSpikeLeft);
        this.setValue(blueAllianceBackstageSpikeCenter);
        this.setValue(blueAllianceBackstageSpikeRight);


        //red alliance audience spike positions
        WorldObject redAllianceAudienceSpikeLeft = new WorldObject("RedAllianceAudienceSpikeLeft","RedAllianceAudienceSpikeLeft", new Pose2d(-48.0, -30.0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceAudienceSpikeCenter = new WorldObject("RedAllianceAudienceSpikeCenter","RedAllianceAudienceSpikeCenter", new Pose2d(-36, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceAudienceSpikeRight = new WorldObject("RedAllianceAudienceSpikeRight","RedAllianceAudienceSpikeRight", new Pose2d(-24, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceAudienceSpikeLeft);
        this.setValue(redAllianceAudienceSpikeCenter);
        this.setValue(redAllianceAudienceSpikeRight);


        //red alliance backstage spike positions
        WorldObject redAllianceBackstageSpikeLeft = new WorldObject("RedAllianceBackstageSpikeLeft","RedAllianceBackstageSpikeLeft", new Pose2d(0, -30), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceBackstageSpikeCenter = new WorldObject("RedAllianceBackstageSpikeCenter","RedAllianceBackstageSpikeCenter", new Pose2d(12.0, -24), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceBackstageSpikeRight = new WorldObject("RedAllianceBackstageSpikeRight","RedAllianceBackstageSpikeRight", new Pose2d(24.0, -30), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceBackstageSpikeLeft);
        this.setValue(redAllianceBackstageSpikeCenter);
        this.setValue(redAllianceBackstageSpikeRight);

  ///////////////////////////Staging positions
        //Blue Alliance Staging positions
        WorldObject blueAllianceOneLeft = new WorldObject("BlueAllianceOneLeft", StagingPosition.BLUE_ONE_LEFT.toString(), new RelativePosition(1,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceOneCenter = new WorldObject("BlueAllianceOneCenter",StagingPosition.BLUE_ONE_CENTER.toString(), new RelativePosition(1,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceOneRight = new WorldObject("BlueAllianceOneRight",StagingPosition.BLUE_ONE_RIGHT.toString(), new RelativePosition(1,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceOneLeft);
        this.setValue(blueAllianceOneCenter);
        this.setValue(blueAllianceOneRight);


        WorldObject blueAllianceOneTwoCenter = new WorldObject("BlueAllianceOneTwoCenter",StagingPosition.BLUE_ONE_TWO_CENTER.toString(), new RelativePosition(1,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceOneTwoCenter);


        WorldObject blueAllianceTwoLeft = new WorldObject("BlueAllianceTwoLeft",StagingPosition.BLUE_TWO_LEFT.toString(), new RelativePosition(2,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceTwoCenter = new WorldObject("BlueAllianceTwoCenter",StagingPosition.BLUE_TWO_CENTER.toString(), new RelativePosition(2,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceTwoRight = new WorldObject("BlueAllianceTwoRight",StagingPosition.BLUE_TWO_RIGHT.toString(), new RelativePosition(2,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceTwoLeft);
        this.setValue(blueAllianceTwoCenter);
        this.setValue(blueAllianceTwoRight);


        WorldObject blueAllianceTwoThreeCenter = new WorldObject("BlueAllianceTwoThreeCenter",StagingPosition.BLUE_TWO_THREE_CENTER.toString(), new RelativePosition(2,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceTwoThreeCenter);


        WorldObject blueAllianceThreeLeft = new WorldObject("BlueAllianceThreeLeft",StagingPosition.BLUE_THREE_LEFT.toString(), new RelativePosition(3,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject blueAllianceThreeCenter = new WorldObject("BlueAllianceThreeCenter",StagingPosition.BLUE_THREE_CENTER.toString(), new RelativePosition(3,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject blueAllianceThreeRight = new WorldObject("BlueAllianceThreeRight",StagingPosition.BLUE_THREE_RIGHT.toString(), new RelativePosition(3,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(blueAllianceThreeLeft);
        this.setValue(blueAllianceThreeCenter);
        this.setValue(blueAllianceThreeRight);



        //Red Alliance Staging positions
        WorldObject redAllianceFourLeft = new WorldObject("RedAllianceFourLeft",StagingPosition.RED_FOUR_LEFT.toString(), new RelativePosition(4,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceFourCenter = new WorldObject("RedAllianceFourCenter",StagingPosition.RED_FOUR_CENTER.toString(), new RelativePosition(4,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceFourRight = new WorldObject("RedAllianceFourRight",StagingPosition.RED_FOUR_RIGHT.toString(), new RelativePosition(4,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceFourLeft);
        this.setValue(redAllianceFourCenter);
        this.setValue(redAllianceFourRight);


        WorldObject redAllianceFourFiveCenter = new WorldObject("RedAllianceFourFiveCenter",StagingPosition.RED_FOUR_FIVE_CENTER.toString(), new RelativePosition(4,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceFourFiveCenter);


        WorldObject redAllianceFiveLeft = new WorldObject("RedAllianceFiveLeft",StagingPosition.RED_FIVE_LEFT.toString(), new RelativePosition(5,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceFiveCenter = new WorldObject("RedAllianceFiveCenter",StagingPosition.RED_FIVE_CENTER.toString(), new RelativePosition(5,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceFiveRight = new WorldObject("RedAllianceFiveRight",StagingPosition.RED_FIVE_RIGHT.toString(), new RelativePosition(5,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceFiveLeft);
        this.setValue(redAllianceFiveCenter);
        this.setValue(redAllianceFiveRight);

        WorldObject redAllianceFiveSixCenter = new WorldObject("RedAllianceFiveSixCenter",StagingPosition.RED_FIVE_SIX_CENTER.toString(), new RelativePosition(5,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceFiveSixCenter);

        WorldObject redAllianceSixLeft = new WorldObject("RedAllianceSixLeft",StagingPosition.RED_SIX_LEFT.toString(), new RelativePosition(6,20,0,0), new WorldObjectSize(10.5, 5.50));
        WorldObject redAllianceSixCenter = new WorldObject("RedAllianceSixCenter",StagingPosition.RED_SIX_CENTER.toString(), new RelativePosition(6,20,0,0), new WorldObjectSize(20.5, 10.50));
        WorldObject redAllianceSixRight = new WorldObject("RedAllianceSixRight",StagingPosition.RED_SIX_RIGHT.toString(), new RelativePosition(6,20,0,0), new WorldObjectSize(30.5, 15.50));

        this.setValue(redAllianceSixLeft);
        this.setValue(redAllianceSixCenter);
        this.setValue(redAllianceSixRight);





        ///////////////////

    */

    }
}
