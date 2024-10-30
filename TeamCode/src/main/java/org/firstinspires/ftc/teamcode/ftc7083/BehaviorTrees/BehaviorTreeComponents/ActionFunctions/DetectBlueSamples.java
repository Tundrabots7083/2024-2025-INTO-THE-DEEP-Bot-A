package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.ActionFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

public class DetectBlueSamples implements ActionFunction {
    private Limelight limelight;
    private Telemetry telemetry;
    private Limelight.TargetHeight targetHeight;
    private int count=0;
    private double xDistance;
    private double Tx;

    public DetectBlueSamples(Telemetry telemetry, Limelight limelight, Limelight.TargetHeight targetHeight) {
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.targetHeight = targetHeight;
    }

    public Status perform(BlackBoardSingleton blackBoard) {

        telemetry.addData("DetectAprilTags "," perform count: %d", count);
        telemetry.update();
        count++;

        limelight.detectBlue();
        limelight.execute();

        LLResult result = limelight.getResult();

        if(result != null){
            xDistance = limelight.getDistance(targetHeight);
            Tx = limelight.getTx();

            blackBoard.setValue("xDistanceToSample", xDistance);
            blackBoard.setValue("Tx",Tx);

            telemetry.addData("[DetectSamples]","X-Distance to Sample: %f",xDistance);
            telemetry.update();
        } else {
            blackBoard.setValue("xDistanceToSample", "No detected samples");
        }
        return Status.SUCCESS;
    }


}
