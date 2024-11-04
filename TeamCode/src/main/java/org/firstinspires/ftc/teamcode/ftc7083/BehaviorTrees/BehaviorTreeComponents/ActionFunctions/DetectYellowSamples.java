package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.ActionFunctions;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.BlackBoardSingleton;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Limelight;

public class DetectYellowSamples implements ActionFunction {
    private Limelight limelight;
    private Telemetry telemetry;
    private Limelight.TargetHeight targetHeight;
    private int count=0;
    private int retryCount = 0;
    private double xDistance;
    private double Tx;

    public DetectYellowSamples(Telemetry telemetry, Limelight limelight, Limelight.TargetHeight targetHeight) {
        this.limelight = limelight;
        this.telemetry = telemetry;
        this.targetHeight = targetHeight;
    }

    public Status perform(BlackBoardSingleton blackBoard) {

        telemetry.addData("[DetectYellowSamples]"," perform count: %d", count);
        telemetry.update();
        count++;

        limelight.detectYellow();
        limelight.execute();

        LLResult result = limelight.getResult();

        if((result != null) && (limelight.getTx() != null) && (limelight.getDistance(targetHeight) != null)){
            xDistance = (double)limelight.getDistance(targetHeight);
            Tx = result.getTx();

            blackBoard.setValue("xDistanceToSample", xDistance);
            blackBoard.setValue("Tx",Tx);

            telemetry.addData("[DetectYellowSamples] X-Distance to Sample:",xDistance);
            telemetry.addData("[DetectYellowSamples]","Tx-Angle from Sample: %f",Tx);
            telemetry.update();

            retryCount = 0;

            return Status.SUCCESS;
        } else {
            blackBoard.setValue("xDistanceToSample", null);
            blackBoard.setValue("Tx",null);

            telemetry.addData("[DetectYellowSamples]","Didn't detect anything");
            telemetry.update();

            if(retryCount >= 20) {
                return Status.FAILURE;
            } else {
                retryCount++;
                return Status.RUNNING;
            }
        }
    }


}
