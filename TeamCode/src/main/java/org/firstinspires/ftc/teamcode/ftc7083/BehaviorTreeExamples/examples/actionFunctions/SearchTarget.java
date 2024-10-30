package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class SearchTarget implements ActionFunction {
    private static DriveTrain driveTrain;
    private double yawStep = 0.5; // check value later
    protected LinearOpMode opMode;
    public SearchTarget(LinearOpMode opMode) {
        this.opMode=opMode;
        this.driveTrain = new DriveTrain(opMode);
    }

    public Status perform(GlobalStoreSingleton globalStore) {
        this.yawStep = (double) globalStore.getValue("YawStep");

        List<AprilTagDetection>  currentDetections =  (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");
        int referenceTagId = (int) globalStore.getValue("ReferenceAprilTagId");

        //prevent searching as required by other Actions to complete (e.g ScoreBoardPixel)
        if (referenceTagId < 0){
            return Status.SUCCESS;
        }

        AprilTagDetection  referenceTag =currentDetections !=null ? currentDetections.stream()
                .filter(detection -> detection.metadata != null && detection.id == referenceTagId).findFirst( ).orElse(null)
                : null;

        if(referenceTag == null) {
            driveTrain.moveRobot(0, 0, yawStep);

            opMode.sleep(10);
            opMode.telemetry.addData("SearchTarget ", "perform referenceTagId %d not found",referenceTagId);
            opMode.telemetry.update();
            return Status.FAILURE;
        } else {
            opMode.telemetry.addData("SearchTarget ", "Search ID: %d found referenceTag %d",referenceTagId,referenceTag.id);
            opMode.telemetry.update();
            return Status.SUCCESS;
        }
    }
}