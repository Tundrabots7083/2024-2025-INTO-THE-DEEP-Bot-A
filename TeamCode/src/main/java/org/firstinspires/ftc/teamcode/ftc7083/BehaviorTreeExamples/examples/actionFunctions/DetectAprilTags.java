package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.ActionFunction;
import org.firstinspires.ftc.teamcode.ftc7083.BehaviorTrees.BehaviorTreeComponents.general.Status;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class DetectAprilTags implements ActionFunction {
    private final AprilTagProcessor  aprilTagProcessor;
    private final CenterStageVisionDetectorSingleton visionDetector;
    private final LinearOpMode opMode;

    private int count=0;

    public DetectAprilTags(LinearOpMode opMode) {
        this.opMode=opMode;

            this.visionDetector = CenterStageVisionDetectorSingleton.getInstance(opMode);
            this.aprilTagProcessor = visionDetector.getAprilTagProcessor();
    }

    public Status perform(GlobalStoreSingleton globalStore) {
        opMode.telemetry.addData("DetectAprilTags "," perform count: %d", count);
        opMode.telemetry.update();
        count++;

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

         if (currentDetections != null && currentDetections.stream().count() > 0) {
             opMode.telemetry.addData("DetectAprilTags ","Found %d targets", currentDetections.stream().count());
             opMode.telemetry.update();
             int referenceTagId = (int) globalStore.getValue("ReferenceAprilTagId");

             AprilTagDetection  referenceTag =currentDetections !=null ? currentDetections.stream()
                     .filter(detection -> detection.metadata != null && detection.id == referenceTagId).findFirst( ).orElse(null)
                     : null;
             /*
             if(referenceTag == null){
                 return Status.FAILURE;
             }*/

             opMode.telemetry.addData("DetectAprilTags ","First detection range %f", currentDetections.get(0).ftcPose.range);
             opMode.telemetry.update();
             globalStore.setValue("CurrentDetections", currentDetections);
             this.logDetectionsDetails(currentDetections);
             return Status.SUCCESS;
         } else {
             opMode.telemetry.addData("DetectAprilTags "," no tag found");
             opMode.telemetry.update();
             globalStore.removeValue("CurrentDetections");
             return Status.SUCCESS;
         }
    }

    private void logDetectionsDetails(List<AprilTagDetection> currentDetections ){
        currentDetections.forEach(detection -> {
            if(detection == null
                    || detection.metadata ==null
                    || detection.metadata.name ==null
                    || detection.ftcPose ==null){
                return;
            }
//            opMode.telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
            opMode.telemetry.addData("Target", "ID %d ", detection.id);
            opMode.telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
            opMode.telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
            opMode.telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
            opMode.telemetry.update();

        });

    }
}
