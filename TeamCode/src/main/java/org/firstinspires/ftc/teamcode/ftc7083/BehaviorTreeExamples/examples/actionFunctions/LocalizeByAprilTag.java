package org.firstinspires.ftc.teamcode.ftc7083.BehaviorTreeExamples.examples.actionFunctions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.behaviorTree.general.ActionFunction;
import org.firstinspires.ftc.teamcode.behaviorTree.general.GlobalStoreSingleton;
import org.firstinspires.ftc.teamcode.behaviorTree.general.Status;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldModel;
import org.firstinspires.ftc.teamcode.models.worldModel.WorldObject;
import org.firstinspires.ftc.teamcode.subsystems.CenterStageORMechanumDriveSingleton;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class LocalizeByAprilTag implements ActionFunction {
    private WorldModel worldModel;
    private AprilTagDetection currentDetections;
    private  int desiredTagId=-1;
    private AprilTagDetection referenceTag = null;

    private List<Pose2d> cameraAbsolutePoses = new ArrayList<Pose2d>();
    CenterStageORMechanumDriveSingleton drive = null;

    private LinearOpMode opMode;

    public LocalizeByAprilTag(LinearOpMode opMode) {
        this.opMode=opMode;
    }

    public Status perform(GlobalStoreSingleton globalStore) {
       setCameraAbsolutePoses(globalStore);

       //Calculate the estimated camera pose as the average of calculated poses
        Pose2d estCameraPosesSum = cameraAbsolutePoses.stream().reduce(new Pose2d(0,0,0),(subtotal, element)->subtotal.plus(element));
        Pose2d estCameraPose = estCameraPosesSum.div(cameraAbsolutePoses.size());

        // Pose2d estCameraPose = cameraAbsolutePoses.get(0);

        Pose2d estRobotPose = calculateRobotPoseFromCameraPose(estCameraPose);


        CenterStageORMechanumDriveSingleton.reset();
        this.drive = CenterStageORMechanumDriveSingleton.getInstance(opMode.hardwareMap, this.opMode, globalStore);
        //set drive starting pose
        this.drive.setPoseEstimate(estRobotPose);


        globalStore.setValue("AprilTagCameraPose", estCameraPose);
        globalStore.setValue("AprilTagRobotPose", estRobotPose);

        opMode.telemetry.addData("LocalizeByAprilTag-", "Hr: %f Xr: %f Yr: %f",
                estRobotPose.getHeading(),estRobotPose.getX(),estRobotPose.getY());
        //opMode.telemetry.addData("LocalizeByAprilTag-", "Hc: %f Xc: %f Yc: %f count: %d",
        //        estCameraPose.getHeading(),estCameraPose.getX(),estCameraPose.getY(), cameraAbsolutePoses.stream().count());
        opMode.telemetry.update();

        return Status.SUCCESS;
        //return Status.RUNNING;
    }


    private void setCameraAbsolutePoses(GlobalStoreSingleton globalStore){
        WorldModel worldModel =(WorldModel) globalStore.getValue("WorldModel");

        List<AprilTagDetection> currentDetections =  (List<AprilTagDetection>) globalStore.getValue("CurrentDetections");

        if (currentDetections == null){
            return;
        }

        List<AprilTagDetection> validCurrentDetections = currentDetections.stream().filter(detection -> (detection.metadata != null)).collect(Collectors.toList());

        if (validCurrentDetections == null){
            return;
        }

        cameraAbsolutePoses.clear();

        validCurrentDetections.forEach((AprilTagDetection robotDetection)->{


            WorldObject tagWorldObject = worldModel.getValue(String.valueOf(robotDetection.id));

            Pose2d aprilTagPose =tagWorldObject.position;

           cameraAbsolutePoses.add(calculateCameraPose(aprilTagPose, robotDetection));
        });
    }

    private Pose2d calculateCameraPose(Pose2d aprilTagPose, AprilTagDetection cameraDetection){
        double degToRadRatio = Math.PI/180;

        double R = cameraDetection.ftcPose.range;           // Range from AprilTag to robot
        double bearing = degToRadRatio * cameraDetection.ftcPose.bearing; // Bearing angle in radians
        double yaw = degToRadRatio * cameraDetection.ftcPose.yaw;   // Yaw angle in radians

        // Calculate relative position of the robot to the aprilTag
        double xRelative = R * Math.cos(bearing);
        double yRelative = R * Math.sin(bearing);
/*
        opMode.telemetry.addData("LocalizeByAprilTag-", "calculateCameraPose Id: %d",cameraDetection.id);
        opMode.telemetry.update();
        opMode.telemetry.addData("LocalizeByAprilTag-", "B: %f xRel: %f yRel: %f aprilTagPose Xr:%f; Yr:%f; Hr:%f;",bearing,xRelative,yRelative, aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getHeading());
        opMode.telemetry.update();
*/
        double cameraXAbsolute = aprilTagPose.getX() + xRelative * Math.cos(aprilTagPose.getHeading()*degToRadRatio) - yRelative * Math.sin(aprilTagPose.getHeading()*degToRadRatio);
        double cameraYAbsolute = aprilTagPose.getY() + xRelative * Math.sin(aprilTagPose.getHeading()*degToRadRatio) + yRelative * Math.cos(aprilTagPose.getHeading()*degToRadRatio);
        double cameraHeadingAbsolute = aprilTagPose.getHeading()- Math.toRadians(180) + yaw;

        Pose2d cameraAbsolutePose = new Pose2d(cameraXAbsolute,cameraYAbsolute,cameraHeadingAbsolute);
     //   opMode.telemetry.addData("LocalizeByAprilTag ","First cameraDetection range %f cameraAbsolutePose.X %f", cameraDetection.ftcPose.range, cameraAbsolutePose.getX());
     //   opMode.telemetry.update();

        return cameraAbsolutePose;
    }

    private Pose2d calculateRobotPoseFromCameraPose(Pose2d cameraPose){
        double CAMERA_X_OFFSET = 2.5625;// 2.265; // Camera x offset relative to the robot's center
        double CAMERA_Y_OFFSET =7.625;// 7.5; // Camera y offset relative to the robot's center

// Assuming cameraPose is the absolute pose of the camera
        double robotCenterX = cameraPose.getX() - CAMERA_X_OFFSET * Math.cos(cameraPose.getHeading()) + CAMERA_Y_OFFSET * Math.sin(cameraPose.getHeading());
        double robotCenterY = cameraPose.getY() - CAMERA_X_OFFSET * Math.sin(cameraPose.getHeading()) - CAMERA_Y_OFFSET * Math.cos(cameraPose.getHeading());
        double robotCenterHeading = cameraPose.getHeading(); // The heading of the robot's center is assumed to be the same as the camera's heading

// Now create a new Pose2d with the calculated components
        Pose2d robotCenterPose = new Pose2d(robotCenterX, robotCenterY, robotCenterHeading);
        return robotCenterPose;
    }
}