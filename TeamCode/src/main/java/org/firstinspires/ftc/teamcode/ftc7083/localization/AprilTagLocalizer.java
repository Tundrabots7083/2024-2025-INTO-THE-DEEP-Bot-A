package org.firstinspires.ftc.teamcode.ftc7083.localization;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * An AprilTagLocalizer gets the Pose2d and PoseVelocity2d of the robot using one or more
 * webcams. The webcams determine the values by looking for April Tags on the field and using
 * the distance and orientation of the robot using values returned by the FTC SDK April Tag
 * vision processor.
 */
public class AprilTagLocalizer implements Localizer {
    private final ElapsedTime timer = new ElapsedTime();
    private final List<Webcam> webcams;
    private Pose2d lastPose;
    private Pose2d currentPose;
    private double elapsedTime = 0.0;
    private boolean aprilTagsDetected = false;

    /**
     * Instantiates a new April Tag localizer. The localizer uses the webcams on the robot to
     * determine its position on the field and its velocity.
     */
    public AprilTagLocalizer(List<Webcam> webcams) {
        this.webcams = webcams;
        this.currentPose = new Pose2d(0, 0, 0);
        this.lastPose = currentPose;
    }

    /**
     * Gets an indicator as to whether any April Tags were detected.
     *
     * @return <code>true</code> if at least one April Tag was detected;
     *         <code>false</code> if no April Tags were detected
     */
    public boolean aprilTagsDetected() {
        return aprilTagsDetected;
    }

    @Override
    public void update() {
        updatePose();

        elapsedTime = timer.time();
        timer.reset();
    }

    @Override
    public Pose2d getPose2d() {
        return currentPose;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void setPose2d(Pose2d pose) {
        Telemetry telemetry = Robot.getInstance().telemetry;
        lastPose = currentPose;
        currentPose = pose;
    }

    @Override
    public PoseVelocity2d getVelocity() {
        return new PoseVelocity2d(getLinearVelocity(), getAngularVelocity());
    }

    /**
     * Calculates and returns the linear velocity of the robot. This is the velocity along the X
     * and Y axis.
     *
     * @return the linear velocity of the robot
     */
    private Vector2d getLinearVelocity() {
        if (elapsedTime == 0.0) {
            return new Vector2d(0.0, 0.0);
        }

        double xVel = Math.abs(currentPose.position.x - lastPose.position.x) / elapsedTime;
        double yVel = Math.abs(currentPose.position.y - lastPose.position.y) / elapsedTime;
        return new Vector2d(xVel, yVel);
    }

    /**
     * Calculates and returns the angular velocity of the robot. This is the rate of change in the
     * heading between the last two updates of the robot's position and heading.
     *
     * @return the angular velocity of the robot
     */
    private double getAngularVelocity() {
        if (elapsedTime == 0.0) {
            return 0.0;
        }

        return currentPose.heading.toDouble() / elapsedTime;
    }

    /**
     * Updates the pose of the robot using each webcam on the robot. For each webcam, it gets the
     * April Tags detected by that camera and determines the pose and orientation of the robot.
     * The pose is then averaged over all April Tag detections for each webcam attached to the
     * robot.
     */
    @SuppressLint("DefaultLocale")
    private void updatePose() {
        Telemetry telemetry = Robot.getInstance().telemetry;

        double totalX = 0.0;
        double totalY = 0.0;
        double totalHeading = 0.0;

        // Get the total number of April Tag detections and the X, Y and heading for the robot as
        // determined by that detection.
        int numDetections = 0;
        for (Webcam webcam : webcams) {
            telemetry.addLine(String.format("\nWebcam %s", webcam));

            ArrayList<AprilTagDetection> detections = webcam.getDetections();
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    Position position = detection.robotPose.getPosition();
                    totalX += position.x;
                    totalY += position.y;

                    YawPitchRollAngles orientation = detection.robotPose.getOrientation();
                    totalHeading += orientation.getYaw();

                    numDetections++;

                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("X=%6.1f Y=%6.1f Z=%6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("P=%6.1f R=%6.1f Y=%6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
        }

        aprilTagsDetected = numDetections > 0;

        // Take the average of all April tag detections and save as the current pose for the robot
        if (aprilTagsDetected) {
            Pose2d pose = new Pose2d(
                    new Vector2d(totalX / numDetections, totalY / numDetections)
                    , new Rotation2d(totalHeading / numDetections, 0.0)
            );
            lastPose = currentPose;
            currentPose = pose;
            telemetry.addLine(String.format("\nPose X=%6.1f Y=%6.1f H=%6.1f", pose.position.x, pose.position.y, pose.heading.toDouble()));
        }
    }
}
