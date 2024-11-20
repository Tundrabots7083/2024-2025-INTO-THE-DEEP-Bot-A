package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2dMovingAverageFilter;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A test TeleOp OpMode that gets the moving average values for each webcam, as well as an
 * average of the values from both webcams. This is intended as a way to determine the accuracy
 * of the webcams and how effective the April Tag detections are for localization.
 */
@Config
@TeleOp(name = "April Tag Localization Test", group = "tests")
public class AprilTagLocalizationTest extends OpMode {
    private static int WINDOW_SIZE = 9;
    private static int MIN_NUM_SAMPLES = 5;

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final Pose2dMovingAverageFilter webcamAverage = new Pose2dMovingAverageFilter(MIN_NUM_SAMPLES, WINDOW_SIZE);
    private List<Webcam> webcams;
    private List<Pose2dMovingAverageFilter> webcamFilters;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires the code
        // to clear the cache at the start of each loop.
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
        Webcam leftWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.LEFT, viewIds[0]);
        Webcam rightWebcam = new Webcam(hardwareMap, telemetry, Webcam.Location.RIGHT, viewIds[1]);
        webcams = Arrays.asList(leftWebcam, rightWebcam);

        webcamFilters = new ArrayList<>(webcams.size());
        for (int i = 0; i < webcams.size(); i++) {
            webcamFilters.add(new Pose2dMovingAverageFilter(MIN_NUM_SAMPLES, WINDOW_SIZE));
        }

        // Wait for both cameras to initialize
        while (!leftWebcam.webcamInitialized() || !rightWebcam.webcamInitialized()) {
            // NO-OP
        }

        // Wait for the DS start button to be touched.
        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateWebcams();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        timer.reset();

        updateWebcams();
        telemetryAprilTag();

        telemetry.addLine(" ");
        telemetry.addLine(String.format("loop time (millis)=%.2f", timer.time()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save more CPU resources when camera is no longer needed.
        for (Webcam webcam : webcams) {
            webcam.close();
        }
    }

    /**
     * Send telemetry data for the webcams.
     */
    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        // Loop through each webcam and output the average measurements for that webcam
        for (int i = 0; i < webcamFilters.size(); i++) {
            Webcam webcam = webcams.get(i);
            Pose2dMovingAverageFilter webcamFilter = webcamFilters.get(i);

            telemetry.addLine(String.format("Webcam %s (%s)", webcam.getLocation().webcamName(), webcam.getLocation()));
            if (!webcamFilter.hasAverage()) {
                Pose2d pose = webcamFilter.getAverage();
                telemetry.addLine(String.format("\tX=%.2f, Y=%.2f, H=%.2f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            } else {
                telemetry.addLine(String.format("\tApril Tags not available, num_measurements=%d", webcamFilter.numMeasurements()));
            }
        }

        // Get the average measurements for all webcams
        if (!webcamAverage.hasAverage()) {
            Pose2d pose = webcamAverage.getAverage();
            telemetry.addLine(String.format("Averages: X=%.2f, Y=%.2f, H=%.2f", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble())));
            telemetry.addData("Avg X", pose.position.x);
            telemetry.addData("Avg Y", pose.position.y);
            telemetry.addData("Avg H", Math.toDegrees(pose.heading.toDouble()));
        } else {
            telemetry.addLine(String.format("Averages not available, num_measurements=%d", webcamAverage.numMeasurements()));
        }

        telemetry.update();
    }

    /**
     * Update the measurements for the webcams, as well as the average values for all webcams.
     */
    @SuppressLint("DefaultLocale")
    private void updateWebcams() {
        double totalX = 0;
        double totalY = 0;
        double totalHeading = 0;
        int totalNumMeasurements = 0;

        // Update the moving averages for each webcam. This will weight each April Tag seen by the
        // webcam equally.
        for (int i = 0; i < webcams.size(); i++) {
            Webcam webcam = webcams.get(i);
            Pose2dMovingAverageFilter webcamFilter = webcamFilters.get(i);

            List<AprilTagDetection> detections = webcam.getDetections();
            if (detections.isEmpty()) {
                webcamFilter.removeMeasurement();
            } else {
                // Get the total X, Y and heading values for all April Tags detected by the webcam
                double webcamX = 0;
                double webcamY = 0;
                double webcamHeading = 0;
                for (AprilTagDetection detection : detections) {
                    webcamX += detection.ftcPose.x;
                    webcamY += detection.ftcPose.y;
                    webcamHeading += detection.ftcPose.yaw;
                }

                // Update the moving average for the webcam
                double avgX = webcamX / (double) detections.size();
                double avgY = webcamY / (double) detections.size();
                double avgHeading = webcamHeading / (double) detections.size();
                Pose2d pose = webcamFilter.filter(new Pose2d(avgX, avgY, avgHeading));

                // Keep track of the total average values for all webcams that can see an April Tag.
                // This will weight each webcam equally.
                totalNumMeasurements++;
                totalX += pose.position.x;
                totalY += pose.position.y;
                totalHeading += Math.toRadians(pose.heading.toDouble());
            }
        }

        // Update the average values for the webcams, if at least one webcam can see an April Tag
        if (totalNumMeasurements > 0) {
            double avgX = totalX / (double) totalNumMeasurements;
            double avgY = totalY / (double) totalNumMeasurements;
            double avgHeading = totalHeading / (double) totalNumMeasurements;
            webcamAverage.filter(new Pose2d(avgX, avgY, avgHeading));
        } else {
            webcamAverage.removeMeasurement();
        }
    }
}
