package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2DMovingAverageFilter;
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
    public static int WINDOW_SIZE = 10;
    public static int MIN_NUM_SAMPLES = 5;

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final Pose2DMovingAverageFilter webcamAverage = new Pose2DMovingAverageFilter(MIN_NUM_SAMPLES, WINDOW_SIZE);
    private List<Webcam> webcams;
    private List<Pose2DMovingAverageFilter> webcamFilters;

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
            webcamFilters.add(new Pose2DMovingAverageFilter(MIN_NUM_SAMPLES, WINDOW_SIZE));
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
        telemetry.addData("loop time (millis)", String.format("%.2f", timer.time()));
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
            Pose2DMovingAverageFilter webcamFilter = webcamFilters.get(i);

            telemetry.addLine(" ");
            telemetry.addLine(String.format("Webcam %s (%s)", webcam.getLocation().webcamName(), webcam.getLocation()));
            if (webcamFilter.hasAverage()) {
                Pose2D pose = webcamFilter.getAverage();
                telemetry.addLine(String.format("X=%.2f, Y=%.2f, H=%.2f", pose.x, pose.y, Math.toDegrees(pose.h)));
            } else {
                telemetry.addLine(String.format("April Tags not available, num_measurements=%d", webcamFilter.numMeasurements()));
            }
        }

        // Get the average measurements for all webcams
        telemetry.addLine(" ");
        if (webcamAverage.hasAverage()) {
            Pose2D pose = webcamAverage.getAverage();
            telemetry.addLine(String.format("Averages: X=%.2f, Y=%.2f, H=%.2f", pose.x, pose.y, Math.toDegrees(pose.h)));
            double roundedX = ((double) (((int) (pose.x * 100.0))) / 100.0);
            double roundedY = ((double) (((int) (pose.y * 100.0))) / 100.0);
            double roundedHeading = ((double) (((int) (Math.toDegrees(pose.h) * 100.0))) / 100.0);
            telemetry.addData("Avg X", roundedX);
            telemetry.addData("Avg Y", roundedY);
            telemetry.addData("Avg H", roundedHeading);
        } else {
            telemetry.addLine(String.format("Averages not available, num_measurements=%d", webcamAverage.numMeasurements()));
        }
    }

    /**
     * Update the measurements for the webcams, as well as the average values for all webcams.
     */
    @SuppressLint("DefaultLocale")
    private void updateWebcams() {
        double totalX = 0.0;
        double totalY = 0.0;
        double totalHeading = 0.0;
        double totalDetections = 0.0;

        // Update the moving averages for each webcam. This will weight each April Tag seen by the
        // webcam equally.
        for (int i = 0; i < webcams.size(); i++) {
            Webcam webcam = webcams.get(i);
            Pose2DMovingAverageFilter webcamFilter = webcamFilters.get(i);

            List<AprilTagDetection> detections = webcam.getDetections();
            if (detections.isEmpty()) {
                webcamFilter.removeMeasurement();
            } else {
                // Get the total X, Y and heading values for all April Tags detected by the webcam
                double totalWebcamX = 0.0;
                double totalWebcamY = 0.0;
                double totalWebcamHeading = 0.0;
                double totalWebcamDetections = 0.0;
                for (AprilTagDetection detection : detections) {
                    if (detection.robotPose != null) {
                        // Get the x-axis, y-axis and heading from the April Tag detection
                        double x = detection.robotPose.getPosition().x;
                        double y = detection.robotPose.getPosition().y;
                        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                        // Update the total detections by this webcam
                        totalWebcamDetections++;
                        totalWebcamX += x;
                        totalWebcamY += y;
                        totalWebcamHeading += heading;

                        // Update the total detections by all webcams
                        totalDetections++;
                        totalX += x;
                        totalY += y;
                        totalHeading += heading;
                    }
                }

                // Update the moving average for the webcam
                double avgWebcamX = totalWebcamX / totalWebcamDetections;
                double avgWebcamY = totalWebcamY / totalWebcamDetections;
                double avgWebcamHeading = totalWebcamHeading / totalWebcamDetections;
                webcamFilter.filter(new Pose2D(avgWebcamX, avgWebcamY, Math.toRadians(avgWebcamHeading)));
            }
        }

        // Update the average values for the webcams, if at least one webcam can see an April Tag
        if (totalDetections > 0) {
            double avgX = totalX / totalDetections;
            double avgY = totalY / totalDetections;
            double avgHeading = totalHeading / totalDetections;
            webcamAverage.filter(new Pose2D(avgX, avgY, Math.toRadians(avgHeading)));
        } else {
            webcamAverage.removeMeasurement();
        }
    }
}
