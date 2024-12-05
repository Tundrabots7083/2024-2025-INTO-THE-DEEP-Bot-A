package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ftc7083.filter.Pose2DMovingAverageFilter;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Arrays;
import java.util.List;

/**
 * A test TeleOp OpMode that gets the moving average values for each webcam, as well as an
 * average of the values from both webcams. This is intended as a way to determine the accuracy
 * of the webcams and how effective the April Tag detections are for localization.
 */
@TeleOp(name = "April Tag Localization Test", group = "tests")
public class AprilTagLocalizationTest extends OpMode {

    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private Pose2DFilter average;
    private List<Webcam> webcams;
    private double totalLoops = 0.0;
    private double totalLoopTime = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        average = new Pose2DFilter(telemetry);

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
        average.addTelemetry();

        double elapsedTime = timer.time();
        totalLoopTime += elapsedTime;
        totalLoops++;

        telemetry.addLine(" ");
        telemetry.addData("avg loop time (millis)", totalLoopTime / totalLoops);
        telemetry.addData("loop time (millis)", String.format("%.2f", elapsedTime));
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
     * Update the measurements for the webcams, as well as the average values for all webcams.
     */
    @SuppressLint("DefaultLocale")
    private void updateWebcams() {
        double totalX = 0.0;
        double totalY = 0.0;
        double totalH = 0.0;
        double totalDetections = 0.0;

        // Get the April Tag detections for each webcam
        for (Webcam webcam : webcams) {
            int numWebcamDetections = 0;
            List<AprilTagDetection> detections = webcam.getDetections();
            if (!detections.isEmpty()) {
                // Get the total X, Y and heading values for all April Tags detected by the webcam
                for (AprilTagDetection detection : detections) {
                    if (detection.robotPose != null) {
                        numWebcamDetections++;

                        // Get the x-axis, y-axis and heading from the April Tag detection
                        double x = detection.robotPose.getPosition().x;
                        double y = detection.robotPose.getPosition().y;
                        double h = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

                        // Update the total April Tag detections by all webcams
                        totalDetections++;
                        totalX += x;
                        totalY += y;
                        totalH += h;
                    }
                }
            }
            telemetry.addData("[" + webcam.getLocation().webcamName() + " (" + webcam.getLocation().name() + ")] detections", numWebcamDetections);
        }

        // Update the average values for the webcams, if at least one webcam can see an April Tag
        if (totalDetections > 0) {
            double avgX = totalX / totalDetections;
            double avgY = totalY / totalDetections;
            double avgH = totalH / totalDetections;
            average.addMeasurement(new Pose2D(avgX, avgY, avgH));
        } else {
            average.noMeasurement();
        }
    }

    /**
     * Class to manage the Moving Average Filter for Pose2D measurements.
     */
    @Config
    public static class Pose2DFilter {
        // Include debug telemetry data
        public static boolean DEBUG = false;

        // Moving Average Filter configuration
        public static int WINDOW_SIZE = 40;
        public static int MIN_NUM_SAMPLES = 15;
        public static int INVALID_PACKETS_BEFORE_CLEARING = 8;

        // Allowable tolerances for error in the X-axis, Y-axis, and heading measurements
        public static double X_TOLERABLE_ERROR = 0.5; // inches
        public static double Y_TOLERABLE_ERROR = 0.5; // inches
        public static double H_TOLERABLE_ERROR = 1.0; // degrees

        private final Telemetry telemetry;
        private final Pose2DMovingAverageFilter average = new Pose2DMovingAverageFilter(MIN_NUM_SAMPLES, WINDOW_SIZE);
        private int invalidPackets = 0;
        private int numTimesCleared = 0;

        /**
         * Instantiates a new Pose2D filter.
         *
         * @param telemetry used to output data to the user
         */
        public Pose2DFilter(@NonNull Telemetry telemetry) {
            this.telemetry = telemetry;
        }

        /**
         * Adds a new Pose2D to the Moving Average Filter.
         *
         * @param pose the Pose2D to add to the Moving Average Filter
         */
        public void addMeasurement(@NonNull Pose2D pose) {
            Pose2D mean = average.getMean();
            double diffX = Math.abs(pose.x - mean.x);
            double diffY = Math.abs(pose.y - mean.y);
            double diffH = Math.abs(pose.h - mean.h);

            // Only add the Pose2D if either a) there aren't enough measurements to calculate the
            // mean yet, or b) the measurement is sufficiently close to the mean that it should
            // be included
            if (!average.hasMean()
                    || (diffX <= X_TOLERABLE_ERROR
                        && diffY <= Y_TOLERABLE_ERROR
                        && Math.toDegrees(diffH) <= H_TOLERABLE_ERROR)) {
                average.filter(pose);
                invalidPackets = 0;
            } else {
                invalidMeasurement();
            }

            // Add some telemetry data for debug purposes
            if (DEBUG) {
                telemetry.addData("Diff X", round(diffX));
                telemetry.addData("Diff Y", round(diffY));
                telemetry.addData("Diff H", round(Math.toDegrees(diffH)));
            }
        }

        /**
         * Removes the oldest measurement from the Moving Average filter.
         */
        public void removeMeasurement() {
            if (average.numMeasurements() != 0) {
                average.removeMeasurement();
                invalidPackets++;
                clearIfInvalid();
            }
        }

        /**
         * Increases the number of invalid measurements received without a valid measurement being
         * detected. If a sufficient number of invalid measurements are received in a row then
         * the moving average filter is cleared of all entries, which is intended to quickly reset
         * the queue when the robot moves rapidly.
         */
        public void invalidMeasurement() {
            removeMeasurement();
        }

        /**
         * No measurement is detected. This is an alias for <code>invalidMeasurement</code>.
         */
        public void noMeasurement() {
            invalidMeasurement();
        }

        /**
         * Removes all entries from the Moving Average Filter if there have been enough invalid
         * or missing entries in a row. This allows the filter to be reset more rapidly if there
         * when no measurements can be made.
         */
        private void clearIfInvalid() {
            if (invalidPackets >= INVALID_PACKETS_BEFORE_CLEARING) {
                average.clear();
                numTimesCleared++;
                invalidPackets = 0;
            }
        }

        /**
         * Gets the mean (average) from the Moving Average Filter.
         *
         * @return the mean (average) from the Moving Average Filter
         */
        public Pose2D getMean() {
            return average.getMean();
        }

        /**
         * Send telemetry data for the webcams.
         */
        @SuppressLint("DefaultLocale")
        public void addTelemetry() {
            // Get the average measurements for all webcams
            telemetry.addLine(" ");
            if (average.hasMean()) {
                Pose2D mean = average.getMean();
                telemetry.addData("X", round(mean.x));
                telemetry.addData("Y", round(mean.y));
                telemetry.addData("H", round((Math.toDegrees(mean.h))));
            } else {
                telemetry.addLine(String.format("Averages not available, measurements=%d", average.numMeasurements()));
            }
            telemetry.addData("measurements", average.numMeasurements());
            telemetry.addData("invalid", invalidPackets);
            telemetry.addData("clearing", numTimesCleared);
        }

        /**
         * Helper method to round a double value to two decimal places.
         *
         * @param value the double value to round
         * @return the rounded double value
         */
        private double round(double value) {
            return Math.round(value * 100.0) / 100.0;
        }
    }
}
