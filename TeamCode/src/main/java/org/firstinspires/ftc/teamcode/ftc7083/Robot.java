package org.firstinspires.ftc.teamcode.ftc7083;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;

import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
@Config
public class Robot {
    // SparkFun OTOS configuration
    private static final String OTOS_SENSOR_NAME = "otos_sensor";
    public static double OTOS_OFFSET_X = 0.25;    // For Robot A chassis
    public static double OTOS_OFFSET_Y = 0.0;
    public static double OTOS_OFFSET_H = 180.0;   // For Robot A chassis
    public static double OTOS_LINEAR_SCALAR = 1.18215;
    public static double OTOS_ANGULAR_SCALAR = 1.0126555556;

    private static Robot robot = null;

    public final Telemetry telemetry;

    // Subsystems
    public final MecanumDrive mecanumDrive;
    public final Webcam webcam;

    // Sensors
    public final SparkFunOTOS otos;

    // All lynx module huba
    public final List<LynxModule> allHubs;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     */
    private Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = this;
        this.telemetry = telemetry;

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        otos = hardwareMap.get(SparkFunOTOS.class, OTOS_SENSOR_NAME);
        configureOTOS(otos);

        if (opModeType == OpModeType.AUTO) {
            // Create the vision sensor
            webcam = new Webcam(hardwareMap, telemetry, "Webcam Front");
        } else {
            webcam = null;
        }

        this.telemetry.addLine("[Robot] initialized");
        this.telemetry.update();
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = new Robot(hardwareMap, telemetry, opModeType);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    /**
     * Initialize the SparkFun OTOS sensor
     *
     * @param otos the SparkFUn OTOS sensor to initialize
     */
    @SuppressLint("DefaultLocale")
    private void configureOTOS(SparkFunOTOS otos) {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. This setting is not
        // persisted in the sensor, so it must be reset each time the robot is initialized.
        // We use radians for the angular unit, as this is what RoadRunner uses for path
        // calculations.
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        // Set the offset for where the sensor is attached relative to the center of the robot.
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(OTOS_OFFSET_X, OTOS_OFFSET_Y, OTOS_OFFSET_H);
        otos.setOffset(offset);

        // To set a value, move the robot a fixed distance (say 100 inches) and measure how far
        // it has moved. If it moved 103 inches, then the linear scaler would be 100/103, or 0.971.
        // For the angular scalar, rotate the robot 10 times, and then set the angular scalr to
        // the inverse of the error.
        otos.setLinearScalar(OTOS_LINEAR_SCALAR);
        otos.setAngularScalar(OTOS_ANGULAR_SCALAR);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. The calibration must be recalculated each time the robot is
        // initialized.
        otos.calibrateImu();

        // Reset the robot tracking to the origin. This must be done each time the robot is initialized.
        otos.resetTracking();

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }

    /**
     * Gets a string representation of the robot.
     *
     * @return a string representation of the robot
     */
    @NonNull
    @Override
    public String toString() {
        return "Robot{" +
                "mecanumDrive=" + mecanumDrive +
                ", webcam=" + webcam +
                '}';
    }

    // Enum to specify opmode type
    public enum OpModeType {
        /**
         * Driver controlled OpMode
         */
        TELEOP,
        /**
         * Autonomous OpMode
         */
        AUTO
    }
}
