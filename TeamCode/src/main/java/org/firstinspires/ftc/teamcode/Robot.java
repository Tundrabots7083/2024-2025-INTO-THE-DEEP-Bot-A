package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Webcam;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class Robot {
    private static Robot robot = null;

    public final Telemetry telemetry;

    // Subsystems
    public final MecanumDrive mecanumDrive;
    public final Webcam webcam;

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

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        if (opModeType == OpModeType.AUTO) {
            // Create the vision sensor
            webcam = new Webcam("Webcam Front", hardwareMap, telemetry);
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

    // Enum to specify opmode type
    public enum OpModeType {
        /** Driver controlled OpMode */
        TELEOP,
        /** Autonomous OpMode */
        AUTO
    }
}
