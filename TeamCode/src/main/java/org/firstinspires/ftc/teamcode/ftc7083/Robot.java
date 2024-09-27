package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
public class Robot {
    private static Robot robot = null;

    public final Telemetry telemetry;

    // Subsystems
    public final MecanumDrive mecanumDrive;
    public final IntakeSubsystem intakeSubsystem;
    public final Webcam webcam;
    public final Arm arm;
    public final LinearSlide linearSlide;
    public final Wrist wrist;
    public final Claw claw;

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
        if (opModeType == OpModeType.AUTO) {
            // Create the vision sensor
            webcam = new Webcam("Webcam Front", hardwareMap, telemetry);
        } else {
            webcam = null;
        }
        arm = new Arm(hardwareMap, telemetry);
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);

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