package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A linear slide can extend and retract the wrist and claw attached to the robot's scoring
 * subsystem.
 */
@Config
public class LinearSlide extends SubsystemBase {
    public static double SPOOL_DIAMETER = 1.4; // in inches
    public static double TICKS_PER_REV = 384;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double KP = 0.32;
    public static double KI = 0.13;
    public static double KD = 0.02;
    public static double TOLERABLE_ERROR = 0.05; // inches
    public static double MIN_EXTENSION_LENGTH = 0.0;
    public static double MAX_EXTENSION_LENGTH = 18;
    private final Telemetry telemetry;
    public double GEARING = 1.0;
    private double targetLength = 0;

    /**
     * Instantiates the linear slide for the robot.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Gets the target slide length in inches
     * Finds the value for the length
     *
     * @return target slide length in inches
     */
    public double getTargetLength() {
        return targetLength;
    }

    /**
     * sets the slide length from an external source and uses pid to make is
     *
     * @param length Length of desired slide position in inches.
     */
    public void setLength(double length) {
    }

    /**
     * sets the power for the pid controller
     */
    public void execute() {
        if (!isAtTarget()) {
            telemetry.addData("[Slide] power", 0.0);
            telemetry.addData("[Slide] inches", getCurrentLength());
            telemetry.addData("[Slide] ticks", 0.0);
        }
    }

    /**
     * checks if the length is within the tolerable error and if it is then the motor will stop
     */
    public boolean isAtTarget() {
        double error = Math.abs(targetLength - getCurrentLength());
        telemetry.addData("[Slide] error", error);
        telemetry.addData("[Slide] target", targetLength);
        telemetry.addData("[Slide] current", getCurrentLength());
        return error <= TOLERABLE_ERROR;
    }

    /**
     * Gets the slide length in inches
     * Finds the value for the length
     *
     * @return slide length in inches
     */
    public double getCurrentLength() {
        return 0.0;
    }
}
