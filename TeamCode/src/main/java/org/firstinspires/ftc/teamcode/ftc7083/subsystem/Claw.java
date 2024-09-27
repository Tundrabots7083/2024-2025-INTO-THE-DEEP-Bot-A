package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Servo;

/**
 * The Claw class implements a claw used by the robot to pick up and score
 * samples and specimens.  By default the claw is closed or set to its zero
 * position initially.  Users can request that the claw be opened or moved
 * from its zero position to some angle specified in degrees.  The claw is
 * reset to its zero position when a close is requested.
 */
@Config
public class Claw extends SubsystemBase {
    public static String CLAW_SERVO = "clawServo";
    // Make default open/close degrees settable by FTC dashboard
    public static double DEFAULT_OPEN_DEGREES = 45.0;
    public static double DEFAULT_CLOSE_DEGREES = 0.0;
    // Make max claw degrees settable by FTC dashboard
    public static double MAX_CLAW_DEGREES = 270.0;
    // Implement the claw using a Servo class
    private final Servo servo;
    private final Telemetry telemetry;

    /**
     * Constructor
     *
     * @param hardwareMap Servo configuration information
     * @param telemetry telemetry used for output of information to the user.
     */
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        servo = new Servo(hardwareMap, CLAW_SERVO);
        servo.setMaxDegrees(MAX_CLAW_DEGREES);
        servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
        servo.setDegrees(DEFAULT_CLOSE_DEGREES);
    }

    /**
     * Sets the maximum number of degrees the servo can move to.
     *
     * @param degrees the maximum number of degrees the servo can move to.
     */
    public void setMaxDegrees(double degrees) {
        servo.setMaxDegrees(degrees);
    }

    /**
     * Opens claw to a default angle of 45 degrees from its zero position.
     *
     * @return double:  The degrees the claw was opened to.
     * Should be 45.
     */
    public double open() {
        return open(DEFAULT_OPEN_DEGREES);
    }

    /**
     * Opens the claw the number of degrees requested from its zero
     * position.
     *
     * @param degrees: Number of degrees the claw is to be opened from
     *                 its zero position.
     * @return double:  The angle in degrees that the claw was opened to.
     */
    public double open(double degrees) {
        servo.setDegrees(degrees);
        return servo.getDegrees();
    }

    /**
     * Get the current position of the claw in degrees from
     * it's start position.
     *
     * @return double: The current position of the claw in degrees.
     */
    public double getCurrentPosition() {
        return servo.getDegrees();
    }

    /**
     * Close the claw back to its zero position.
     *
     * @return double:  Degrees from claw's 0 position.
     * Should be zero.
     */
    public double close() {
        servo.setDegrees(DEFAULT_CLOSE_DEGREES);
        return servo.getDegrees();
    }
}