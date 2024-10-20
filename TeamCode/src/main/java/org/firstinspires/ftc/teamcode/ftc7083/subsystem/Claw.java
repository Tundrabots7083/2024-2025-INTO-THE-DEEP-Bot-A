package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
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
    // Delay when opening or closing the claw, to give the servo time to complete it's processing
    public static int CLAW_MOVE_DELAY_IN_MILLIS = 250;

    public static String CLAW_SERVO = "clawServo";
    // Make default open/close degrees settable by FTC dashboard
    public static double DEFAULT_OPEN_DEGREES = 180.0;
    public static double DEFAULT_CLOSE_DEGREES = 21.0;
    // Make max claw degrees settable by FTC dashboard
    public static double MAX_CLAW_DEGREES = 180.0;
    // Implement the claw using a Servo class
    private final Servo servo;
    private final Telemetry telemetry;

    /**
     * Constructor
     *
     * @param hardwareMap Servo configuration information
     * @param telemetry   telemetry used for output of information to the user.
     */
    public Claw(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        servo = new Servo(hardwareMap, CLAW_SERVO);
        servo.setMaxDegrees(MAX_CLAW_DEGREES);
        servo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        servo.setDegrees(DEFAULT_CLOSE_DEGREES);
    }

    /**
     * Opens claw to a default angle of 45 degrees from its zero position.
     *
     * @return double:  The degrees the claw was opened to.
     *         Should be 45.
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
     *         Should be zero.
     */
    public double close() {
        servo.setDegrees(DEFAULT_CLOSE_DEGREES);
        return servo.getDegrees();
    }

    /**
     * Returns an action to open the claw, and then wait for a short delay to give the claw time
     * to move to it's new position.
     *
     * @return an action to open the claw
     */
    public ActionEx openClaw() {
        return new AdjustClaw(this, DEFAULT_OPEN_DEGREES).withTimeout(CLAW_MOVE_DELAY_IN_MILLIS);
    }

    /**
     * Returns an action to close the claw, and then wait for a short delay to give the claw time
     * to move to it's new position.
     *
     * @return an action to close the claw
     */
    public ActionEx closeClaw() {
        return new AdjustClaw(this, DEFAULT_CLOSE_DEGREES).withTimeout(CLAW_MOVE_DELAY_IN_MILLIS);
    }

    /**
     * An action that sets a claw to the requested degrees.
     */
    private static class AdjustClaw extends ActionExBase {
        private final Claw claw;
        private final double degrees;
        private boolean initialized = false;

        /**
         * Instantiates an action that will set the claw to the requested degrees.
         *
         * @param claw    the claw to be acted upon
         * @param degrees the degrees to which to set the claw
         */
        public AdjustClaw(Claw claw, double degrees) {
            this.claw = claw;
            this.degrees = degrees;
        }

        /**
         * Sets the claw to the requested number of degrees. This action terminates after a single
         * run. If you want to wait for a period of time afterwards, use the <code>withWait</code>
         * method to create an action that adjusts the claw's angles and then waits for the requested
         * time before completing.
         *
         * @param telemetryPacket telemetry that may be used for output of information
         * @return <code>false</code>, indicating the action has completed it's processing.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }

            claw.execute();
            return false;
        }

        /**
         * Initializes the action. This is called the first time the action is run.
         */
        private void initialize() {
            claw.servo.setDegrees(degrees);
            initialized = true;
        }
    }
}
