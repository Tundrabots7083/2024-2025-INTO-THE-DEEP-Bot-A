package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.action.WaitAction;
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
    public static long CLAW_WAIT_TIME = 250; // milliseconds
    public static String CLAW_SERVO = "clawServo";
    // Make default open/close degrees settable by FTC dashboard
    public static double DEFAULT_OPEN_DEGREES = 180.0;
    public static double DEFAULT_CLOSE_DEGREES = 20.0;
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
     * Gets an action to open the claw. This action does not wait for the claw to be successfully
     * opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClaw() {
        return new OpenClaw(this);
    }

    /**
     * Gets an action to open the claw. This action waits for the claw to be successfully opened.
     *
     * @return an action to open the claw
     */
    public ActionEx actionOpenClawWithWait() {
        return new SequentialAction(
                new OpenClaw(this),
                new WaitAction(CLAW_WAIT_TIME)
        );
    }

    /**
     * Gets an action to close the claw. This action does not wait for the claw to be successfully
     * closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClaw() {
        return new CloseClaw(this);
    }

    /**
     * Gets an action to close the claw. This action waits for the claw to be successfully closed.
     *
     * @return an action to close the claw
     */
    public ActionEx actionCloseClawWithWait() {
        return new SequentialAction(
                new CloseClaw(this),
                new WaitAction(CLAW_WAIT_TIME)
        );
    }

    /**
     * An action that opens the claw.
     */
    public static class OpenClaw extends ActionExBase {
        private final Claw claw;
        private boolean initialized = false;

        /**
         * Instantiates a new action to open the claw.
         *
         * @param claw the claw to be opened
         */
        public OpenClaw(Claw claw) {
            this.claw = claw;
        }

        /**
         * Opens the claw and completes execution.
         *
         * @param telemetryPacket telemetry used for output of data to the user
         * @return <code>false</code>, which causes the action to complete
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            return false;
        }

        /**
         * Initializes the claw, setting the claw to the <code>opened</code> position.
         */
        private void initialize() {
            claw.open();
            initialized = true;
        }
    }

    /**
     * An action that closes the claw.
     */
    public static class CloseClaw extends ActionExBase {
        private final Claw claw;
        private boolean initialized = false;

        public CloseClaw(Claw claw) {
            this.claw = claw;
        }

        /**
         * Closes the claw and completes execution.
         *
         * @param telemetryPacket telemetry used for output of data to the user
         * @return <code>false</code>, which causes the action to complete
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            return false;
        }

        /**
         * Initializes the claw, setting the claw to the <code>closed</code> position.
         */
        private void initialize() {
            claw.close();
            initialized = true;
        }
    }
}
