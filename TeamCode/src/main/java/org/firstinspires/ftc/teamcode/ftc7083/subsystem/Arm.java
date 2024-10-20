package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionExBase;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.GainSchedulingPIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.LookUpTableArgs;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * An Arm is used to move the scoring subsystem in a circular arc, allowing the robot to both
 * pickup and score sample and specimens.
 */
@Config
public class Arm extends SubsystemBase {
    public static double START_ANGLE = -50.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 1120.0; // AndyMark NeverRest ticks per rev
    public static double TOLERABLE_ERROR = 0.1; // In degrees
    public static double MIN_ANGLE = -50.0;
    public static double MAX_ANGLE = 90.0;
    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    private final double feedforward;
    private final GainSchedulingPIDController gainSchedulingPIDController;
    public double GEARING = 120.0 / 24.0;
    private double targetAngle = START_ANGLE;

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, 0);
    }

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     * @param feedforward feedforward
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry, double feedforward) {
        this.telemetry = telemetry;
        this.feedforward = feedforward;
        shoulderMotor = new Motor(hardwareMap, telemetry, "armShoulderMotor");
        configMotor(shoulderMotor);

        LookUpTableArgs[] kpLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-53, 0.01),
                new LookUpTableArgs(0, 0.04),
                new LookUpTableArgs(90, 0.019),
                new LookUpTableArgs(120, 0.025),
                new LookUpTableArgs(180, 0.032),
                new LookUpTableArgs(230, 0.01)};
        LookUpTableArgs[] kiLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-53, 0.1),
                new LookUpTableArgs(0, 0.1),
                new LookUpTableArgs(90, 0.07),
                new LookUpTableArgs(150, 0.09),
                new LookUpTableArgs(230, 0.1)};
        LookUpTableArgs[] kdLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-53, 0.006),
                new LookUpTableArgs(0, 0.0075),
                new LookUpTableArgs(90, 0.003),
                new LookUpTableArgs(150, 0.008),
                new LookUpTableArgs(230, 0.006)};

        gainSchedulingPIDController = new GainSchedulingPIDController(kpLUTArgs, kiLUTArgs, kdLUTArgs);
    }

    /**
     * Configures the shoulder motor.
     *
     * @param motor the shoulder motor for the arm
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(TICKS_PER_REV);
        motorConfigurationType.setGearing(GEARING);
        motorConfigurationType.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Gets the arm current in degrees to which the arm has moved.
     *
     * @return the current position in degrees to which the arm has moved
     */
    public double getCurrentAngle() {
        return shoulderMotor.getCurrentDegrees() + START_ANGLE;
    }

    /**
     * Gets the arm position in degrees to which the arm is moving.
     *
     * @return the target position in degrees to which the arm is moving
     */
    public double getTargetAngle() {
        return shoulderMotor.getTargetDegrees();
    }

    /**
     * Sets the shoulder motor to a position in degrees.
     */
    public void setTargetAngle(double angle) {
        double targetAngle = Range.clip(angle, MIN_ANGLE, MAX_ANGLE);
        if (this.targetAngle != targetAngle) {
            this.targetAngle = targetAngle;
            gainSchedulingPIDController.reset();
        }
    }

    /**
     * Sends power to the shoulder motor.
     */
    public void execute() {
        double degrees = shoulderMotor.getCurrentDegrees() + START_ANGLE;
        double power = gainSchedulingPIDController.calculate(targetAngle, degrees) + this.feedforward;

        shoulderMotor.setPower(power);
        telemetry.addData("[Arm] Target", targetAngle);
        telemetry.addData("[Arm] Current", degrees);
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     *
     * @return <code>true</code> if the arm is at the target angle; <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        double degrees = shoulderMotor.getCurrentDegrees() + START_ANGLE;
        double error = Math.abs(targetAngle - degrees);
        return error <= TOLERABLE_ERROR;
    }

    /**
     * Gets an action to set the target angle for the arm to the requested number of degrees.
     *
     * @param angle the target angle for the arm.
     * @return an action to set the target angle for the arm to the requested number of degrees
     */
    public ActionEx setAngle(double angle) {
        return new AdjustArm(this, angle);
    }

    /**
     * An Action that sets the arm to the requested angle.
     */
    private static class AdjustArm extends ActionExBase {
        private final Arm arm;
        private final double angle;
        private boolean initialized = false;

        /**
         * Instantiates an Action to set the arm to the requested angle.
         *
         * @param arm   the arm to be adjusted.
         * @param angle the angle, in degrees, to which to  move the arm.
         */
        public AdjustArm(Arm arm, double angle) {
            this.arm = arm;
            this.angle = angle;
        }

        /**
         * Runs the Action to adjust the arm angle. This action returns <code>false</code> once the
         * arm has reached the target angle, and <code>true</code> while it is moving to the target
         * angle.
         *
         * @param telemetryPacket telemetry that may be used to output information on the action
         * @return <code>true</code> while the arm is moving to the target angle, <code>false</code>
         *         once it reaches the target angle.
         */
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialize();
            }
            arm.execute();
            return !arm.isAtTarget();
        }

        /**
         * Initializes the action to move the arm to the requested angle.
         */
        private void initialize() {
            arm.setTargetAngle(angle);
            initialized = true;
        }
    }
}
