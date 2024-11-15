package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.GainSchedulingPIDController;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.LookUpTableArgs;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * An Arm is used to move the scoring subsystem in a circular arc, allowing the robot to both
 * pickup and score sample and specimens.
 */
@Config
public class Arm extends SubsystemBase {
    public static double START_ANGLE = -47.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 537.6; // gobuilda ticks per rev
    public static double TOLERABLE_ERROR = 0.7; // In degrees
    public static double MIN_ANGLE = -47.0;
    public static double MAX_ANGLE = 90.0;
    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    private double feedforward;
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
        this.telemetry = telemetry;
        shoulderMotor = new Motor(hardwareMap, telemetry, "armShoulderMotor");
        configMotor(shoulderMotor);

        LookUpTableArgs[] kpLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59, 0.03),
                new LookUpTableArgs(-20, 0.04),
                new LookUpTableArgs(0, 0.05),
                new LookUpTableArgs(20, 0.04),
                new LookUpTableArgs(90, 0.026),
                new LookUpTableArgs(120, 0.04),
                new LookUpTableArgs(200,0.03)};
        LookUpTableArgs[] kiLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59, 0.12),
                new LookUpTableArgs(-20, 0.12),
                new LookUpTableArgs(0, 0.135),
                new LookUpTableArgs(20, 0.08),
                new LookUpTableArgs(90, 0.1),
                new LookUpTableArgs(120, 0.08),
                new LookUpTableArgs(200,0.08)};
        LookUpTableArgs[] kdLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-59, 0.001),
                new LookUpTableArgs(-20, 0.001),
                new LookUpTableArgs(0, 0.001),
                new LookUpTableArgs(20, 0.0015),
                new LookUpTableArgs(90, 0.0017),
                new LookUpTableArgs(120, 0.001),
                new LookUpTableArgs(200,0.001)};
        
        gainSchedulingPIDController = new GainSchedulingPIDController(kpLUTArgs, kiLUTArgs, kdLUTArgs);
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

    public void setFeedforward(double feedforward) {
        this.feedforward = feedforward;
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
     * Sends power to the shoulder motor.
     */
    public void execute() {
        double degrees = getCurrentAngle();
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
}
