package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

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
    public static double START_ANGLE = -50;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 1120.0; // AndyMark NeverRest ticks per rev
    public double GEARING = 120.0 / 24.0;
    public static double KD1 = 0.006;
    public static double KD2 = 0.0075;
    public static double KD3 = 0.003;
    public static double KD4 = 0.008;
    public static double kI;
    public static double kD;
    public static double TOLERABLE_ERROR = 0.25; // In degrees

    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    private final double feedforward;
    //private final PIDController pidController;
    private final GainSchedulingPIDController gainSchedulingPIDController;
    private double angle = START_ANGLE;

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
                new LookUpTableArgs(-52, 0.01),
                new LookUpTableArgs(0, 0.04),
                new LookUpTableArgs(90, 0.019),
                new LookUpTableArgs(120, 0.025),
                new LookUpTableArgs(180, 0.032),
                new LookUpTableArgs(230, 0.01)};
        LookUpTableArgs[] kiLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-52, 0.1),
                new LookUpTableArgs(0, 0.1),
                new LookUpTableArgs(90, 0.07),
                new LookUpTableArgs(150, 0.09),
                new LookUpTableArgs(230, 0.1)};
        LookUpTableArgs[] kdLUTArgs = new LookUpTableArgs[]{
                new LookUpTableArgs(-52, 0.006),
                new LookUpTableArgs(0, 0.0075),
                new LookUpTableArgs(90, 0.003),
                new LookUpTableArgs(150, 0.008),
                new LookUpTableArgs(230, 0.006)};

        gainSchedulingPIDController = new GainSchedulingPIDController(kpLUTArgs, kiLUTArgs, kdLUTArgs);
        //pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Gets the shoulder motor position to which we are moving
     */
    public double getShoulderAngle() {
        return shoulderMotor.getDegrees();
    }

    /**
     * Sets the shoulder motor to a position in degrees.
     */
    public void setShoulderAngle(double angle) {
        if (this.angle != angle) {
            this.angle = angle;
            gainSchedulingPIDController.reset();
        }
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
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER); // TODO: was RUN_WITH_ENCODER, but that seems wrong
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sends power to the shoulder motor.
     */
    public void execute() {
        double degrees = shoulderMotor.getDegrees() + START_ANGLE;
       // double power = pidController.calculate(angle, degrees) + this.feedforward;
        double power = gainSchedulingPIDController.calculate(angle, degrees) + this.feedforward;
        shoulderMotor.setPower(power);
        telemetry.addData("[Arm] Target", angle);
        telemetry.addData("[Arm] Current", degrees);
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     *
     * @return <code>true</code> if the arm is at the target angle; <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        double degrees = shoulderMotor.getDegrees() + START_ANGLE;
        double error = Math.abs(angle - degrees);
        return error <= TOLERABLE_ERROR;
    }
}