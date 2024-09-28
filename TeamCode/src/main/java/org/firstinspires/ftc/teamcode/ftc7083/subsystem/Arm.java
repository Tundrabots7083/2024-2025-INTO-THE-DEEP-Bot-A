package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.LookUpTableArgs;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, shoulder motor.
 */
@Config
public class Arm extends SubsystemBase {
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;
    public static double TICKS_PER_REV = 1120.0; // AndyMark NeverRest ticks per rev
    public static double GEARING = 0.03175;
    public static double kP = 0.1;
    public static double kI;
    public static double kD;
    public static double TOLERABLE_ERROR = 0.25; // In degrees

    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    public PIDController pidController;
    //public GainSchedulingPIDController gainSchedulingPIDController;
    public LookUpTableArgs[] KpLUTArgs;
    public LookUpTableArgs[] KiLUTArgs;
    public LookUpTableArgs[] KdLUTArgs;
    private double angle = 0.0;
    private final double feedforward;

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

        KpLUTArgs = new LookUpTableArgs[]{new LookUpTableArgs(40, 0.08), new LookUpTableArgs(90, 0.001), new LookUpTableArgs(-75, 0.05)};
        KiLUTArgs = new LookUpTableArgs[]{new LookUpTableArgs(40, 0.08), new LookUpTableArgs(90, 0.001), new LookUpTableArgs(-75, 0.05)};
        KdLUTArgs = new LookUpTableArgs[]{new LookUpTableArgs(40, 0.08), new LookUpTableArgs(90, 0.001), new LookUpTableArgs(-75, 0.05)};

        // gainSchedulingPIDController = new GainSchedulingPIDController(KpLUTArgs, KiLUTArgs, KdLUTArgs);
        pidController = new PIDController(kP, kI, kD);
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
            pidController.reset();
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
        motor.setMode(Motor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sends power to the shoulder motor.
     */
    public void execute() {
        //double power = gainSchedulingPIDController.calculate(angle, shoulderMotor.getDegrees()) + this.feedforward;
        double power = pidController.calculate(angle, shoulderMotor.getDegrees()) + this.feedforward;
        shoulderMotor.setPower(power);
        telemetry.addData("[Arm] Target", angle);
        telemetry.addData("[Arm] Current", shoulderMotor.getDegrees());
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     *
     * @return <code>true</code> if the arm is at the target angle; <code>false</code> otherwise.
     */
    public boolean isAtTarget() {
        double error = Math.abs(angle - shoulderMotor.getDegrees());
        return error <= TOLERABLE_ERROR;
    }
}