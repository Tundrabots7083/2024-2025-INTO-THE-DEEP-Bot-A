package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, shoulder motor.
 */

public class Arm extends SubsystemBase {
    public static double kP;
    public static double kI;
    public static double kD;
    public static double TOLERABLE_ERROR = 1.0;

    private final Motor shoulderMotor;
    private final Telemetry telemetry;
    public PIDController pidController;
    private double angle = 0.0;
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
        pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Sets the shoulder motor to a position in degrees.
     *
     */
    public void setShoulderAngle(double angle) {
        this.angle = angle;
        pidController.reset();
    }

    /**
     * Gets the shoulder motor position to
     */
    public double getShoulderAngle() {
        return shoulderMotor.getDegrees();
    }

    /**
     * Configures the shoulder motor.
     * @param motor
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(1.0);
        motorConfigurationType.setGearing(16.0);
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sends power to the PID controller.
     */
    public void execute() {
        double power = pidController.calculate(angle, shoulderMotor.getDegrees());
        shoulderMotor.setPower(power);
    }

    /**
     * Can tell if the current position is within a reasonable distance of the target.
     * @return
     */
    public boolean isAtTarget(){
        double error = Math.abs(angle - shoulderMotor.getDegrees());
        return error <= TOLERABLE_ERROR;
    }
}