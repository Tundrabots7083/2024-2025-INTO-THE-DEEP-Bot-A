package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.feedback.PIDController;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, slide motor, and PID.
 */
@Config
public class LinearSlide extends SubsystemBase {
    public static double KP;
    public static double KI;
    public static double KD;
    public static double TOLERABLE_ERROR = 1.0;

    private final Motor slideMotor;
    private final Telemetry telemetry;
    private PIDController pidController;
    private double length = 0;

    /**
     * Makes an arm that can raise, lower, retract, and extend.
     * sets the pid controller
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = new Motor(hardwareMap, telemetry, "armSlideMotor");
        configMotor(slideMotor);
        pidController = new PIDController(KP, KI, KD);
    }


    /**
     * sets the slide length from an external source and uses pid to make is
     *
     * @param length Length of desired slide position in inches.
     */
    public void setLength(double length) {
        this.length = length;
        pidController.reset();

    }

    /**
     * Gets the target slide length in inches
     * Finds the value for the length
     *
     * @return target slide length in inches
     */
    public double getLength() {
        return length;
    }

    /**
     * Gets the slide length in inches
     * Finds the value for the length
     *
     * @return slide length in inches
     */
    public double getCurrentLength() {
        return slideMotor.getInches();
    }

    /**
     * Configures the motor used for the linear slide
     *
     * @param motor
     */
    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(1.0);
        motorConfigurationType.setGearing(16.0);
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * sets the power for the pid controller
     */
    public void execute() {
        double power = pidController.calculate(length, slideMotor.getInches());
        slideMotor.setPower(power);
    }

    /**
     * checks if the length is within the tolerable error and if it is then the motor will stop
     */
    public boolean isAtTarget() {
        double error = Math.abs(length - slideMotor.getInches());
        return error <= TOLERABLE_ERROR;
    }
}
