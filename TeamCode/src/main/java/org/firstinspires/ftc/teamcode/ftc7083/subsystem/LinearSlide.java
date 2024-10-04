package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
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
    public static double SPOOL_DIAMETER = 1.4; // in inches
    public static double TICKS_PER_REV = 538;
    public double GEARING = 1.0;
    public static double ACHIEVABLE_MAX_RPM_FRACTION = 1.0;

    public static double KP = 0.1;
    public static double KI = 0.05;
    public static double KD = 0.0;
    public static double TOLERABLE_ERROR = 0.1; // inches

    private final Motor slideMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    private double targetLength = 0;

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
        if (targetLength != length) {
            targetLength = length;
            pidController.reset();
        }
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
        motorConfigurationType.setTicksPerRev(TICKS_PER_REV);
        motorConfigurationType.setGearing(GEARING);
        motorConfigurationType.setAchieveableMaxRPMFraction(ACHIEVABLE_MAX_RPM_FRACTION);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setInchesPerRev(Math.PI * SPOOL_DIAMETER);
    }

    /**
     * sets the power for the pid controller
     */
    public void execute() {
        if (!isAtTarget()) {
            double power = pidController.calculate(targetLength, slideMotor.getInches());
            slideMotor.setPower(power);
            telemetry.addData("[Slide] power", power);
            telemetry.addData("[Slide] inches", slideMotor.getInches());
            telemetry.addData("[Slide] ticks", slideMotor.getCurrentPosition());
        }
    }

    /**
     * checks if the length is within the tolerable error and if it is then the motor will stop
     */
    public boolean isAtTarget() {
        double error = Math.abs(targetLength - slideMotor.getInches());
        telemetry.addData("[Slide] error", error);
        telemetry.addData("[Slide] target", targetLength);
        telemetry.addData("[Slide] current", slideMotor.getInches());
        return error <= TOLERABLE_ERROR;
    }
}
