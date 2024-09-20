package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, slide motor, and sholder motor.
 */

public class LinearSlide extends SubsystemBase {
    private final Motor slideMotor;
    private final Telemetry telemetry;

    /**
     * Makes an arm that can raise, lower, retract, and extend.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = new Motor(hardwareMap, telemetry, "armSlideMotor");
        configMotor(slideMotor);
    }


    /**
     * Sets the slide length to a length in inches.
     *
     * @param length Length of desired slide position in inches.
     */
    public void setLength(double length) {
        slideMotor.setInches(length);

    }

    /**
     * Gets the slide length in inches
     * Finds the value for the length
     *
     * @return slide length in inches
     */
    public double getLength() {
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
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
