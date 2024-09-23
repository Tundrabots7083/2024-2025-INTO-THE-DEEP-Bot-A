package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, shoulder motor.
 */

public class Arm extends SubsystemBase {
    private final Motor shoulderMotor;
    private final Telemetry telemetry;

    /**
     * Makes an arm that can raise and lower.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        shoulderMotor = new Motor(hardwareMap, telemetry, "armShoulderMotor");
    }

    /**
     * Sets the shoulder motor to a position in degrees.
     *
     */
    public void setShoulderAngle(double angle) {
        shoulderMotor.setDegrees(angle);
    }

    /**
     * Gets the shoulder motor position to
     * @return
     */
    public double getShoulderAngle() {
        return shoulderMotor.getDegrees();
    }

    private void configMotor(Motor motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setTicksPerRev(1.0);
        motorConfigurationType.setGearing(16.0);
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}