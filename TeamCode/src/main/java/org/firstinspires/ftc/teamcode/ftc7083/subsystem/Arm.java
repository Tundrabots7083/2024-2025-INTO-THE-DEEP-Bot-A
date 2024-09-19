package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.hardware.Motor;

/**
 * Defines an arm with telemetry, slide motor, and sholder motor.
 */

public class Arm extends SubsystemBase {
    private final Motor slideMotor;
    private final Motor sholderMotor;
    private final Telemetry telemetry;

    /**
     * Makes an arm that can raise, lower, retract, and extend.
     *
     * @param hardwareMap Hardware Map
     * @param telemetry   Telemetry
     */
    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        slideMotor = new Motor(hardwareMap, telemetry, "armSlideMotor");
        sholderMotor = new Motor(hardwareMap, telemetry, "armSholderMotor");
    }

    /**
     * Sets the sholder motor to a position in degrees.
     *
     * @param angle Angle of desired arm position in degrees
     */
    public void setSholderAngle(double angle) {
        sholderMotor.setDegrees(angle);

    }

    /**
     * Sets the slide length to a length in inches.
     *
     * @param length Length of desired slide position in inches.
     */
    public void setSlideLength(double length) {
        slideMotor.setInches(length);

    }

}
