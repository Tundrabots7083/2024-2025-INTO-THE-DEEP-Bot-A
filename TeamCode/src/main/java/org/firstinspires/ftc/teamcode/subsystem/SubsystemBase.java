package org.firstinspires.ftc.teamcode.subsystem;

/**
 * Base subsystem used by our subsystems. This includes common methods that are useful to
 * multiple subsystems.
 */
public abstract class SubsystemBase implements Subsystem {

    /**
     * Modifies the power value so that it is between the minimum and maximum values.
     *
     * @param power    the power to apply to the motor
     * @param minPower the minimum amount of power that must be applied. If the amount of power
     *                 is less than the minimum, the power is set to zero.
     * @return the power, or if less than the minimum amount, 0 (zero).
     */
    protected double modifyMotorPower(double power, double minPower) {
        // Cap the motor power at 1 and -1
        power = Math.max(-1, Math.min(1, power));
        // If the power level of the motor is below the minimum threshold, set it to 0
        if (Math.abs(power) < minPower) {
            power = 0;
        }
        return power;
    }
}