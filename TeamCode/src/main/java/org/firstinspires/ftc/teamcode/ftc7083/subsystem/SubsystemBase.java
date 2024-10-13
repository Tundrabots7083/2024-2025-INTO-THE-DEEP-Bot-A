package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

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

    /**
     * Returns the maximum absolute value in the list of numbers. If no numbers are provided, the
     * value of <code>0.0</code> is returned.
     *
     * @param values the list of values from which to find the maximum absolute value.
     * @return the maximum absolute value from the list of values.
     */
    protected final double maxAbs(double... values) {
        double max = 0.0;
        for (double v : values) {
            max = Math.max(max, Math.abs(v));
        }
        return max;
    }

    /**
     * Calculates the angle in degrees from the x-axis for a point,
     * (x, y) anywhere on a two dimensional plane.
     *
     * @param x The length in inches of the arm and the wrist
     *          with zero extension.
     * @param y The height in inches of the arm's center of rotation
     *          from the floor.
     * @return double The angle in degrees from the x-axis associated
     * with the input (x, y) point on a two dimensional plane.
     */
    protected final double getAngle(double x, double y) {
        double targetAngleRadians = Math.atan2(y, x);
        return Math.toDegrees(targetAngleRadians);
    }

    /**
     * Calculates the height, in inches, of a right triangle given the
     * angle the hypotenuse makes relative to the base and the length
     * of the hypotenuse in inches.
     *
     * @param angle      The angle of the hypotenuse of a right triangle relative
     *                   to its base, in degrees.
     * @param hypotenuse The length of the hypotenuse of a right triangle in inches.
     * @return double  The height, in inches, of the right triangle.
     */
    protected final double getY(double angle, double hypotenuse) {
        double y = hypotenuse * Math.sin(Math.toRadians(angle));
        return y;
    }

    /**
     * Calculates the length of the base of a right triangle, in inches, given the
     * angle the hypotenuse makes relative to the base and the length
     * of the hypotenuse in inches.
     *
     * @param angle      The angle of the hypotenuse of a right triangle relative
     *                   to its base, in degrees.
     * @param hypotenuse The length of the hypotenuse of a right triangle in inches.
     * @return double     The length of the base, in inches, of the right triangle.
     */
    protected final double getX(double angle, double hypotenuse) {
        double x = hypotenuse * Math.cos(Math.toRadians(angle));
        return x;
    }

}
