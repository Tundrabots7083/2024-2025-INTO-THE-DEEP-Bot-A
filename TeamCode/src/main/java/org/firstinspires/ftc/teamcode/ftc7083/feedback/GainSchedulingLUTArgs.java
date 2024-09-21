package org.firstinspires.ftc.teamcode.ftc7083.feedback;

/**
 * GainSchedulingLUTArgs creates an object containing angles and Kx values
 * that are passed into the LUT (look up table) in the class GainScheduling.
 */
public class GainSchedulingLUTArgs {

    public final double angle;
    public final double constantValue;

    /**
     * Creates an instance of this class with angle and Kx
     *
     * @param angle is the angle at which the Kx value was calculated
     * @param constantValue is the PID constant value in question (could be Kp, Ki, Kd)
     */
    public GainSchedulingLUTArgs(double angle, double constantValue) {
        this.angle = angle;
        this.constantValue = constantValue;
    }
}
