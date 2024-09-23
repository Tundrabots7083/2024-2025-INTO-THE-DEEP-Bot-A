package org.firstinspires.ftc.teamcode.ftc7083.feedback;

/**
 * GainSchedulingLUTArgs creates an object containing angles and PID constant (Kx) values
 * to be plugged into a LUT (look up table).
 */
public class LookUpTableArgs {

    public final double state;
    public final double constantValue;

    /**
     * Creates an instance of this class with state and Kx
     *
     * @param state is the state at which the constant (Kx) value was calculated
     * @param constantValue is the PID constant value in question (could be Kp, Ki, Kd)
     */
    public LookUpTableArgs(double state, double constantValue) {
        this.state = state;
        this.constantValue = constantValue;
    }
}
