package org.firstinspires.ftc.teamcode.ftc7083.feedback;

/**
 * A PID controller calculates the output given the target state and current state.
 */
public interface PIDController {
    /**
     * Calculates the PID output.
     *
     * @param reference the target position
     * @param state     current system state
     * @return PID output
     */
    double calculate(double reference, double state);

    /**
     * Resets the PID controller so that it behaves as though it hasn't previously run.
     */
    void reset();

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     */
    void setCoefficients(double Kp, double Ki, double Kd);
}
