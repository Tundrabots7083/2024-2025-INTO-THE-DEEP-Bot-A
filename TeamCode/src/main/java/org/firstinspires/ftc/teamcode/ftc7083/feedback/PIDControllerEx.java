package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import androidx.annotation.NonNull;

/**
 * A PID controller that uses a feed forward mechanism in its calculations.
 */
public class PIDControllerEx extends PIDControllerImpl {
    private FeedForward ff;

    /**
     * Creates a new PID controller with a constant gravity FeedForward component.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param Kg constant feed forward gravity term, added to the output of the PID calculation
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, double Kg) {
        this(Kp, Ki, Kd, p -> Kg);
    }

    /**
     * Creates a new PID controller with the provided FeedForward function.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param ff the feed forward function
     */
    public PIDControllerEx(double Kp, double Ki, double Kd, FeedForward ff) {
        super(Kp, Ki, Kd);
        this.ff = ff;
    }

    @Override
    public double calculate(double reference, double state) {
        double power = super.calculate(reference, state);
        return power + ff.calculate(state);
    }

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param kF feed forward term, added to the output of the PID calculation
     */
    public void setCoefficients(double Kp, double Ki, double Kd, double kF) {
        setCoefficients(Kp, Ki, Kd, p->kF);
    }

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     * @param ff the feed forward function
     */
    public void setCoefficients(double Kp, double Ki, double Kd, FeedForward ff) {
        setCoefficients(Kp, Ki, Kd);
        this.ff = ff;
    }

    /**
     * Gets a string representation of this PID controller.
     *
     * @return a string representation of this PID controller
     */
    @NonNull
    @Override
    public String toString() {
        return "PIDControllerEx{" +
                super.toString() +
                "ff=" + ff +
                '}';
    }
}
