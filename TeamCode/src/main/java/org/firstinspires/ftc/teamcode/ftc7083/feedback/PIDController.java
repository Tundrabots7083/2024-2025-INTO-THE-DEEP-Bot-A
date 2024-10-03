package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A PID controller to calculate the output given the target state and current state.
 */
public class PIDController {

    public double Kp;
    public double Ki;
    public double Kd;

    private boolean hasRun = false;

    private final ElapsedTime timer = new ElapsedTime();

    private double previousError = 0;

    private double integralSum = 0;

    private double derivative = 0;

    private double minIntegralBound = -1;
    private double maxIntegralBound = 1;

    /**
     * Creates a new PID controller.
     *
     * @param Kp proportional term, multiplied directly by the state error
     * @param Ki integral term, multiplied directly by the state error integral
     * @param Kd derivative term, multiplied directly by the state error rate of change
     */
    public PIDController(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    /**
     * Calculates the PID output.
     *
     * @param reference the target position
     * @param state     current system state
     * @return PID output
     */
    public double calculate(double reference, double state) {
        if (Double.isNaN(reference)) {
            return 0;
        }

        double dt = getDT();
        double error = calculateError(reference, state);
        double derivative = calculateDerivative(error, dt);
        integrate(error, dt);
        previousError = error;

        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("[PID] error", error);
        telemetry.addData("[PID] integral", integralSum);
        telemetry.addData("[PID] derivative", derivative);
        telemetry.addData("[PID] Kp", Kp);
        telemetry.addData("[PID] Ki", Ki);
        telemetry.addData("[PID] Kd", Kd);

        return error * Kp
                + integralSum * Ki
                + derivative * Kd;
    }

    public void setIntegrationBounds(double min, double max) {
        minIntegralBound = min;
        maxIntegralBound = max;
    }

    /**
     * Gets the time constant
     *
     * @return time constant
     */
    public double getDT() {
        if (!hasRun) {
            hasRun = true;
            timer.reset();
        }
        double dt = timer.time();
        timer.reset();
        return dt;
    }

    /**
     * Resets the PID controller so that it behaves as though it hasn't previously run.
     */
    public void reset() {
        hasRun = false;
        timer.reset();

    }

    /**
     * Calculates the error between the target and the current states.
     *
     * @param reference the target state.
     * @param state     the current state.
     * @return the difference between the target state and the current state.
     */
    private double calculateError(double reference, double state) {
        return reference - state;
    }

    /**
     * Adds the current error to the integral sum.
     *
     * @param error the current error
     * @param dt    the time since the PID controller last updated.
     */
    private void integrate(double error, double dt) {
        integralSum += ((error + previousError) / 2) * dt;
        integralSum = Range.clip(integralSum, minIntegralBound, maxIntegralBound);
    }

    /**
     * Calculates the derivative.
     *
     * @param error the current error
     * @param dt    the time since the PID controller last updated.
     * @return the derrivative.
     */
    private double calculateDerivative(double error, double dt) {
        derivative = (error - previousError) / dt;

        if (Double.isNaN(derivative)) {
            derivative = 0;
        }

        return derivative;
    }

    /**
     * Gets a string representation of this PID controller.
     *
     * @return a string representation of this PID controller
     */
    @NonNull
    @Override
    public String toString() {
        return "PIDController{" +
                "Kp=" + Kp +
                ", Kd=" + Kd +
                ", Ki=" + Ki +
                '}';
    }
}
