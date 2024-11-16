package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import com.acmerobotics.dashboard.config.Config;

/**
 * A PID controller that includes feed forward and a lower limit.
 * <p>
 * The feed forward component is intended to address the effects of gravity on the system, and
 * can vary depending on the type of mechanism being used. For example, a slide will typically have
 * a constant feed forward component, whereas an arm may change as the arm rotates through its
 * motion (e.g., the gravity affecting the arm is highest when horizontal, and zero when vertical.
 * <p>
 * The lower limit component addresses friction on the system. This will vary based on the direction
 * of movement, where the friction varies based on the direction the mechanism is moving.
 */
@Config
public class PDFLController extends PIDControllerEx {
    public static double DEFAULT_DEADZONE = 5.0;
    private double kL;
    private double deadzone = DEFAULT_DEADZONE;

    /**
     * Instantiates a PDFL controller that does not use the integral and differential components.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kF feed forward term, added to the output of the PID calculation
     * @param kL lower limit term, with the opposite sign added based on the direction of movement
     */
    public PDFLController(double kP, double kF, double kL) {
        this(kP, 0, 0, kF, kL);
    }

    /**
     * Instantiates a PDFL controller that does not use the integral and differential components.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kI integral term, multiplied directly by the state error integral
     * @param kD derivative term, multiplied directly by the state error rate of change
     * @param kF feed forward term, added to the output of the PID calculation
     * @param kL lower limit term, with the opposite sign added based on the direction of movement
     */
    public PDFLController(double kP, double kI, double kD, double kF, double kL) {
        super(kP, kI, kD, kF);
        this.kL = kL;
    }

    /**
     * Sets a deadzone for the PDFL controller.
     *
     * @param deadzone the deadzone for the PDFL controller.
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    @Override
    public double calculate(double reference, double state) {
        double error = calculateError(reference, state);

        double pidf = super.calculate(reference, state);
        double l = lComponent(error);

        return pidf + l;
    }

    /**
     * Sets the PID coefficients to the new values.
     *
     * @param kP proportional term, multiplied directly by the state error
     * @param kI integral term, multiplied directly by the state error integral
     * @param kD derivative term, multiplied directly by the state error rate of change
     * @param kF feed forward term, added to the output of the PID calculation
     * @param kL lower limit term, with the opposite sign added based on the direction of movement
     */
    public void setCoefficients(double kP, double kI, double kD, double kF, double kL) {
        setCoefficients(kP, kI, kD, kF);
        this.kL = kL;
    }

    /**
     * Gets the lower limit component based on the current error.
     *
     * @param error the current error
     * @return the lower limit component based on the current error
     */
    private double lComponent(double error) {
        if (Math.abs(error) < deadzone) {
            return 0.0;
        }
        double direction = Math.signum(error);
        return direction * kL;
    }
}
