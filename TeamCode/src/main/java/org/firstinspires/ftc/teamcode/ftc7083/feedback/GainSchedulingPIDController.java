package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import com.arcrobotics.ftclib.util.InterpLUT;

/**
 * This class uses the FTCLib interpolated Look Up Table to
 * add gain scheduling functionality to PIDController primarily
 * for arms.
 */
public class GainSchedulingPIDController implements PIDController {

    // look up tables for PID coefficients
    private final InterpLUT pCoefficients = new InterpLUT();
    private final InterpLUT iCoefficients = new InterpLUT();
    private final InterpLUT dCoefficients = new InterpLUT();
    public PIDController controller;

    /**
     * Sets values to LUTs for Kp, Ki, and Kd. Creates a PID controller, and sets preliminary values for the PID.
     *
     * @param KpLUTArgs array of objects containing angles and the Kp constants for each angle.
     * @param KiLUTArgs array of objects containing angles and the Ki constants for each angle.
     * @param KdLUTArgs array of objects containing angles and the Kd constants for each angle.
     */
    public GainSchedulingPIDController(LookUpTableArgs[] KpLUTArgs,
                                       LookUpTableArgs[] KiLUTArgs,
                                       LookUpTableArgs[] KdLUTArgs) {


        for (LookUpTableArgs kpLUTArg : KpLUTArgs) {
            pCoefficients.add(kpLUTArg.state, kpLUTArg.constantValue);
        }

        for (LookUpTableArgs kiLUTArg : KiLUTArgs) {
            iCoefficients.add(kiLUTArg.state, kiLUTArg.constantValue);
        }

        for (LookUpTableArgs kdLUTArg : KdLUTArgs) {
            dCoefficients.add(kdLUTArg.state, kdLUTArg.constantValue);
        }

        pCoefficients.createLUT();
        iCoefficients.createLUT();
        dCoefficients.createLUT();

        controller = new PIDControllerImpl(0, 0, 0);
    }

    @Override
    public double calculate(double target, double state) {
        double Kp = pCoefficients.get(state);
        double Ki = iCoefficients.get(state);
        double Kd = dCoefficients.get(state);
        controller.setCoefficients(Kp, Ki, Kd);

        return controller.calculate(target, state);
    }

    @Override
    public void reset() {
        controller.reset();
    }

    @Override
    public void setCoefficients(double Kp, double Ki, double Kd) {
        controller.setCoefficients(Kp, Ki, Kd);
    }
}
