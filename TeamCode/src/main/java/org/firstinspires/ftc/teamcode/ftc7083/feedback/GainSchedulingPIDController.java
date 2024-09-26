package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import com.arcrobotics.ftclib.util.InterpLUT;

/**
 * This class uses the FTCLib interpolated Look Up Table to
 * add gain scheduling functionality to PIDController primarily
 * for arms.
 */
public class GainSchedulingPIDController {

    // look up tables for PID coefficients
    private final InterpLUT pCoefficients = new InterpLUT();
    private final InterpLUT iCoefficients = new InterpLUT();
    private final InterpLUT dCoefficients = new InterpLUT();

    /**
     * Sets values to LUTs for Kp, Ki, and Kd.
     *
     * @param KpLUTArgs array of objects containing angles and the Kp constants for each angle.
     * @param KiLUTArgs array of objects containing angles and the Ki constants for each angle.
     * @param KdLUTArgs array of objects containing angles and the Kd constants for each angle.
     */
    public GainSchedulingPIDController(LookUpTableArgs[] KpLUTArgs,
                          LookUpTableArgs[] KiLUTArgs,
                          LookUpTableArgs[] KdLUTArgs) {

        for (LookUpTableArgs kp : KpLUTArgs) {
            pCoefficients.add(kp.state, kp.constantValue);
        }

        for (LookUpTableArgs ki : KiLUTArgs) {
            pCoefficients.add(ki.state, ki.constantValue);
        }

        for (LookUpTableArgs kd : KdLUTArgs) {
            pCoefficients.add(kd.state, kd.constantValue);
        }
    }

    public double calculate(double target, double state) {

        pCoefficients.createLUT();
        iCoefficients.createLUT();
        dCoefficients.createLUT();

        double Kp = pCoefficients.get(state);
        double Ki = iCoefficients.get(state);
        double Kd = dCoefficients.get(state);

        PIDController controller = new PIDController(Kp, Ki, Kd);

        return controller.calculate(target, state);

    }
}
