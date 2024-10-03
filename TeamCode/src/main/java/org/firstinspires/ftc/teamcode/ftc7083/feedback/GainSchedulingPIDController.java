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
    private double Kp = 0.0;
    private double Ki = 0.0;
    private double Kd = 0.0;
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

        controller = new PIDController(Kp, Ki, Kd);
        //controller.setIntegrationBounds(0.0,1.0);

    }

    public double calculate(double target, double state) {

        Kp = pCoefficients.get(state);
        Ki = iCoefficients.get(state);
        Kd = dCoefficients.get(state);

        controller.Kp = Kp;
        controller.Ki = Ki;
        controller.Kd = Kd;

        return controller.calculate(target,state);

    }

    public void reset(){
        controller.reset();
    }
}
