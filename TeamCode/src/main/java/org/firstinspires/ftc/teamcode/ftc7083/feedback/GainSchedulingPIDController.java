package org.firstinspires.ftc.teamcode.ftc7083.feedback;

import com.arcrobotics.ftclib.util.InterpLUT;


public class GainSchedulingPIDController {

    // look up tables for PID coefficients
    private final InterpLUT pCoefficients = new InterpLUT();
    private final InterpLUT iCoefficients = new InterpLUT();
    private final InterpLUT dCoefficients = new InterpLUT();

    public GainSchedulingPIDController(GainSchedulingLUTArgs[] KpLUTArgs,
                          GainSchedulingLUTArgs[] KiLUTArgs,
                          GainSchedulingLUTArgs[] KdLUTArgs) {

        for (GainSchedulingLUTArgs kp : KpLUTArgs) {
            pCoefficients.add(kp.angle, kp.constantValue);
        }

        for (GainSchedulingLUTArgs ki : KiLUTArgs) {
            pCoefficients.add(ki.angle, ki.constantValue);
        }

        for (GainSchedulingLUTArgs kd : KdLUTArgs) {
            pCoefficients.add(kd.angle, kd.constantValue);
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
