package org.firstinspires.ftc.teamcode.autonomous.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Params {
    // drive model parameters
    public static double inPerTick = 1;
    public static double lateralInPerTick = inPerTick;
    public static double trackWidthTicks = 0;

    // feedforward parameters (in tick units)
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    // path profile parameters (in inches)
    public static double maxWheelVel = 50;
    public static double minProfileAccel = -30;
    public static double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public static double maxAngVel = Math.PI; // shared with path
    public static double maxAngAccel = Math.PI;

    // path controller gains
    public static double axialGain = 0.0;
    public static double lateralGain = 0.0;
    public static double headingGain = 0.0; // shared with turn

    public static double axialVelGain = 0.0;
    public static double lateralVelGain = 0.0;
    public static double headingVelGain = 0.0; // shared with turn
}
