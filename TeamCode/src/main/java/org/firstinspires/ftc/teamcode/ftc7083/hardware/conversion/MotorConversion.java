package org.firstinspires.ftc.teamcode.ftc7083.hardware.conversion;

import java.math.BigDecimal;
import java.math.MathContext;

public class MotorConversion {
    private static final BigDecimal DEGREES_360 = new BigDecimal("360.0");

    private final BigDecimal ticksPerDegree;

    public MotorConversion(double ticksPerRev, double gearRatio) {
        double ticksPer360 = ticksPerRev * gearRatio;
        ticksPerDegree = new BigDecimal(ticksPer360).divide(DEGREES_360, MathContext.DECIMAL128);
    }

    public double positionToDegrees(double position) {
        BigDecimal pos = new BigDecimal(position);
        BigDecimal degrees = pos.divide(ticksPerDegree, MathContext.DECIMAL128);
        return degrees.doubleValue();
    }

    public double degreesToPosition(double degrees) {
        BigDecimal deg = new BigDecimal(degrees);
        BigDecimal position = deg.multiply(ticksPerDegree);
        return position.doubleValue();
    }
}
