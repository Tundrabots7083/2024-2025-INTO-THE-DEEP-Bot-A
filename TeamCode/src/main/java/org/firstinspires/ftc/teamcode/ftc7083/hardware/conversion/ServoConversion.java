package org.firstinspires.ftc.teamcode.ftc7083.hardware.conversion;

import java.math.BigDecimal;
import java.math.MathContext;

public class ServoConversion {
    private static final BigDecimal MAX_POSITION= new BigDecimal("1.0");

    private final BigDecimal maxDegrees;

    public ServoConversion(double maxDegrees) {
        this.maxDegrees= new BigDecimal(maxDegrees);
    }

    public double positionToDegrees(double position) {
        BigDecimal pos = new BigDecimal(position);
        BigDecimal degrees = pos.multiply(maxDegrees);
        return degrees.doubleValue();
    }

    public double degreesToPosition(double degrees) {
        BigDecimal deg = new BigDecimal(degrees);
        BigDecimal degreePercent = deg.divide(maxDegrees, MathContext.DECIMAL128);
        return degreePercent.doubleValue();
    }
}
