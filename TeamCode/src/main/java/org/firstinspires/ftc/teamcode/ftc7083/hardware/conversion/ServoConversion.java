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
        BigDecimal positionPercent = MAX_POSITION.divide(pos, MathContext.UNLIMITED);
        BigDecimal degrees = maxDegrees.multiply(positionPercent);
        return degrees.doubleValue();
    }

    public double degreesToPosition(double degrees) {
        BigDecimal deg = new BigDecimal(degrees);
        BigDecimal degreePercent = maxDegrees.divide(deg, MathContext.UNLIMITED);
        return degreePercent.doubleValue();
    }
}
