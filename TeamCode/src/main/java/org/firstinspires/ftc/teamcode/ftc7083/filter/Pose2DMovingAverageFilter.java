package org.firstinspires.ftc.teamcode.ftc7083.filter;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

/**
 * A Moving Average Filter for the X, Y and Heading values for a webcam. This stores the values in
 * a SparkFunOTOS.Pose2D object.
 */
public class Pose2DMovingAverageFilter extends MovingAverageFilterBase<Pose2D> {
    /**
     * Creates a new Moving Average Filter for Pose2D's with the given window size.
     *
     * @param minNumSamples the minimum number of samples required to calculate an average
     * @param windowSize    the window size for the Moving Average Filter
     */
    public Pose2DMovingAverageFilter(int minNumSamples, int windowSize) {
        super(minNumSamples, windowSize);
    }

    @Override
    public Pose2D startingValue() {
        return new Pose2D(0, 0, 0);
    }

    @Override
    public Pose2D addValue(Pose2D sum, Pose2D newValue) {
        return new Pose2D(
                sum.x + newValue.x,
                sum.y + newValue.y,
                sum.h + newValue.h
        );
    }

    @Override
    public Pose2D remValue(Pose2D sum, Pose2D oldValue) {
        return new Pose2D(
                sum.x - oldValue.x,
                sum.y - oldValue.y,
                sum.h - oldValue.h
        );
    }

    @Override
    public Pose2D getAverage(Pose2D sum, int numValues) {
        return new Pose2D(
                sum.x / (double) numValues,
                sum.y / (double) numValues,
                sum.h / (double) numValues
        );
    }
}
