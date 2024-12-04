package org.firstinspires.ftc.teamcode.ftc7083.filter;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;

import java.util.Queue;

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
    protected Pose2D getMean(Pose2D sum, int numValues) {
        return new Pose2D(
                sum.x / (double) numValues,
                sum.y / (double) numValues,
                sum.h / (double) numValues
        );
    }

    @Override
    protected Pose2D getVariance(Queue<Pose2D> buffer) {
        if (buffer.isEmpty()) {
            return new Pose2D();
        }

        Pose2D variance = new Pose2D();
        Pose2D mean = getMean();

        for (Pose2D value : buffer) {
            variance.x += Math.pow(value.x - mean.x, 2);
            variance.y += Math.pow(value.y - mean.y, 2);
            variance.h += Math.pow(value.h - mean.h, 2);
        }
        variance.x /= buffer.size();
        variance.y /= buffer.size();
        variance.h /= buffer.size();

        return variance;
    }

    @Override
    protected Pose2D getStdDev(Queue<Pose2D> buffer) {
        Pose2D variance = getVariance(buffer);
        return new Pose2D(Math.sqrt(variance.x), Math.sqrt(variance.y), Math.sqrt(variance.h));
    }
}
