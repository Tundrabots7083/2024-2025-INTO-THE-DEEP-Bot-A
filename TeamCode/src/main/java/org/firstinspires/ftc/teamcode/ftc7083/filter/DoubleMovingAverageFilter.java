package org.firstinspires.ftc.teamcode.ftc7083.filter;

import java.util.Queue;

public class DoubleMovingAverageFilter extends MovingAverageFilterBase<Double> {

    /**
     * Creates a new Moving Average Filter for doubles with the given window size.
     *
     * @param minNumSamples the minimum number of samples required to calculate an average
     * @param windowSize    the window size for the Moving Average Filter
     */
    public DoubleMovingAverageFilter(int minNumSamples, int windowSize) {
        super(minNumSamples, windowSize);
    }

    @Override
    public Double startingValue() {
        return 0.0;
    }

    @Override
    protected Double addValue(Double sum, Double newValue) {
        return sum + newValue;
    }

    @Override
    protected Double remValue(Double sum, Double oldValue) {
        return sum - oldValue;
    }

    @Override
    protected Double getMean(Double sum, int numValues) {
        return sum / (double) numValues;
    }

    @Override
    protected Double getVariance(Queue<Double> buffer) {
        if (buffer.isEmpty()) {
            return 0.0;
        }

        double variance = 0.0;
        double mean = getMean();

        for (Double value : buffer) {
            variance += Math.pow(value - mean, 2);
        }
        return variance / (double) buffer.size();
    }

    @Override
    public Double getStdDev(Queue<Double> buffer) {
        double variance = getVariance(buffer);
        return Math.sqrt(variance);
    }
}
