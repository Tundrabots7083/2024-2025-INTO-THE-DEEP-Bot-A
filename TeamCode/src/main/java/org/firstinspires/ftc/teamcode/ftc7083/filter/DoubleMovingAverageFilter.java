package org.firstinspires.ftc.teamcode.ftc7083.filter;

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
    public Double addValue(Double sum, Double newValue) {
        return sum + newValue;
    }

    @Override
    public Double remValue(Double sum, Double oldValue) {
        return sum - oldValue;
    }

    @Override
    public Double getAverage(Double sum, int numValues) {
        return sum / (double) numValues;
    }
}
