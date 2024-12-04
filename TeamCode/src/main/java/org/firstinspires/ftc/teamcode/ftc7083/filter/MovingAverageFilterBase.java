package org.firstinspires.ftc.teamcode.ftc7083.filter;

import java.util.LinkedList;
import java.util.Queue;

/**
 * A Moving Average Filter is a digital signal processing technique used to smooth out noisy data
 * from a sensor. The basic idea behind the Moving Average Filter is to calculate the average of a
 * fixed window of recent sensor values, effectively reducing random fluctuations and providing a
 * more consistent output.
 */
public abstract class MovingAverageFilterBase<T> implements MovingAverageFilter<T> {
    private final Queue<T> buffer;
    private final int minNumSamples;
    private final int windowSize;
    private T sum;

    /**
     * Creates a new Moving Average Filter with the given window size.
     *
     * @param minNumSamples the minimum number of samples required to calculate an average
     * @param windowSize    the window size for the Moving Average Filter
     */
    public MovingAverageFilterBase(int minNumSamples, int windowSize) {
        this.buffer = new LinkedList<>();
        this.minNumSamples = minNumSamples;
        this.windowSize = windowSize;
        this.sum = startingValue();
    }

    /**
     * Returns the starting value for the moving average.
     */
    abstract public T startingValue();

    @Override
    public T filter(T newMeasurement) {
        buffer.add(newMeasurement);

        sum = addValue(sum, newMeasurement);

        if (buffer.size() > windowSize) {
            removeMeasurement();
        }

        return getMean();
    }

    /**
     * Adds a value to the moving average and returns the updated moving average.
     *
     * @param newValue the value to add to the moving average.
     * @return the updated moving average
     */
    abstract protected T addValue(T sum, T newValue);

    /**
     * Removes a value to the moving average and returns the updated moving average.
     *
     * @param oldValue the value to remove to the moving average.
     * @return the updated moving average
     */
    abstract protected T remValue(T sum, T oldValue);

    /**
     * Gets the average value of the Moving Average Filter.
     *
     * @return the average value of the Moving Average Filter
     */
    abstract protected T getMean(T sum, int numValues);

    /**
     * Gets the variance for the moving average filter.
     *
     * @return the variance for the moving average filter
     */
    abstract protected T getVariance(Queue<T> buffer);

    /**
     * Gets the standard deviation for the moving average filter.
     *
     * @return the standard deviation for the moving average filter
     */
    abstract protected T getStdDev(Queue<T> buffer);

    @Override
    public T removeMeasurement() {
        if (!buffer.isEmpty()) {
            T val = buffer.poll();
            if (val != null) {
                sum = remValue(sum, val);
            }
        }

        return getMean(sum, buffer.size());
    }

    @Override
    public T getMean() {
        return getMean(sum, buffer.size());
    }

    @Override
    public T getStdDev() {
        return getStdDev(buffer);
    }

    @Override
    public int numMeasurements() {
        return buffer.size();
    }

    @Override
    public boolean hasMean() {
        return buffer.size() >= minNumSamples;
    }

    @Override
    public void clear() {
        buffer.clear();
        sum = startingValue();
    }
}
