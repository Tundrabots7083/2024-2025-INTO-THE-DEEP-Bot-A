package org.firstinspires.ftc.teamcode.ftc7083.filter;

public interface MovingAverageFilter<T> {
    /**
     * Adds a new value to the Moving Average Filter and returns the average of the values.
     *
     * @param newMeasurement the new value to add to the Moving Average Filter
     * @return the new average of values
     */
    T filter(T newMeasurement);

    /**
     * Removes the oldest value from the Moving Average Filter without adding a new value.
     *
     * @return the updated average value of the Moving Average Filter
     */
    T removeMeasurement();

    /**
     * Gets the average value of the Moving Average Filter.
     *
     * @return the average value of the Moving Average Filter
     */
    T getMean();

    /**
     * Gets the standard deviation for the moving average filter.
     *
     * @return the standard deviation for the moving average filter
     */
    T getStdDev();

    /**
     * Gets the number of values in the Moving Average Filter.
     *
     * @return the number of values in the Moving Average Filter
     */
    int numMeasurements();

    /**
     * Gets an indication as to whether there are enough measurements to calculate an average value.
     *
     * @return <code>true</code> if  there are enough measurements to calculate an average value;
     *         <code>false</code> if there aren't
     */
    boolean hasMean();

    /**
     * Clears the Moving Average Filter
     */
    void clear();
}
