package org.firstinspires.ftc.teamcode.ftc7083.filter;

public interface MovingAverageFilter<T> {
    /**
     * Adds a new value to the Moving Average Filter and returns the average of the values.
     *
     * @param newMeasurement the new value to add to the Moving Average Filter
     * @return the new average of values
     */
    public T filter(T newMeasurement);

    /**
     * Removes the oldest value from the Moving Average Filter without adding a new value.
     *
     * @return the updated average value of the Moving Average Filter
     */
    public T removeMeasurement();

    /**
     * Gets the average value of the Moving Average Filter.
     *
     * @return the average value of the Moving Average Filter
     */
    public T getAverage();

    /**
     * Gets the number of values in the Moving Average Filter.
     *
     * @return the number of values in the Moving Average Filter
     */
    public int numMeasurements();

    /**
     * Gets an indication as to whether there are enough measurements to calculate an average value.
     *
     * @return <code>true</code> if  there are enough measurements to calculate an average value;
     *         <code>false</code> if there aren't
     */
    public boolean hasAverage();

    /**
     * Clears the Moving Average Filter
     */
    public void clear();
}
