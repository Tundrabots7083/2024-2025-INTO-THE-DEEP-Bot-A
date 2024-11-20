package org.firstinspires.ftc.teamcode.ftc7083.filter;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * A Moving Average Filter for the X, Y and Heading values for a webcam. This class could, and
 * perhaps should, be changed to pass in and return a Pose2d instead of the individual X, Y,
 * and heading values.
 */
public class Pose2dMovingAverageFilter extends MovingAverageFilterBase<Pose2d> {
    /**
     * Creates a new Moving Average Filter for Pose2d's with the given window size.
     *
     * @param minNumSamples the minimum number of samples required to calculate an average
     * @param windowSize    the window size for the Moving Average Filter
     */
    public Pose2dMovingAverageFilter(int minNumSamples, int windowSize) {
        super(minNumSamples, windowSize);
    }

    @Override
    public Pose2d startingValue() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    public Pose2d addValue(Pose2d sum, Pose2d newValue) {
        return new Pose2d(
                sum.position.x + newValue.position.x,
                sum.position.y + newValue.position.y,
                sum.heading.toDouble() + newValue.heading.toDouble()
        );
    }

    @Override
    public Pose2d remValue(Pose2d sum, Pose2d oldValue) {
        return new Pose2d(
                sum.position.x - oldValue.position.x,
                sum.position.y - oldValue.position.y,
                sum.heading.toDouble() - oldValue.heading.toDouble()
        );
    }

    @Override
    public Pose2d getAverage(Pose2d sum, int numValues) {
        return new Pose2d(
                sum.position.x / (double) numValues,
                sum.position.y / (double) numValues,
                sum.heading.toDouble() / (double) numValues
        );
    }
}
