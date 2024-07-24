package org.firstinspires.ftc.teamcode.feedback;

/**
 * Interface for calculating the feed forward value for a PIDF controller.
 */
@FunctionalInterface
public interface FeedForward {
    /**
     * Calculate the feed forward value.
     *
     * @param target The target position trying to be reached.
     * @return the feed forward value for the PIDF controller.
     */
    double calculate(double target);
}