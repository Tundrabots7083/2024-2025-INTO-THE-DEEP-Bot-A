package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;

/**
 * A Localizer is used to get the current position (<code>getPose2d</code>) of the robot and its
 * current velocity (<code>getVelocity</code>).
 */
public interface Localizer {
    /**
     * Allows the localizer to update the current position and velocity of the robot. This method
     * should be called for every loop through the OpMode.
     */
    void update();

    /**
     * Gets the current location and heading of the robot.
     *
     * @return the current location of the robot.
     */
    Pose2d getPose2d();

    /**
     * Gets the current velocity of the robot along the X and Y axis.
     *
     * @return the current velocity of the robot along the X and Y axis
     */
    PoseVelocity2d getVelocity();
}
