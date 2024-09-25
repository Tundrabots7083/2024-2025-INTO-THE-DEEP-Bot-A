package org.firstinspires.ftc.teamcode.ftc7083.shared;

public class Pose2d {

    public double x;
    public double z;
    public double yDirection;

    /**
     * This class can create a Position2d object (not Pose2d since it doesn't contain a direction arg)
     * which can be passed to other classes keeping the x and z positions together and in the right order.
     *
     * @param x the x position
     * @param z the z position
     * @param yDirection the direction with respect to the y axis
     */
    public Pose2d(double x, double z, double yDirection) {
        this.x = x;
        this.z = z;
        this.yDirection = yDirection;
    }
}
