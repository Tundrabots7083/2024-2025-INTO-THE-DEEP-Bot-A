package org.firstinspires.ftc.teamcode.ftc7083.shared;

public class Position2d {

    public double x;
    public double z;

    /**
     * This class can create a Position2d object (not Pose2d since it doesn't contain a direction arg)
     * which can be passed to other classes keeping the x and z positions together and in the right order.
     *
     * @param x the x position
     * @param z the z position
     */
    public Position2d(double x, double z) {
        this.x = x;
        this.z = z;
    }
}
