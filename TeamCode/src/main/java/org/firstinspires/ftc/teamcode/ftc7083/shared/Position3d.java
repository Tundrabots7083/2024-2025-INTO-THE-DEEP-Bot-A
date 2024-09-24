package org.firstinspires.ftc.teamcode.ftc7083.shared;

public class Position3d {

    public double x;
    public double y;
    public double z;

    /**
     * This class can create a Position3d object (not Pose3d since it doesn't contain a direction arg)
     * which can be passed to other classes keeping the x, y, and z positions together and in the right order.
     *
     * @param x the x position
     * @param y the y position
     * @param z the z position
     */
    public Position3d (double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
}
