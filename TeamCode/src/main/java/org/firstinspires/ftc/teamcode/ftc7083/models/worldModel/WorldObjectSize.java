package org.firstinspires.ftc.teamcode.models.worldModel;

public class WorldObjectSize {
    public double width;
    public double height;

    public WorldObjectSize(double width, double height) {
        this.width = width;
        this.height = height;

    }

    @Override
    public String toString() {
        return " WorldObjectSize [width = " + width + ", height = " + height + "]";
    }
}
