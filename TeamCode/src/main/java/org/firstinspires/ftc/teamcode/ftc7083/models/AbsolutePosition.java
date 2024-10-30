package org.firstinspires.ftc.teamcode.ftc7083.models;


//this is a place holder until we have roadrunner imported in the project
public class AbsolutePosition {
    public double X;
    public double Y;
    public double Heading;

    public AbsolutePosition(double x, double y, double heading) {
        this.X = x;
        this.Y = y;
        this.Heading = heading;

    }

    @Override
    public String toString() {
        return " AbsolutePosition [x = " + X + ", y  =" + Y+ ", heading  =" + Heading + "]";
    }
}
