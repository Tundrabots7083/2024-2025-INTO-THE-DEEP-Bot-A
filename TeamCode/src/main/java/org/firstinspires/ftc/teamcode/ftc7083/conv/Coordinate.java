package org.firstinspires.ftc.teamcode.ftc7083.conv;

/**
 * A Coordinate represents a position on a plane. The values may be retrieved as cartesian
 * coordinates (x,y) or as polar coordinates (theta,radians).
 */
public class Coordinate {
    private final double x;
    private final double y;

    /**
     * Instantiates a new coordinate system based on the cartesian coordinates (x,y).
     * @param x the position along the x-axis
     * @param y the position along the y-axis
     */
    private Coordinate(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Creates a new coordinate from the polar position of (theta,radius).
     * @param theta the angle of the polar coordinate
     * @param radians the radians of the polar coordinate.
     * @return a new coordinate for the polar position
     */
    public static Coordinate fromPolar(double theta, double radians) {
        double x = radians * Math.cos(theta);
        double y = radians * Math.sin(theta);

        return new Coordinate(x, y);
    }

    /**
     * Creates a new coordinate from the cartesian position of (x,y).
     * @param x the position along the x-axis
     * @param y the position along the y-axis
     * @return a new coordinate for the position using the cartesian coordinates
     */
    public static Coordinate fromCartesian(double x, double y) {
        return new Coordinate(x, y);
    }

    /**
     * Gets the value along the x-axis in the cartesian coordinates system.
     * @return the value along the x-axis
     */
    public double x() {
        return x;
    }

    /**
     * Gets the value along the y-axis in the cartesian coordinates system.
     * @return the value along the y-axis
     */
    public double y() {
        return y;
    }

    /**
     * Gets the angle (theta) in the polar coordinate system.
     * @return the angle (theta) in the polar coordinate system
     */
    public double theta() {
        return Math.atan2(y, x);
    }

    /**
     * Gets the length (radians) in the polar coordinate system.
     * @return the length (radians) in the polar coordinate system
     */
    public double radians() {
        return Math.hypot(x, y);
    }

    /**
     * Gets an array containing the polar coordinates, with the values being {theta,radians}.
     * @return an array containing the polar coordinates, with the values being {theta,radians}
     */
    public double[] polar() {
        return new double[]{theta(), radians()};
    }

    /**
     * Gets an array containing the cartesian coordinates, with the values being {x,y}.
     * @return an array containing the cartesian coordinates, with the values being {x,y}
     */
    public double[] cartesian() {
        return new double[]{x, y};
    }
}
