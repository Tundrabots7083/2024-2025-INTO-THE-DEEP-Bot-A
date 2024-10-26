package org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

/**
 * Parameters originally embedded in RoadRunner's MecanumDrive, split out here for ease of finding
 * and configuring.
 */
@Config
public class Params {
    // Assuming you've mounted your sensor to a robot and it's not centered,
    // you can specify the offset for the sensor relative to the center of the
    // robot. The units default to inches and degrees, but if you want to use
    // different units, specify them before setting the offset! Note that as of
    // firmware version 1.0, these values will be lost after a power cycle, so
    // you will need to set them each time you power up the sensor. For example, if
    // the sensor is mounted 5 inches to the left (negative X) and 10 inches
    // forward (positive Y) of the center of the robot, and mounted 90 degrees
    // clockwise (negative rotation) from the robot's orientation, the offset
    // would be {-5, 10, -90}. These can be any value, even the angle can be
    // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).

    // RR localizer note: These units are inches and radians.
    public static double SPARKFUN_OTOS_OFFSET_X = 0.0;
    public static double SPARKFUN_OTOS_OFFSET_Y = 0.0;
    public static double SPARKFUN_OTOS_HEADING_IN_DEGREES = 0.0;
    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(SPARKFUN_OTOS_OFFSET_X, SPARKFUN_OTOS_OFFSET_Y, Math.toRadians(SPARKFUN_OTOS_HEADING_IN_DEGREES));

    // Here we can set the linear and angular scalars, which can compensate for
    // scaling issues with the sensor measurements. Note that as of firmware
    // version 1.0, these values will be lost after a power cycle, so you will
    // need to set them each time you power up the sensor. They can be any value
    // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
    // first set both scalars to 1.0, then calibrate the angular scalar, then
    // the linear scalar. To calibrate the angular scalar, spin the robot by
    // multiple rotations (eg. 10) to get a precise error, then set the scalar
    // to the inverse of the error. Remember that the angle wraps from -180 to
    // 180 degrees, so for example, if after 10 rotations counterclockwise
    // (positive rotation), the sensor reports -15 degrees, the required scalar
    // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
    // robot a known distance and measure the error; do this multiple times at
    // multiple speeds to get an average, then set the linear scalar to the
    // inverse of the error. For example, if you move the robot 100 inches and
    // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
    public double linearScalar = 1.0;
    public double angularScalar = 1.0;
}
