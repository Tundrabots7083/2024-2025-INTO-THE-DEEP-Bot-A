package org.firstinspires.ftc.teamcode.autonomous.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage;
import org.firstinspires.ftc.teamcode.roadrunner.messages.TwoDeadWheelInputsMessage;

/**
 * Localizer for RoadRunner that uses the SparkFunOTOS sensor.
 */
@Config
public class SparkFunOTOSLocalizer implements Localizer {
    public static double OTOS_X_OFFSET = 0.0;
    public static double OTOS_Y_OFFSET = 0.0;
    public static double OTOS_HEADING_OFFSET = 0.0;
    public static double ROBOT_START_X = 0.0;
    public static double ROBOT_START_Y = 0.0;
    public static double ROBOT_START_HEADING = 0.0;
    public static double ANGULAR_SCALAR = 1.0;
    public static double LINEAR_SCALAR = 1.0;
    public static AngleUnit ANGLE_UNIT = AngleUnit.RADIANS;
    public static DistanceUnit LINEAR_UNIT = DistanceUnit.INCH;

    private final SparkFunOTOS otos;
    private boolean initialized = false;
    private Pose2d lastPose;

    /**
     * Instantiate a new SparkFunOTOS localizer.
     *
     * @param otos the SparkFunOTOS to be used by the localizer
     */
    public SparkFunOTOSLocalizer(SparkFunOTOS otos) {
        this.otos = otos;

        setPose2D(OTOS_X_OFFSET, OTOS_Y_OFFSET, OTOS_HEADING_OFFSET);
        setAngularScalar(ANGULAR_SCALAR);
        setLinearScalar(LINEAR_SCALAR);
        setAngularUnit(ANGLE_UNIT);
        setLinearUnit(LINEAR_UNIT);
        calibrateImu();
        setPosition(ROBOT_START_X, ROBOT_START_Y, ROBOT_START_HEADING);
    }

    /**
     * Assuming you've mounted your sensor to a robot and it's not centered,
     * you can specify the offset for the sensor relative to the center of the
     * robot. The units default to inches and degrees, but if you want to use
     * different units, specify them before setting the offset! Note that as of
     * firmware version 1.0, these values will be lost after a power cycle, so
     * you will need to set them each time you power up the sensor. For example, if
     * the sensor is mounted 5 inches to the left (negative X) and 10 inches
     * forward (positive Y) of the center of the robot, and mounted 90 degrees
     * clockwise (negative rotation) from the robot's orientation, the offset
     * would be {-5, 10, -90}. These can be any value, even the angle can be
     * tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
     *
     * @param x x-coordinate
     * @param y y-coordinate
     * @param h heading
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer setPose2D(double x, double y, double h) {
        otos.setOffset(new SparkFunOTOS.Pose2D(x, y, h));
        return this;
    }

    /**
     * Sets the linear unit used by all methods using a pose
     *
     * @param unit Linear unit
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer setLinearUnit(DistanceUnit unit) {
        otos.setLinearUnit(unit);
        return this;
    }

    /**
     * Set the linear scalar, which can compensate for scaling issues with the sensor measurements.
     * This value are lost each time the robot power is cycled, so they must be reset when
     * this occurs.
     * <p>
     * To calibrate the linear scalar, move the
     * robot a known distance and measure the error; do this multiple times at
     * multiple speeds to get an average, then set the linear scalar to the
     * inverse of the error. For example, if you move the robot 100 inches and
     * the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
     *
     * @param scalar the linear scalar value
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer setLinearScalar(double scalar) {
        otos.setLinearScalar(scalar);
        return this;
    }

    /**
     * Sets the angular unit used by all methods using a pose
     *
     * @param unit Angular unit
     */
    public SparkFunOTOSLocalizer setAngularUnit(AngleUnit unit) {
        otos.setAngularUnit(unit);
        return this;
    }

    /**
     * Set the angular scalar, which can compensate for scaling issues with the sensor measurements.
     * This value is lost each time the robot power is cycled, so they must be reset when this
     * occurs.
     * <p>
     * To calibrate the angular scalar, spin the robot by
     * multiple rotations (eg. 10) to get a precise error, then set the scalar
     * to the inverse of the error. Remember that the angle wraps from -180 to
     * 180 degrees, so for example, if after 10 rotations counterclockwise
     * (positive rotation), the sensor reports -15 degrees, the required scalar
     * would be 3600/3585 = 1.004.
     *
     * @param scalar the angular scalar
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer setAngularScalar(double scalar) {
        otos.setLinearScalar(scalar);
        return this;
    }

    /**
     * The IMU on the OTOS includes a gyroscope and accelerometer, which could
     * have an offset. Note that as of firmware version 1.0, the calibration
     * will be lost after a power cycle; the OTOS performs a quick calibration
     * when it powers up, but it is recommended to perform a more thorough
     * calibration at the start of all your OpModes. Note that the sensor must
     * be completely stationary and flat during calibration! When calling
     * calibrateImu(), you can specify the number of samples to take and whether
     * to wait until the calibration is complete. If no parameters are provided,
     * it will take 255 samples and wait until done; each sample takes about
     * 2.4ms, so about 612ms total
     *
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer calibrateImu() {
        otos.calibrateImu();
        return this;
    }

    /**
     * Reset the tracking algorithm - this resets the position to the origin,
     * but can also be used to recover from some rare tracking errors
     *
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer resetTracking() {
        otos.resetTracking();
        return this;
    }

    /**
     * After resetting the tracking, the OTOS will report that the robot is at
     * the origin. If your robot does not start at the origin, or you have
     * another source of location information (eg. vision odometry), you can set
     * the OTOS location to match and it will continue to track from there.
     *
     * @param x x-coordinate
     * @param y y-coordinate
     * @param h heading
     * @return this SparkFunOTOSLocalizer
     */
    public SparkFunOTOSLocalizer setPosition(double x, double y, double h) {
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        return this;
    }

    /**
     * Gets the position measured by the OTOS.
     *
     * @return Position measured by the OTOS
     */
    public SparkFunOTOS.Pose2D getPosition() {
        return otos.getPosition();
    }

    /**
     * Gets the velocity measured by the OTOS
     *
     * @return Velocity measured by the OTOS
     */
    public SparkFunOTOS.Pose2D getVelocity() {
        return otos.getVelocity();
    }

    /**
     * Gets the acceleration measured by the OTOS
     *
     * @return Acceleration measured by the OTOS
     */
    public SparkFunOTOS.Pose2D getAcceleration() {
        return otos.getAcceleration();
    }

    /**
     * Converts a SparkFunOTOS pose to a RoadRunner pose.
     *
     * @param otosPose the SparkFunOTOS pose
     * @return a corresponding RoadRunner pose
     */
    public Pose2d OTOSPoseToRRPose(SparkFunOTOS.Pose2D otosPose) {
        return new Pose2d(otosPose.x, otosPose.y, otosPose.h);
    }

    /**
     * Converts a RoadRunner pose to a SparkFunOTOS pose.
     *
     * @param rrPose the RoadRunner pose
     * @return a corresponding SparkFunOTOS pose
     */
    public SparkFunOTOS.Pose2D RRPoseToOTOSPose(Pose2d rrPose) {
        return new SparkFunOTOS.Pose2D(rrPose.position.x, rrPose.position.y, rrPose.heading.toDouble());
    }

    /**
     * Updates the pose information to be used by RoadRunner.
     *
     * @return
     */
    @Override
    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        SparkFunOTOS.Pose2D vel = otos.getVelocity();
        SparkFunOTOS.Pose2D acc = getAcceleration();

        return null;
        /* TODO: figure this out, or punt and just update AutoMecanumDrive
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS);

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel, angles, angularVelocity));

        Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        double rawHeadingVel = angularVelocity.zRotationRate;
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double parPosDelta = parPosVel.position - lastParPos;
        double perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
        */
    }
}
