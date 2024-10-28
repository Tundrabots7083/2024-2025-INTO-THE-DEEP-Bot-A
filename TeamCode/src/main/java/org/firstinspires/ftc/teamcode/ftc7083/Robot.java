package org.firstinspires.ftc.teamcode.ftc7083;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

import java.util.Arrays;
import java.util.List;

/**
 * The Robot. This is implemented as a singleton, meaning there is one robot instance that exists.
 */
@Config
public class Robot {
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
    public static double SPARKFUN_LINEAR_SCALAR = 1.0;
    public static double SPARKFUN_ANGULAR_SCALAR = 1.0;


    private static Robot robot = null;

    public final Telemetry telemetry;

    // Subsystems
    public final MecanumDrive mecanumDrive;
    public final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    public final Webcam webcamLeft;
    public final Webcam webcamRight;
    public final Arm arm;
    public final LinearSlide linearSlide;
    public final Wrist wrist;
    public final Claw claw;
    public final SparkFunOTOSCorrected otos;

    // All webcams
    public final List<Webcam> webcams;

    // All lynx module huba
    public final List<LynxModule> allHubs;

    /**
     * Creates a new instance of the robot.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     */
    private Robot(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = this;
        this.telemetry = telemetry;

        // Enable bulk reads. This is almost always the "correct" answer, and can speed up loop
        // times. We will be managing the bulk read caches manually, which requires each OpMode
        // to clear the cache at the start of each loop.
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Instantiate all the hardware on the robot
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);

        webcamLeft = new Webcam(hardwareMap, telemetry, Webcam.Location.LEFT);
        webcamRight = new Webcam(hardwareMap, telemetry, Webcam.Location.RIGHT);
        webcams = Arrays.asList(webcamLeft, webcamRight);

        arm = new Arm(hardwareMap, telemetry);
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        wrist = new Wrist(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        intakeAndScoringSubsystem = new IntakeAndScoringSubsystem(hardwareMap, telemetry);

        otos = hardwareMap.get(SparkFunOTOSCorrected.class, "sensor_otos");
        initializeSparkFunOTOS();

        this.telemetry.addLine("[Robot] initialized");
        this.telemetry.update();
    }

    public void initializeSparkFunOTOS() {
        // Don't change the units, it will stop FTCDashboard field view from working properly
        // and might cause various other issues
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(SPARKFUN_OTOS_OFFSET_X, SPARKFUN_OTOS_OFFSET_Y, Math.toRadians(SPARKFUN_OTOS_HEADING_IN_DEGREES));
        otos.setOffset(offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(SPARKFUN_LINEAR_SCALAR));
        System.out.println(otos.setAngularScalar(SPARKFUN_ANGULAR_SCALAR));

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your programs. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total

        // RR localizer note: It is technically possible to change the number of samples to slightly reduce init times,
        // however, I found that it caused pretty severe heading drift.
        // Also, if you're careful to always wait more than 612ms in init, you could technically disable waitUntilDone;
        // this would allow your OpMode code to run while the calibration occurs.
        // However, that may cause other issues.
        // In the future I hope to do that by default and just add a check in updatePoseEstimate for it
        telemetry.addData("[OTOS] calibrate IMU", (otos.calibrateImu(255, true)));
        telemetry.addLine("OTOS calibration complete!");
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry) {
        return init(hardwareMap, telemetry, OpModeType.TELEOP);
    }

    /**
     * Initializes the hardware mechanisms for the robot. This creates the singleton that is retrieved
     * using the <code>getInstance</code> method.
     *
     * @param hardwareMap hardware map for the robot.
     * @param telemetry   telemetry class for displaying data.
     * @param opModeType  the type of opmode the robot is being used for
     * @return the robot instance
     */
    public static Robot init(@NonNull HardwareMap hardwareMap, @NonNull Telemetry telemetry, OpModeType opModeType) {
        robot = new Robot(hardwareMap, telemetry, opModeType);
        return robot;
    }

    /**
     * Gets the singleton instance of the robot.
     */
    public static Robot getInstance() {
        return robot;
    }

    /**
     * Gets a string representation of the robot.
     *
     * @return a string representation of the robot
     */
    @NonNull
    @Override
    public String toString() {
        return "Robot{" +
                "mecanumDrive=" + mecanumDrive +
                ", webcam=" + webcamLeft +
                '}';
    }

    // Enum to specify opmode type
    public enum OpModeType {
        /**
         * Driver controlled OpMode
         */
        TELEOP,
        /**
         * Autonomous OpMode
         */
        AUTO
    }
}
