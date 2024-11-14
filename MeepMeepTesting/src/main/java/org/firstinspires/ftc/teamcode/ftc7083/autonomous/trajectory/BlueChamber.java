package org.firstinspires.ftc.teamcode.ftc7083.autonomous.trajectory;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.autonomous.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Autonomous trajectory builder for the Blue Alliance when scoring specimen on the chamber.
 */
@Config
public class BlueChamber {
    // Orientations for the robot, in degrees
    public static double ORIENTATION_TOWARD_WALL = -90;
    public static double ORIENTATION_SPIKE_MARK_1 = 120;
    public static double ORIENTATION_SPIKE_MARK_2 = 90;
    public static double ORIENTATION_SPIKE_MARK_3 = 60;

    // Initial pose for the robot
    public static double INITIAL_POSE_X = 16.0;
    public static double INITIAL_POSE_Y = -62.0;
    public static double INITIAL_HEADING = Math.toRadians(ORIENTATION_TOWARD_WALL);

    // Position for scoring on the high chamber
    public static double CHAMBER_HIGH_X = 5;
    public static double CHAMBER_HIGH_Y = -35;

    // Positions for the spike marks
    public static double BLUE_SPIKE_MARK_X = 58;
    public static double BLUE_SPIKE_MARK_Y = -42;

    // Pickup specimen from wall
    public static double OBSERVATION_ZONE_X = 47;
    public static double OBSERVATION_ZONE_Y = -50;

    // Park in the observation zone
    public static double PARK_X = 50;
    public static double PARK_Y = -60;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public BlueChamber(AutoMecanumDrive drive) {
        this(drive, new Pose2d(new Vector2d(INITIAL_POSE_X, INITIAL_POSE_Y), INITIAL_HEADING));
    }

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the specified pose for the robot, which allows the invoker to override
     * the pose if desired.
     *
     * @param drive       the Mecanum Drive used to move the robot autonomously
     * @param initialPose the initial pose for the robot
     */
    public BlueChamber(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public BlueChamber(TrajectoryActionBuilder actionBuilder) {
        this.actionBuilder = actionBuilder;
    }

    /**
     * Gets the action for running the trajectory for the autonomous driving period.
     *
     * @return the action for running the trajectory for the autonomous driving period
     */
    public Action getTrajectory() {
        IntakeAndScoringSubsystem ias = Robot.getInstance().intakeAndScoringSubsystem;
        return actionBuilder
                // Move to the chamber and score the specimen
                .strafeTo(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Move to the spike marks
                .strafeTo(new Vector2d(BLUE_SPIKE_MARK_X, BLUE_SPIKE_MARK_Y))
                // Move the sample from Spike Mark 1 to the observation zone
                .turnTo(Math.toRadians(ORIENTATION_SPIKE_MARK_1))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to pickup sample
                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to deposit sample
                // Move the sample from Spike Mark 2 to the observation zone
                .turnTo(Math.toRadians(ORIENTATION_SPIKE_MARK_2))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to pickup sample
                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to deposit sample
                // Move the sample from Spike Mark 3 to the observation zone
                .turnTo(Math.toRadians(ORIENTATION_SPIKE_MARK_3))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to pickup sample
                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action to deposit sample
                // Move to the observation zone to pickup specimen 1 from the wall and score on the chamber
                .strafeTo(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y))
                .turnTo(Math.toRadians(ORIENTATION_TOWARD_WALL))
                .stopAndAdd(ias.actionIntakeSpecimen())
                .strafeTo(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Move to the observation zone to pickup specimen 2 from the wall and score on the chamber
                .strafeTo(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y))
                .stopAndAdd(ias.actionIntakeSpecimen())
                .strafeTo(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Move to the observation zone to pickup specimen 3 from the wall and score on the chamber
                .strafeTo(new Vector2d(OBSERVATION_ZONE_X, OBSERVATION_ZONE_Y))
                .stopAndAdd(ias.actionIntakeSpecimen())
                .strafeTo(new Vector2d(CHAMBER_HIGH_X, CHAMBER_HIGH_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber())
                // Park the robot
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .stopAndAdd(ias.actionMoveToStartPosition())
                .build();
    }
}
