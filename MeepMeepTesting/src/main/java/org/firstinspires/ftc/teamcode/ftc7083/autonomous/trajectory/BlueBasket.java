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
public class BlueBasket {
    // Initial pose for the robot
    public static double INITIAL_POSE_X = -16.0;
    public static double INITIAL_POSE_Y = -62.0;
    public static double INITIAL_HEADING_DEGREES = -90;
    public static double INITIAL_HEADING = Math.toRadians(INITIAL_HEADING_DEGREES);

    // Position for scoring on the high chamber
    public static double CHAMBER_HIGH_X = -10;
    public static double CHAMBER_HIGH_Y = -35;

    // Position for scoring in the high basket
    public static double BASKET_HIGH_X = -52;
    public static double BASKET_HIGH_Y = -52;;

    // Intake positions for samples on the spike marks
    public static double YELLOW_SPIKE_MARK_Y = -45;
    public static double YELLOW_SPIKE_MARK_1_X = -47;
    public static double YELLOW_SPIKE_MARK_2_X = -57;
    public static double YELLOW_SPIKE_MARK_3_X = -60;

    // Park in the observation zone
    public static double PARK_APPROACH_X = -40;
    public static double PARK_X = -25;
    public static double PARK_Y = 0;

    private final TrajectoryActionBuilder actionBuilder;

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the default Pose2d for the robot.
     *
     * @param drive the Mecanum Drive used to move the robot autonomously
     */
    public BlueBasket(AutoMecanumDrive drive) {
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
    public BlueBasket(AutoMecanumDrive drive, Pose2d initialPose) {
        this(drive.actionBuilder(initialPose));
    }

    /**
     * Creates a new autonomous trajectory builder for the Blue Alliance when scoring on the
     * chamber. This uses the specified trajectory action builder for building the trajectories.
     *
     * @param actionBuilder the action builder to use when creating the trajectories
     */
    public BlueBasket(TrajectoryActionBuilder actionBuilder) {
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
                // Pick up the sample from Spike Mark 1 and score in the high basket
                .strafeTo(new Vector2d(YELLOW_SPIKE_MARK_1_X, YELLOW_SPIKE_MARK_Y))
                .turnTo(Math.toRadians(90))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action - pickup sample from spike mark
                .strafeTo(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y))
                .turnTo(Math.toRadians(45))
                .stopAndAdd(ias.actionAcquireSpecimenFromWall()) // TODO: new action - score sample in high basket
                // Pick up the sample from Spike Mark 2 and score in the high basket
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(YELLOW_SPIKE_MARK_2_X, YELLOW_SPIKE_MARK_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action - pickup sample from spike mark
                .strafeTo(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y))
                .turnTo(Math.toRadians(45))
                .stopAndAdd(ias.actionAcquireSpecimenFromWall()) // TODO: new action - score sample in high basket
                // Pick up the sample from Spike Mark 2 and score in the high basket
                .turnTo(Math.toRadians(105))
                .strafeTo(new Vector2d(YELLOW_SPIKE_MARK_3_X, YELLOW_SPIKE_MARK_Y))
                .stopAndAdd(ias.actionScoreSpecimenHighChamber()) // TODO: new action - pickup sample from spike mark
                .strafeTo(new Vector2d(BASKET_HIGH_X, BASKET_HIGH_Y))
                .turnTo(Math.toRadians(45))
                .stopAndAdd(ias.actionAcquireSpecimenFromWall()) // TODO: new action - score scample in high basket
                // Park the robot
                .turnTo(Math.toRadians(180))
                .strafeTo(new Vector2d(PARK_APPROACH_X, PARK_Y))
                .strafeTo(new Vector2d(PARK_X, PARK_Y))
                .stopAndAdd(ias.actionAcquireSpecimenFromWall()) // TODO: new action - touch chamber bar
                .build();
    }
}
