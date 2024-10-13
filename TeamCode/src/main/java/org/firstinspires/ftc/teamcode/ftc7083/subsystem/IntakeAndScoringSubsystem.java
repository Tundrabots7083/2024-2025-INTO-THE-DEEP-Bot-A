package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

/**
 * This class uses the Arm, LinearSlide, Wrist, and Claw subsystems to pick up, control,
 * and score samples and specimens.
 */
@Config
public class IntakeAndScoringSubsystem extends SubsystemBase {

    // Robot measurements
    // Length of the arm and the wrist with zero extension in inches.
    public static double ARM_LENGTH = 23.0;
    // Height from the field to the center of rotation of the arm in inches.
    public static double ARM_HEIGHT = 15.0;
    // Distance from the front of the robot to the back of the robot in inches.
    public static double ROBOT_LENGTH = 18.0;
    // Distance the arm can reach from the center of rotation of the arm in inches.
    public static double MAX_ARM_LENGTH = 40.5;

    // Intake constants
    public static double START_X = 14;
    public static double START_Y = 0.0;
    public static double NEUTRAL_X = ARM_LENGTH;
    public static double NEUTRAL_Y = ARM_HEIGHT;
    public static double INTAKE_X = 23.0;
    public static double INTAKE_Y = 1.0;

    // Start position

    // Heights of scoring places for game are in inches
    public static double HIGH_CHAMBER_HEIGHT = 26.0;
    public static double LOW_CHAMBER_HEIGHT = 13.0;
    public static double HIGH_BASKET_HEIGHT = 43.0;
    public static double LOW_BASKET_HEIGHT = 25.75;

    // Maximum horizontal length of robot when extended
    public static double MAX_EXTENDED_ROBOT_LENGTH = 40.0;

    // (x, y) distances in inches from the center of rotation of
    // the arm that the scoring subsystem needs to reach to score in
    // different places.
    public static double HIGH_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double HIGH_CHAMBER_SCORING_Y = HIGH_CHAMBER_HEIGHT + 1;
    public static double HIGH_BASKET_SCORING_X = ARM_LENGTH;
    public static double HIGH_BASKET_SCORING_Y = HIGH_BASKET_HEIGHT + 1;
    public static double LOW_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double LOW_CHAMBER_SCORING_Y = LOW_CHAMBER_HEIGHT + 1;
    public static double LOW_BASKET_SCORING_X = ARM_LENGTH;
    public static double LOW_BASKET_SCORING_Y = LOW_BASKET_HEIGHT + 1;

    // Other scoring constants
    public static double SCORE_SPECIMEN_HEIGHT_DECREMENT = 3.0;

    private final Telemetry telemetry;
    private final Robot robot;
    private double targetX;
    private double targetY;

    /**
     * Constructor
     */
    public IntakeAndScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = Robot.getInstance();
    }

    /**
     * Returns indication as to whether the intake and scoring subsystem has reached the target
     * position.
     *
     * @return <code>true</code> if the intake and scoring subsystem is at the target position;
     * <code>false</code> otherwise
     */
    public boolean isAtTarget() {
        return robot.arm.isAtTarget() && robot.linearSlide.isAtTarget();
    }

    /**
     * Called by FTC opmode loop to take action if needed.
     */
    @Override
    public void execute() {
        robot.linearSlide.setLength(getIntermediateSlideLength());

        // Update each components of the intake/scoring subsystem.
        robot.arm.execute();
        robot.linearSlide.execute();
        robot.wrist.execute();
        robot.claw.execute();
    }

    /**
     * Calculate and set the intermediate slide length. This ensures the arm can reach, but
     * never exceed, the target X distance of the final position for the arm and slide. If
     * the intermediate slide length hasn't changed, then this will be a no-op in the
     * `LinearSlide` subsystem, so there is no need to keep track of it here.
     *
     * @return the intermediate slide length given the current arm angle.
     */
    private double getIntermediateSlideLength() {
        double armAngle = robot.arm.getCurrentAngle();
        double slideLength = robot.linearSlide.getCurrentLength();
        double hypotenuse = slideLength + ARM_LENGTH;
        double intermediateY = getY(armAngle, hypotenuse);
        double intermediateSlideLength = Math.hypot(targetX, intermediateY) - ARM_LENGTH;

        telemetry.addData("[IAS] arm angle", armAngle);
        telemetry.addData("[IAS] hypot", hypotenuse);
        telemetry.addData("[IAS] int Y", intermediateY);
        telemetry.addData("[IAS] int slide len", intermediateSlideLength);

        return intermediateSlideLength;
    }

    /**
     * Gets the current length along the x-axis.
     *
     * @return the current length along the x-axis
     */
    public double getCurrentX() {
        double angle = robot.arm.getCurrentAngle();
        double hypotenuse = robot.linearSlide.getCurrentLength() + ARM_LENGTH;
        return getX(angle, hypotenuse);
    }

    /**
     * Gets the current height along the y-axis.
     *
     * @return the current height along the y-axis
     */
    public double getCurrentY() {
        double angle = robot.arm.getCurrentAngle();
        double currentX = getCurrentX();
        return getY(angle, currentX);
    }

    /**
     * Moves the arm and slide to the position (x,y) on the plane
     * of the robot arm.  The point (0,0) on this plane is the center
     * of the shoulder around which the arm rotates.  The tip of the
     * claw should end up at the point (x,y).
     *
     * @param x The horizontal distance of the end of the claw from (0,0) along
     *          the x-axis in inches.
     * @param y The vertical distance of the end of the claw from (0,0) along
     *          the y-axis in inches.
     */
    public void moveToPosition(double x, double y) {
        x = Math.min(x, MAX_EXTENDED_ROBOT_LENGTH);
        // Slight optimization by only recalculating the target shoulder angle if either the new
        // target X or new target Y values have changed
        if (targetX != x || targetY != y) {
            targetX = x;
            targetY = Math.max(y - ARM_HEIGHT, -ARM_HEIGHT);

            // Calculate and set the target arm angle, based on the target X and Y coordinates.
            // This is done in moveToPosition as it is only calculated and set once when either the
            // X and/or Y coordinates change.
            robot.arm.setTargetAngle(getAngle(targetX, targetY));

            telemetry.addData("[IAS] X", x);
            telemetry.addData("[IAS] Y", y);
            telemetry.addData("[IAS] target X", targetX);
            telemetry.addData("[IAS] target Y", targetY);
        }
    }

    /**
     * Moves the subsystem to the starting position, with the claw closed.
     */
    public void moveToStartPosition() {
        moveToPosition(START_X, START_Y);
        robot.claw.close();
        telemetry.addData("[IAS] position", "start");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen.
     * This will lower and extend the arm so the claw may be used to pickup a sample or specimen.
     */
    public void moveToIntakePosition() {
        moveToPosition(INTAKE_X, INTAKE_Y);
        robot.claw.open();
        telemetry.addData("[IAS] position", "intake");
    }

    /**
     * Moves the arm after a sample or specimen has been picked up. This will raise and retract the
     * arm so the robot may be maneuvered into a scoring position by the IntakeAndScoringSubsystem.
     */
    public void moveToNeutralPosition() {
        moveToPosition(NEUTRAL_X, NEUTRAL_Y);
        telemetry.addData("[IAS] position", "neutral");
    }

    /**
     * Uses the claw to grasp a scoring element (sample or specimen). The arm is not moved when
     * acquiring the scoring element; the <code>moveToIntakePosition</code> method is used for
     * that purpose.
     */
    public void closeClaw() {
        robot.claw.close();
        telemetry.addData("[IAS] claw", "close");
    }

    /**
     * Releases a scoring element (sample or specimen). This will open the claw so that the
     * scoring element is released by the subsystem. The claw will be opened, but the arm
     * is not moved; the <code>moveToIntakePosition</code> method is used for moving the arm.
     */
    public void openClaw() {
        robot.claw.open();
        telemetry.addData("[IAS] claw", "open");
    }

    /**
     * Moves the arm to a neutral position. This will close the claw and move the arm to the same
     * position as when a scoring element has been acquired.
     */
    public void reset() {
        closeClaw();
        moveToNeutralPosition();
        telemetry.addLine("[IAS] reset intake");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high chamber bar.
     */
    public void moveToChamberHighScoringPosition() {
        moveToPosition(HIGH_CHAMBER_SCORING_X, HIGH_CHAMBER_SCORING_Y);
        telemetry.addData("[IAS] position", "high chamber");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the low chamber bar.
     */
    public void moveToChamberLowScoringPosition() {
        moveToPosition(LOW_CHAMBER_SCORING_X, LOW_CHAMBER_SCORING_Y);
        telemetry.addData("[IAS] position", "low chamber");
    }

    /**
     * Moves the arm, slide, wrist, and claw to score a specimen
     * on either the high or low chamber bar.
     */
    public void scoreSpecimen() {
        double y = getY(robot.arm.getCurrentAngle(), targetX) - SCORE_SPECIMEN_HEIGHT_DECREMENT;
        moveToPosition(targetX, y);
        telemetry.addData("[IAS] position", "score specimen");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high basket.
     */
    public void moveToBasketHighScoringPosition() {
        moveToPosition(HIGH_BASKET_SCORING_X, HIGH_BASKET_SCORING_Y);
        telemetry.addData("[IAS] position", "high basket");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the low basket.
     */
    public void moveToBasketLowScoringPosition() {
        moveToPosition(LOW_BASKET_SCORING_X, LOW_BASKET_SCORING_Y);
        telemetry.addData("[IAS] position", "low basket");
    }
}
