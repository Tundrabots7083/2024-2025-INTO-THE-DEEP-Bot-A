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
    public static double ARM_LENGTH = 21.0;
    // Height from the field to the center of rotation of the arm in inches.
    public static double ARM_HEIGHT = 15.0;
    // Distance from the front of the robot to the back of the robot in inches.
    public static double ROBOT_LENGTH = 18.0;

    // Intake constants
    public static double POSITION_NEUTRAL_HORIZONTAL_DISTANCE = ARM_LENGTH;
    public static double POSITION_INTAKE_HORIZONTAL_DISTANCE = 30.0;
    public static double POSITION_INTAKE_HEIGHT = 1.5;

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
    public static double HIGH_CHAMBER_SCORING_Y = HIGH_CHAMBER_HEIGHT - ARM_HEIGHT + 1;
    public static double HIGH_BASKET_SCORING_X = ARM_LENGTH;
    public static double HIGH_BASKET_SCORING_Y = HIGH_BASKET_HEIGHT - ARM_HEIGHT + 1;

    // Other scoring constants
    public static double SCORE_SPECIMEN_ARM_ANGLE_DECREMENT = 10.0;

    private boolean movingToPosition = false;

    private double targetArmAngle = 0.0;
    private double targetSlideExtension = 0.0;

    private final Telemetry telemetry;
    private final Robot robot;

    /**
     * Constructor
     */
    public IntakeAndScoringSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = Robot.getInstance();
    }

    /**
     * Called by FTC opmode loop to take action if needed.
     */
    @Override
    public void execute() {
        if (movingToPosition) {
            // The arm is currently moving to a requested position
            if ((robot.arm.isAtTarget() && (robot.arm.getShoulderAngle() == targetArmAngle)) &&
                    (robot.linearSlide.isAtTarget() &&
                            (robot.linearSlide.getCurrentLength() == targetSlideExtension))) {
                // Both the arm and the slide are at their target positions.
                // We are done.
                movingToPosition = false;
            }
            else if (robot.arm.isAtTarget() && (robot.arm.getShoulderAngle() == targetArmAngle)) {
                // Arm is at target angle but slide is not at the final target length
                robot.linearSlide.setLength(targetSlideExtension);
            }
            else {
                // Neither the arm nor the slide are at their final positions.
                double currentArmAngle = robot.arm.getShoulderAngle();
                double currentSlideExtension = robot.linearSlide.getCurrentLength();
                // Calculate the length of the base of the right triangle made by the
                // current arm angle and hypotenuse
                double rightTriangleBase = getX(currentArmAngle, currentSlideExtension + ARM_LENGTH);
                // Calculate the height of the right triangle made by the current arm
                // angle and base
                double rightTriangleHeight = getY(currentArmAngle, rightTriangleBase);
                // Calculate the new slide extension (hypotenuse) for this triangle.
                robot.linearSlide.setLength(getHypotenuse(rightTriangleBase, rightTriangleHeight) - ARM_LENGTH);
            }
        }
        robot.arm.execute();
        robot.linearSlide.execute();
        robot.wrist.execute();
        robot.claw.execute();
    }

    /**
     * Moves the arm and slide to the position (x,y) on the plane
     * of the robot arm.  The point (0,0) on this plane is the center
     * of the shoulder around which the arm rotates.  The tip of the
     * claw should end up at the point (x,y).
     *
     * @param x  The horizontal distance of the end of the claw from (0,0) along
     *           the x-axis in inches.
     * @param y  The vertical distance of the end of the claw from (0,0) along
     *           the y-axis in inches.
     */
    public void moveToPosition(double x, double y) {
        //todo: Not sure if I need this condition? If called, just override w/ new position?
        if (!movingToPosition) {
            // The arm is not currently moving to a requested position

            // Make sure that the x value does not exceed the
            // maximum robot horizontal length allowed for the game
            if (x > MAX_EXTENDED_ROBOT_LENGTH) { x = MAX_EXTENDED_ROBOT_LENGTH; }
            targetArmAngle = getAngle(x, y);
            targetSlideExtension = getHypotenuse(x, y) - ARM_LENGTH;
            robot.arm.setShoulderAngle(targetArmAngle);
            movingToPosition = true;
        }
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen.
     * This will lower and extend the arm so the claw may be used to pickup a sample or specimen.
     */
    public void moveToIntakePosition() {
        moveToPosition(POSITION_INTAKE_HORIZONTAL_DISTANCE, POSITION_INTAKE_HEIGHT);
        robot.claw.open();
        telemetry.addData("[Intake] position", "intake");
    }

    /**
     * Moves the arm after a sample or specimen has been picked up. This will raise and retract the
     * arm so the robot may be maneuvered into a scoring position by the IntakeAndScoringSubsystem.
     */
    public void moveToNeutralPosition() {
        moveToPosition(POSITION_NEUTRAL_HORIZONTAL_DISTANCE, ARM_HEIGHT);
        telemetry.addData("[Intake] position", "neutral");
    }

    /**
     * Uses the claw to grasp a scoring element (sample or specimen). The arm is not moved when
     * acquiring the scoring element; the <code>moveToIntakePosition</code> method is used for
     * that purpose.
     */
    public void closeClaw() {
        robot.claw.close();
        telemetry.addData("[Intake] claw", "close");
    }

    /**
     * Releases a scoring element (sample or specimen). This will open the claw so that the
     * scoring element is released by the subsystem. The claw will be opened, but the arm
     * is not moved; the <code>moveToIntakePosition</code> method is used for moving the arm.
     */
    public void openClaw() {
        robot.claw.open();
        telemetry.addData("[Intake] claw", "open");
    }

    /**
     * Moves the arm to a neutral position. This will close the claw and move the arm to the same
     * position as when a scoring element has been acquired.
     */
    public void reset() {
        robot.claw.close();
        moveToNeutralPosition();
        telemetry.addLine("[Intake] reset intake subsystem");
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high chamber bar.
     */
    public void moveToChamberHighScoringPosition() {
        moveToPosition(HIGH_CHAMBER_SCORING_X, HIGH_CHAMBER_SCORING_Y);
    }

    /**
     * Moves the arm, slide, wrist, and claw to score a specimen
     * on either the high or low chamber bar.
     */
    public void scoreSpecimen() {
        double currShoulderAngle = robot.arm.getShoulderAngle();
        robot.arm.setShoulderAngle(currShoulderAngle - SCORE_SPECIMEN_ARM_ANGLE_DECREMENT);
        robot.arm.execute();
        //todo:  do I need to open claw?
    }

    /**
     * Moves the arm, slide, wrist, and claw to prepare the robot
     * to score on the high basket.
     */
    public void moveToBasketHighScoringPosition() {
        moveToPosition(HIGH_BASKET_SCORING_X, HIGH_BASKET_SCORING_Y);
    }

    /**
     * Moves the arm, slide, wrist, and claw to score a sample
     * anywhere (high and low baskets and floor).
     */
    public void scoreSample() {
        // Samples can be scored in a basket or on the floor
        // Need to open claw to release the sample.
        robot.claw.open();
        robot.claw.close();
    }
}
