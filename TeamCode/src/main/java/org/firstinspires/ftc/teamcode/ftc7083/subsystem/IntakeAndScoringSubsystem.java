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
    public static double ARM_LENGTH = 22.0;
    // Height from the field to the center of rotation of the arm in inches.
    public static double ARM_HEIGHT = 16.5;
    // Distance from the front of the robot to the back of the robot in inches.
    public static double ROBOT_LENGTH = 18.0;
    // Distance the arm can reach from the center of rotation of the arm in inches.
    public static double MAX_ARM_LENGTH = 40.5;

    // Intake constants
    public static double START_X = 14;
    public static double START_Y = 0.0;
    public static double NEUTRAL_X = ARM_LENGTH;
    public static double NEUTRAL_Y = ARM_HEIGHT;
    public static double RETRACT_X = ARM_LENGTH;
    public static double RETRACT_Y = 5.0;
    public static double INTAKE_SHORT_X = 27.0;
    public static double INTAKE_SHORT_Y = 1.6;
    public static double INTAKE_LONG_X = 36.0;
    public static double INTAKE_LONG_Y = 1.6;

    // Heights of scoring places for game are in inches
    public static double HIGH_CHAMBER_HEIGHT = 26.0;
    public static double LOW_CHAMBER_HEIGHT = 13.0;
    public static double HIGH_BASKET_HEIGHT = 48.6;
    public static double LOW_BASKET_HEIGHT = 25.75;

    // Maximum horizontal length of robot when extended
    public static double MAX_EXTENDED_ROBOT_LENGTH = 40.0;

    // (x, y) distances in inches from the center of rotation of
    // the arm that the scoring subsystem needs to reach to score in
    // different places.
    public static double HIGH_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double HIGH_CHAMBER_SCORING_Y = HIGH_CHAMBER_HEIGHT + 2.5;
    public static double LOW_CHAMBER_SCORING_X = ARM_LENGTH;
    public static double LOW_CHAMBER_SCORING_Y = LOW_CHAMBER_HEIGHT + 3.5;
    public static double HIGH_BASKET_SCORING_X = ARM_LENGTH;
    public static double HIGH_BASKET_SCORING_Y = HIGH_BASKET_HEIGHT + 5.5;
    public static double LOW_BASKET_SCORING_X = ARM_LENGTH;
    public static double LOW_BASKET_SCORING_Y = LOW_BASKET_HEIGHT + 5.5;

    // Other scoring constants
    public static double MOVE_ARM_AMOUNT = 3.0;

    private final Telemetry telemetry;
    private final Robot robot;
    private double targetX = RETRACT_X;
    private double targetY = RETRACT_Y;

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
        // Update each components of the intake/scoring subsystem.
        robot.arm.execute();
        robot.linearSlide.execute();
        robot.wrist.execute();
        robot.claw.execute();

        telemetry.addData("[IAS] target X", targetX);
        telemetry.addData("[IAS] target Y", targetY);
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

        // If the target X and/or target Y have changed, then recalculate the arm angle and
        // linear slide length
        if (targetX != x || targetY != y) {
            targetX = x;
            targetY = y;

            // Set the arm angle and slide length to reach the target X and Y coordinates
            setArmAngle();
            setSlideLength();
        }
    }

    /**
     * Calculate and set the target arm angle, accounting for the fixed height of the post on which
     * the arm is attached
     */
    private void setArmAngle() {
        double adjustedY = targetY - ARM_HEIGHT;
        double armAngle = getAngle(targetX, adjustedY);
        telemetry.addData("[IAS] arm angle", armAngle);
        robot.arm.setTargetAngle(armAngle);
        // Don't use the wrist movement, as the arm can't reach the ground to pickup samples if the
        // wrist is articulated
        // robot.wrist.setPitch(-armAngle * 2.0);
    }

    /**
     * Set the linear slide length, accounting for the fixed length of the arm when the
     * linear slide is fully retracted
     */
    private void setSlideLength() {
        double adjustedY = targetY - ARM_HEIGHT;
        double slideLength = Math.hypot(targetX, adjustedY) - ARM_LENGTH;
        telemetry.addData("[IAS] slide length", slideLength);
        robot.linearSlide.setLength(slideLength);
    }

    /**
     * Moves the subsystem to the starting position, with the claw closed.
     */
    public void moveToStartPosition() {
        moveToPosition(START_X, START_Y);
        telemetry.addData("[IAS] position", "start");
    }

    /**
     * Fully retract the linear slide while not changing the arm angle.
     */
    public void retractLinearSlide() {
        // Get angle on the triangle. This is not going to change as the arm is retracted, as the
        // new triangle will be complementary to the current triangle.
        double theta = getAngle(targetX, targetY);

        // Fully retract the slide by setting the hypotenuse to the arm length, which will effectively
        // set the slide length to zero. Use the hypotenuse to calculate the new X and Y coordinates
        // for the right triangle.
        double hypotenuse = ARM_LENGTH;
        double x = getX(theta, hypotenuse);
        double y = getY(theta,hypotenuse);
        moveToPosition(x, y);
        telemetry.addData("[IAS] position","retract slide");
    }

    /**
     * Retracts the linear slide from the submersible, slightly raising the arm so the slide does
     * not hit the 2" lip around the submersible.
     */
    public void retractFromSubmersible() {
        moveToPosition(RETRACT_X, RETRACT_Y);
        telemetry.addData("[IAS] position", "retract submersible");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative close to the front of the robot. This will lower and extend the arm so the claw
     * may be used to pickup a sample or specimen.
     */
    public void moveToIntakeShortPosition() {
        moveToPosition(INTAKE_SHORT_X, INTAKE_SHORT_Y);
        telemetry.addData("[IAS] position", "intake short");
    }

    /**
     * Moves the subsystem to a position where it may acquire a sample or a specimen that is
     * relative far from the front of the robot. This will lower and extend the arm so the claw
     * may be used to pickup a sample or specimen.
     */
    public void moveToIntakeLongPosition() {
        moveToPosition(INTAKE_LONG_X, INTAKE_LONG_Y);
        telemetry.addData("[IAS] position", "intake long");
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
     * Moves the arm, slide, wrist, and claw to downward.
     */
    public void lowerArm() {
        double newY = targetY - MOVE_ARM_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "lower arm");
    }

    /**
     * Moves the arm, slide, wrist, and claw to upward.
     */
    public void raiseArm() {
        double newY = targetY + MOVE_ARM_AMOUNT;
        newY = Math.max(newY, 0.0);
        moveToPosition(targetX, newY);
        telemetry.addData("[IAS] position", "raise arm");
    }

    /**
     * Increases the slide length by the amount provided. A positive value extends the slide;
     * a negative value retracts the arm.
     *
     * @param amount the amount by which to adjust the arm angle.
     */
    public void adjustX(double amount) {
        targetX += amount;
        setSlideLength();
    }

    /**
     * Increases the arm angle by the amount provided. A positive value moves the arm up; a negative
     * value moves the arm down.
     *
     * @param amount the amount by which to adjust the arm angle.
     */
    public void adjustY(double amount) {
        targetY += amount;
        setArmAngle();
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

   /* public void scoreHighBasket() {
        openClaw();
        robot.claw.execute();
        wait(1);
        retractLinearSlide();
        robot.arm.execute();
    }*/
}
