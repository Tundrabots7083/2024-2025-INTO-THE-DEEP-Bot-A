package org.firstinspires.ftc.teamcode.ftc7083.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.Robot;

/**
 * The IntakeSubsystem is responsible for acquiring up samples and specimens so they may be
 * scored by the ScoringSubsystem. The IntakeSubsystem manages the following subsystems on the
 * robot:
 * <ul>
 *     <li>
 *         <em>Arm</em>: used to raise or lower the scoring subsystem
 *     </li>
 *     <li>
 *         <em>LinearSlide</em>: used to extend or retract the scoring subsystem
 *     </li>
 *     <li>
 *         <em>Wrist</em>: used to change the angle of the claw
 *     </li>
 *     <li>
 *         <em>Claw</em>: used to acquire and deposit the scoring elements (samples and specimens)
 *     </li>
 * </ul>
 */
@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double RETRACTED_ARM_LENGTH = 21.0;
    public static double POSITION_NEUTRAL_HORIZONTAL_DISTANCE = RETRACTED_ARM_LENGTH;
    public static double POSITION_NEUTRAL_HEIGHT = 11.0;
    public static double POSITION_INTAKE_HORIZONTAL_DISTANCE = 30.0;
    public static double POSITION_INTAKE_HEIGHT = 1.5;

    private final Telemetry telemetry;
    private final Robot robot;
    private double targetArmAngle;
    private double targetArmReach;

    private boolean updateIntakeSubsystem = false;

    /**
     * Instantiates a new instance of the intake subsystem.
     *
     * @param hardwareMap the mapping of hardware on the control hub
     * @param telemetry   the telemetry to be used to output data to the driver station and FTC dashboard
     */
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.robot = Robot.getInstance();
    }

    /**
     * Moves the scoring subsystem to a position where it may acquire a sample or a specimen.
     * This will lower and extend the arm so the claw may be used to pickup a sample or specimen.
     */
    public void moveToIntakePosition() {
        targetArmAngle = getArmAngle(POSITION_INTAKE_HORIZONTAL_DISTANCE, POSITION_INTAKE_HEIGHT);
        targetArmReach = POSITION_INTAKE_HORIZONTAL_DISTANCE;
        updateIntakeSubsystem = true;
        robot.claw.open();
        telemetry.addData("[Intake] position", "intake");
    }

    /**
     * Moves the arm after a sample or specimen has been picked up. This will raise and retract the
     * arm so the robot may be maneuvered into a scoring position by the ScoringSubsystem.
     */
    public void moveToNeutralPosition() {
        targetArmAngle = getArmAngle(POSITION_NEUTRAL_HORIZONTAL_DISTANCE, POSITION_NEUTRAL_HEIGHT);
        targetArmReach = POSITION_NEUTRAL_HORIZONTAL_DISTANCE;
        updateIntakeSubsystem = true;
        telemetry.addData("[Intake] position", "neutral");
    }

    /**
     * Uses the claw to grasp a scoring element (sample or specimen). The arm is not moved when
     * acquiring the scoring element; the <code>moveToAcquiredPosition</code> method is used for
     * that purpose.
     */
    public void acquireScoringElement() {
        robot.claw.close();
        telemetry.addData("[Intake] claw", "close");
    }

    /**
     * Releases a scoring element (sample or specimen). This will open the claw so that the
     * scoring element is released by the intake subsystem. The claw will be opened, but the arm
     * is not moved; the <code>moveToIntakePosition</code> method is used for moving the arm.
     */
    public void depositScoringElement() {
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
     * Disable updates to the intake subsystem on each loop through the OpMode. This is used when
     * the scoring subsystem takes over control of the shared subsystem components and this
     * subsystem should no longer make any corresponding updates.
     */
    public void stopUpdating() {
        updateIntakeSubsystem = false;
        telemetry.addLine("[Intake] stop updating");
    }

    /**
     * Determines whether the scoring subsystem is at the target position.
     *
     * @return <code>true</code> if the arm and linear slide are at their respective target positions;
     * <code>false</code> if either is not at their target position.
     */
    public boolean isAtTarget() {
        return robot.arm.isAtTarget() && robot.linearSlide.isAtTarget();
    }

    /**
     * Internal method used to calculate the angle the arm should move to given a target length
     * and height for the claw position.
     *
     * @param length the target length for the claw.
     * @param height the height for the claw.
     * @return the angle to which to move the arm to reach the target length and height for the claw.
     */
    protected double getArmAngle(double length, double height) {
        double theta = Math.atan2(height, length);
        return Math.toDegrees(theta);
    }

    /**
     * Internal method to calculate the linear slide length given the horizontal distance that
     * scoring subsystem wants to reach along with the angle for the arm.
     *
     * @param horizontalDistance the horizontal distance the scoring subsystem wants to reach
     * @param armAngle           the current angle of the arm
     * @return the length of the linear slide to reach the desired horizontal distance
     */
    protected double getSlideLength(double horizontalDistance, double armAngle) {
        double armLength;
        // Handle the special case where the arm is horizontal to the field. The built-in trigonometric
        // functions don't work properly here, so we manually set the arm length to the target
        // horizontal distance.
        if (armAngle == 0.0) {
            armLength = horizontalDistance;
        } else {
            double theta = Math.toRadians(armAngle);
            double height = Math.tan(theta) * horizontalDistance;
            armLength = Math.hypot(horizontalDistance, height);
        }

        // Determine how much the linear slide needs to extend to reach the target length. This
        // accounts for the arm length when the linear slide is fully retracted.
        return armLength - RETRACTED_ARM_LENGTH;
    }

    /**
     * Sets the arm angle and linear slide lengths for the intake subsystem. This method is called
     * for each loop in the running OpMode, and adjust values as needed on each loop.
     */
    @Override
    public void execute() {
        // If we are moving the intake subsystem to a target position, update the controlled
        // subsystems. Otherwise, the execute method is a no-op, which allows the ScoringSubsystem
        // to manage the shared hardware
        if (updateIntakeSubsystem) {
            // Calculate and set the arm angle and linear slide length, if either has changed
            if (robot.arm.getShoulderAngle() != targetArmAngle) {
                robot.arm.setShoulderAngle(targetArmAngle);
                telemetry.addData("[Intake] arm angle", targetArmAngle);
            }
            double target = getSlideLength(targetArmAngle, targetArmReach);
            if (robot.linearSlide.getTargetLength() != target) {
                robot.linearSlide.setLength(target);
                telemetry.addData("[Intake] slide length", target);
            }

            // Once we reach the target positions, no longer update the values for the arm angle
            // and linear slide length
            if (isAtTarget()) {
                updateIntakeSubsystem = false;
                telemetry.addLine("[Intake] reached target");
            }
        }

        // Allow each of the scoring subsystem elements to perform any updates they need to.
        // Ideally, these would be called only once, but since the ScoringSubsystem and IntakeSubsystem
        // share the same underlying subsystem components, the execute() methods for each may be
        // called twice. This won't have any negative effect on the robot so it is safe to make
        // the calls.
        robot.arm.execute();
        robot.linearSlide.execute();
        robot.wrist.execute();
        robot.claw.execute();
    }
}
