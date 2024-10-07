package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import org.firstinspires.ftc.teamcode.ftc7083.shared.Position2d;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

/**
 * This class contains methods to convert Position3d into length and theta and yaw values that
 * can be used by the slide, shoulder, and wrist to navigate to the x, yDirection, and z values specified
 * in Position3d. It also contains a method to keep the wrist parallel to the ground even as the
 * arm moves up and down.
 */
public class ArmSubsystemController {
    double armHeight;
    Wrist wrist;
    Arm arm;
    LinearSlide linearSlide;
    Position2d currentPosition;

    /**
     *The Constructor
     *
     * @param armHeight is the height of the shoulder from the ground in inches. Assumed to be fixed.
     * @param wrist is an instance of the wrist
     */
    public ArmSubsystemController(double armHeight, Wrist wrist, Arm arm, LinearSlide linearSlide, Position2d startingPosition) {
        this.armHeight = armHeight;
        this.wrist = wrist;
        this.arm = arm;
        this.linearSlide = linearSlide;
        this.currentPosition = startingPosition;

    }

    /**
     * This method takes a Position2d object and moves to that position using calculated
     * length and armAngle values.
     *
     * @param targetPosition the target position in inches x,z
     * @param intakeMode the mode which keeps the pitch at such an angle that the claw stays in optimal intake position.
     */
    public void moveToPosition(Position2d targetPosition, boolean intakeMode) {
        double targetAngle = calculateArmAngle(targetPosition);
        double targetLength = calculateLength(targetPosition);
        arm.setShoulderAngle(targetAngle);
        linearSlide.setLength(targetLength);

        if(intakeMode) {
            double wristPitch = calculateIntakeModeWristPitch(targetAngle);
            wrist.setPitch(wristPitch);
        }
    }

    /**
     * Provides the Position2d as x, z, and the angle of the wrist in the yDirection direction.
     */
    public Position2d getCurrentPosition2d() {
    double armAngle = arm.getShoulderAngle();
    double slideLength = linearSlide.getCurrentLength();

    currentPosition.x = getX(armAngle, slideLength);
    currentPosition.z = getZ(armAngle, slideLength);
    return currentPosition;
    }

    /**
     * This method calculates the length the arm should extend to in order to
     * get to the position described by Position2d. All values are in inches.
     *
     * @return the length in inches
     */
    private double calculateLength(Position2d targetPosition) {

        if(targetPosition.x > 0){
            return Math.sqrt(Math.pow(targetPosition.x, 2) + Math.pow((armHeight - targetPosition.z), 2));
        } else if (targetPosition.x < 0) {
            return -Math.sqrt(Math.pow(targetPosition.x, 2) + Math.pow((armHeight - targetPosition.z), 2));
        } else {
            return 0.0;
        }
    }

    /**
     * This method calculates the angle the arm should be at in order to
     * get to the position described by Position2d. All input values are in inches and 0° is at horizontal.
     *
     * @return the angle (theta) in degrees
     */
    private double calculateArmAngle(Position2d targetPosition) {
        double calculatedArmAngle = 0.0;
        double realArmAngle = 0.0;

        if (targetPosition.z != armHeight && targetPosition.x != 0.0) {
            calculatedArmAngle = Math.atan((targetPosition.z - armHeight) / targetPosition.x);
        } else if (targetPosition.z == armHeight){
            realArmAngle = 0.0;
        }

        if (targetPosition.x > 0.0) {
            realArmAngle = Math.toDegrees(calculatedArmAngle);
        } else if (targetPosition.x < 0.0) {
            realArmAngle = Math.toDegrees(calculatedArmAngle) + 180;
        } else {
            if (targetPosition.z < armHeight) {
                realArmAngle = -90;
            } else if (targetPosition.z > armHeight) {
                realArmAngle = 90;
            }
        }

        return realArmAngle;
    }

    /**
     * Calculates the x value of the end effector using the arm angle
     * and slide length as inputs.
     *
     * @param armAngle current anble of the arm
     * @param slideLength current length of the arm
     * @return the x value
     */
    public double getX(double armAngle, double slideLength) {
        double armAngleRad = Math.toRadians(armAngle);

        if(armAngle < 0) {
            return Math.sin(armAngleRad + (Math.PI/2)) * slideLength;
        } else if(armAngle > 0) {
            return Math.cos(armAngleRad) * slideLength;
        } else {
            return slideLength;
        }
    }

    /**
     * Calculates the x value of the end effector using the arm angle
     * and slide length as inputs.
     *
     * @param armAngle the angle of the arm in degrees
     * @param slideLength the length of the slide in inches
     * @return the z value in inches
     */
    public double getZ(double armAngle, double slideLength) {
        double armAngleRad = Math.toRadians(armAngle);

        if (armAngle < 0) {
            return armHeight - slideLength * Math.cos(armAngleRad + (Math.PI / 2));
        } else if(armAngle > 0) {
            return slideLength * Math.sin(armAngleRad) + armHeight;
        } else {
            return 15.0;
        }

    }

    /**
     * This method returns the pitch of the wrist such that the wrist
     * is parallel to the ground.
     *
     * @param armAngle this is the angle of the arm, with 0° when the arm is horizontal and negative below that
     */
    private double calculateIntakeModeWristPitch(double armAngle) {
        return (180 - armAngle);
    }

    /**
     * returns a calculated feedforward value specifically for the shoulder motor to be added instead of the i term
     * @return the feedforward value
     */
    public double armFeedforward () {
        double Ka = 0.0;
        return Ka * linearSlide.getCurrentLength();
    }
}
