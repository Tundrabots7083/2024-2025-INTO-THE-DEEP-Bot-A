package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import org.firstinspires.ftc.teamcode.ftc7083.shared.Pose2d;
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

    /**
     *The Constructor
     *
     * @param armHeight is the height of the shoulder from the ground in inches. Assumed to be fixed.
     * @param wrist is an instance of the wrist
     */
    public ArmSubsystemController(double armHeight, Wrist wrist, Arm arm, LinearSlide linearSlide) {
        this.armHeight = armHeight;
        this.wrist = wrist;
        this.arm = arm;
        this.linearSlide = linearSlide;

    }

    /**
     * This method takes a Position3d object and moves to that position using calculated
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
     * Provides the Pose2d as x, z, and the angle of the wrist in the yDirection direction.
     */
    public Pose2d getCurrentPose2d() {
    double armAngle = arm.getShoulderAngle();
    double slideLength = linearSlide.getCurrentLength();

    double z = - slideLength * Math.cos(armAngle) - this.armHeight;
    double x = Math.tan(armAngle) * (this.armHeight - z);
    double yDirection = wrist.getYawPosition();

    return new Pose2d(x,z,yDirection);
    }

    /**
     * This method calculates the length the arm should extend to in order to
     * get to the position described by Position3d. All values are in inches.
     *
     * @return the length in inches
     */
    private double calculateLength(Position2d targetPosition) {

        return targetPosition.x /
                Math.sin(Math.atan(targetPosition.x /
                (armHeight - targetPosition.z)));
    }

    /**
     * This method calculates the angle the arm should be at in order to
     * get to the position described by Position3d. All input values are in inches and 0° is at horizontal.
     *
     * @return the angle (theta) in degrees
     */
    private double calculateArmAngle(Position2d targetPosition) {

        double armAngle = Math.atan(targetPosition.x /
                armHeight - targetPosition.z);

        return Math.toDegrees(armAngle);
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
        double Ka = 0.05;
        return Ka * linearSlide.getCurrentLength();
    }
}
