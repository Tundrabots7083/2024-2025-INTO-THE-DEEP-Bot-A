package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.MecanumDrive;

/**
 * A controller for a mecanum drive chassis. The following controls on <em>gamepad1</em> are used
 * for by the drive subsystem.
 * <ul>
 *     <li>
 *         <em>gamepad1.left_stick</em>: used to move the robot forward and backwards, and strafe left
 *         and right. The Y value is used for moving forward and backwards; the X value is used for
 *         moving left and right.
 *     </li>
 *     <li>
 *         <em>gamepad1.right_stick</em>: rotate the robot clockwise or counterclockwise. The X value
 *         is used to determine the rotation rate; a positive value rotates the robot clockwise, while
 *         a negative value rotates the robot counterclockwise.
 *     </li>
 *     <li>
 *         <em>gamepad1.left_bumper</em>: toggle the drive speed gain between a <em>fast</em> and
 *         <em>slow</em> mode.
 *     </li>
 * </ul>
 */
@Config
public class MecanumDriveController implements SubsystemController {
    public static double TURN_MULTIPLIER = 0.75; // Max turning speed multiplier
    public static double DRIVE_GAIN_MAX = 1; // Fast drive speed gain
    public static double DRIVE_GAIN_MIN = 0.4; // Slow drive speed gain

    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;

    private boolean previousLeftBumper = false;
    private boolean fastMode;

    /**
     * Creates a controller for a mecanum drive chassis.
     *
     * @param mecanumDrive the mecanum drive to control.
     * @param telemetry    the telemetry used to display data on the driver station.
     */
    public MecanumDriveController(MecanumDrive mecanumDrive, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.mecanumDrive = mecanumDrive;

        mecanumDrive.setDriveGain(DRIVE_GAIN_MAX);
        fastMode = true;
    }

    /**
     * Updates the percentage of the power that is applied to the drive train. Ths affects
     * the X and Y axis (driveGain) and the turn speed (turnGain).
     *
     * @param gamepad the gamepad with the controls for adjusting the robot gain
     */
    private void setGain(Gamepad gamepad) {
        // Toggle the drive gain as the left bumper is pressed
        if (gamepad.left_bumper && !previousLeftBumper) {
            if (fastMode) {
                mecanumDrive.setDriveGain(DRIVE_GAIN_MIN);
                fastMode = false;
            } else {
                mecanumDrive.setDriveGain(DRIVE_GAIN_MAX);
                fastMode = true;
            }
        }
        previousLeftBumper = gamepad.left_bumper;
    }

    /**
     * Updates the robot drive speed based on the left and right joysticks on gamepad1. In addition,
     * the left bumper on gampedad1 can be used to toggle the gain for the X axis, Y axis and
     * turn speed.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        setGain(gamepad1);

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x * TURN_MULTIPLIER;
        telemetry.addData("[DRIVE] X", x);
        telemetry.addData("[DRIVE] Y", y);
        telemetry.addData("[DRIVE] Turn", turn);

        mecanumDrive.drive(x, y, turn);
    }

    /**
     * Gets a string representation of this mecanum drive controller.
     *
     * @return a string representation of this mecanum drive controller
     */
    @NonNull
    @Override
    public String toString() {
        return "MecanumDriveController{" +
                "mecanumDrive=" + mecanumDrive +
                "fastMode=" + fastMode +
                '}';
    }
}
