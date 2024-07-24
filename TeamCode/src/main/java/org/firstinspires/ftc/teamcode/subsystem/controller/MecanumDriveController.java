package org.firstinspires.ftc.teamcode.subsystem.controller;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.MecanumDrive;

/**
 * A controller for a mecanum drive chassis.
 */
@Config
public class MecanumDriveController implements SubsystemController {

    public static double MAX_TURNING_MULT = 0.75; //Max turning speed multiplier
    public static double MAX_DRIVE_MULT = 1; //Max drive speed multiplier
    public static double SLOW_TURNING_MULT = 0.6; //Slow turning speed multiplier
    public static double SLOW_DRIVE_MULT = 0.75; //Slow drive speed multiplier
    private final MecanumDrive mecanumDrive;
    private final Telemetry telemetry;
    private double driveGain;
    private double turnGain;

    private boolean bumperPressed = false;

    /**
     * Creates a controller for a mecanum drive chassis.
     *
     * @param mecanumDrive the mecanum drive to control.
     * @param telemetry    the telemetry used to display data on the driver station.
     */
    public MecanumDriveController(MecanumDrive mecanumDrive, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.mecanumDrive = mecanumDrive;
        this.driveGain = MAX_DRIVE_MULT;
        this.turnGain = MAX_TURNING_MULT;
    }

    /**
     * Updates the percentage of the power that is applied to the drive train. Ths affects
     * the X and Y axis (driveGain) and the turn speed (turnGain).
     *
     * @param gamepad the gamepad with the controls for adjusting the robot gain
     */
    private void setGain(Gamepad gamepad) {
        // Toggle the drive and turn gains as the left bumper is pressed
        if (!bumperPressed && gamepad.left_bumper) {
            driveGain = driveGain == MAX_DRIVE_MULT ? SLOW_DRIVE_MULT : MAX_DRIVE_MULT;
            turnGain = turnGain == MAX_TURNING_MULT ? SLOW_TURNING_MULT : MAX_TURNING_MULT;
        }
        bumperPressed = gamepad.left_bumper;
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

        double x = gamepad1.left_stick_x * driveGain;
        double y = -gamepad1.left_stick_y * driveGain;
        double turn = -gamepad1.right_stick_x * turnGain;
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
                ", driveGain=" + driveGain +
                ", turnGain=" + turnGain +
                '}';
    }
}
