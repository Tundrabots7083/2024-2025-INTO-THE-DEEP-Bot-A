package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeSubsystem;

/**
 * Manages the intake subsystem within a TeleOp OpMode. The following controls are used to manage
 * the intake subsystem:
 * <ul>
 *     <li>
 *         <em>gamepad1.dpad_down</em>: move the scoring subsystem to an intake position. This will
 *         lower the arm and extend the linear slide.
 *     </li>
 *           <li>
 *           <em>gamepad1.dpad_left</em>: move the scoring subsystem to a neutral position. This will
 *           raise the arm and retract the linear slide.
 *       </li>]
 *     <li>
 *         <em>gamepad1.cross</em>: acquire a scoring element. This will close the claw on the
 *         scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad1.triangle</em>: deposition a scoring element. This will open the claw on
 *         the scoring subsystem.
 *     </li>
 * </ul>
 */
public class IntakeSubsystemController {
    private final IntakeSubsystem intakeSubsystem;
    private final Telemetry telemetry;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    public IntakeSubsystemController(IntakeSubsystem intakeSubsystem, Telemetry telemetry) {
        this.intakeSubsystem = intakeSubsystem;
        this.telemetry = telemetry;
    }

    /**
     * Updates the robot drive speed based on the left and right joysticks on gamepad1. In addition,
     * the left bumper on gamepad1 can be used to toggle the gain for the X axis, Y axis and
     * turn speed.
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    public void execute(Gamepad gamepad1, Gamepad gamepad2) {
        if (gamepad1.dpad_down && !previousGamepad1.dpad_down) {
            intakeSubsystem.moveToIntakePosition();
        }
        if (gamepad1.dpad_left && !previousGamepad1.dpad_left) {
            intakeSubsystem.moveToNeutralPosition();
        }
        if (gamepad1.cross && !previousGamepad1.cross) {
            intakeSubsystem.acquireScoringElement();
        }
        if (gamepad1.triangle && !previousGamepad1.triangle) {
            intakeSubsystem.depositScoringElement();
        }

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
