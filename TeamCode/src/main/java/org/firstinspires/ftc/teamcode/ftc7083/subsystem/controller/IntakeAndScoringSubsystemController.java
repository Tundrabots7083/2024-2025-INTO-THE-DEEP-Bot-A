package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Manages the intake subsystem within a TeleOp OpMode. The following controls are used to manage
 * the intake subsystem:
 * <ul>
 *     <li>
 *         <em>gamepad2.dpad_down</em>: move the scoring subsystem to the low chamber position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *           <li>
 *           <em>gamepad2.dpad_up</em>: move the scoring subsystem to the high chamber position. This will
 *           raise the arm and extend the linear slide.
 *       </li>]
 *     <li>
 *         <em>gamepad2.cross</em>: move the scoring subsystem to the low bucket position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.triangle</em>: move the scoring subsystem to the high bucket position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.circle</em>: move the sco9ring subsystem to the intake position. This will
 *         lower the arm and extend the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.square</em>: move the scoring subsystem to a neutral position, where the arm is
 *         extended horizontal to the ground and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.left_stick_y</em>: manually extend and retract the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.right_stick_y</em>: manually raise and lower the arm.
 *     </li>
 *     <li>
 *         <em>gamepad2.left_bumper</em>: open the claw.
 *     </li>
 *     <li>
 *         <em>gamepad2.right_bumper</em>: close the claw.
 *     </li>
 * </ul>
 */
@Config
public class IntakeAndScoringSubsystemController implements SubsystemController {
    public static double SCALAR_X = 0.25;
    public static double SCALAR_Y = 0.25;

    private final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private final Telemetry telemetry;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    /**
     * Instantiate a scoring subsystem controller, which uses gamepad controls to control the
     * scoring subsystem.
     *
     * @param intakeAndScoringSubsystem the scoring subsystem being controlled
     * @param telemetry                 the telemetry used to provide user output on the driver station and FTC dashboard
     */
    public IntakeAndScoringSubsystemController(IntakeAndScoringSubsystem intakeAndScoringSubsystem, Telemetry telemetry) {
        this.intakeAndScoringSubsystem = intakeAndScoringSubsystem;
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
        // Preset positions for the arm and linear slide
        if (gamepad2.dpad_down && !previousGamepad2.dpad_down) {
            intakeAndScoringSubsystem.moveToChamberLowScoringPosition();
            telemetry.addData("[Scoring] position", "low chamber");
        } else if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
            telemetry.addData("[Scoring] position", "high chamber");
        } else if (gamepad2.cross && !previousGamepad2.cross) {
            intakeAndScoringSubsystem.moveToBasketLowScoringPosition();
            telemetry.addData("[Scoring] position", "low basket");
        } else if (gamepad2.triangle && !previousGamepad2.triangle) {
            intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
            telemetry.addData("[Scoring] position", "high basket");
        } else if (gamepad2.circle && !previousGamepad2.circle) {
            intakeAndScoringSubsystem.moveToIntakePosition();
            telemetry.addData("[Scoring] position", "intake");
        } else if (gamepad2.square && !previousGamepad2.square) {
            intakeAndScoringSubsystem.moveToNeutralPosition();
            telemetry.addData("[Scoring] position", "neutral");
        }

        // Manual override controls for the arm and linear slide
        if (gamepad2.left_stick_y != 0.0) {
            double adjustY = -gamepad2.left_stick_y * SCALAR_Y;
            double y = intakeAndScoringSubsystem.getCurrentY() + adjustY;
            double x = intakeAndScoringSubsystem.getCurrentX();
            intakeAndScoringSubsystem.moveToPosition(x, y);
            telemetry.addData("[Scoring] adjust Y", adjustY);
        }
        if (gamepad2.right_stick_y != 0.0) {
            double adjustX = -gamepad2.right_stick_y * SCALAR_X;
            double y = intakeAndScoringSubsystem.getCurrentY();
            double x = intakeAndScoringSubsystem.getCurrentX() + adjustX;
            intakeAndScoringSubsystem.moveToPosition(x, y);
            telemetry.addData("[Scoring] adjust X", adjustX);
        }

        // Open and close the claw; used for acquiring samples/specimens and scoring
        // or depositing them
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            intakeAndScoringSubsystem.openClaw();
            telemetry.addData("[Scoring] claw", "open");
        } else if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            intakeAndScoringSubsystem.closeClaw();
            telemetry.addData("[Scoring] claw", "close");
        }

        // Update the scoring subsystem. This allows it to adjust the position of the managed
        // components as they move to a target position.
        intakeAndScoringSubsystem.execute();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
