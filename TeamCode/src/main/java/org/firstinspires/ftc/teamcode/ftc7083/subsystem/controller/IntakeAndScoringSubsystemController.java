package org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Manages the intake and scoring subsystem within a TeleOp OpMode. The following controls are
 * used to manage the subsystem:
 * <ul>
 *     <li>
 *         <em>gamepad2.dpad_down</em>: move the scoring subsystem to the low chamber position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *           <em>gamepad2.dpad_up</em>: move the scoring subsystem to the high chamber position. This will
 *           raise the arm and extend the linear slide.
 *     </li>
 *     <li>
 *          <em>gamepad2.dpad_left</em>: lowers the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *          <em>gamepad2.dpad_right</em>: raisess the arm on the scoring subsystem.
 *     </li>
 *     <li>
 *         <em>gamepad2.cross</em>: move the scoring subsystem to the low bucket position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.triangle</em>: move the scoring subsystem to the high bucket position. This will
 *         raise the arm and extend the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.square</em>: move the scoring subsystem into or out of the submersible. When
 *         moving into the submersible, this will lower the arm and extend the linear slide to a
 *         position relatively close to the front of the robot. When moving out of the submersible,
 *         this will slightly raise the arm and retract the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.circle</em>: move the scoring subsystem into or out of the submersible. When
 *         moving into the submersible, this will lower the arm and extend the linear slide to a
 *         position relatively far from the front of the robot. When moving out of the submersible,
 *         this will slightly raise the arm and retract the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.share</em>: move the scoring subsystem to a neutral position, where the arm is
 *         extended horizontal to the ground and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.option</em>: move the scoring subsystem to a starting position, where the arm is
 *         all the way down and the linear slide is fully retracted.
 *     </li>
 *     <li>
 *         <em>gamepad2.left_stick_y</em>: manually extend and retract the linear slide.
 *     </li>
 *     <li>
 *         <em>gamepad2.right_stick_y</em>: manually raise and lower the arm.
 *     </li>
 *     <li>
 *         <em>gamepad2.left_bumper</em>: open or close the claw.
 *     </li>
 * </ul>
 */
@Config
public class IntakeAndScoringSubsystemController implements SubsystemController {
    public static double MIN_JOYSTICK_VALUE = 0.5;
    public static double MANUAL_X_ADJUSTMENT = 0.5;
    public static double MANUAL_Y_ADJUSTMENT = 0.5;

    private final IntakeAndScoringSubsystem intakeAndScoringSubsystem;
    private final Telemetry telemetry;

    private final Gamepad previousGamepad1 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();

    private boolean inSubmersibleClose = false;
    private boolean inSubmersibleFar = false;
    private boolean clawOpen = false;
    private boolean atHighBasket = false;

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
        } else if (gamepad2.dpad_up && !previousGamepad2.dpad_up) {
            intakeAndScoringSubsystem.moveToChamberHighScoringPosition();
        } else if (gamepad2.dpad_left && !previousGamepad2.dpad_left) {
            intakeAndScoringSubsystem.raiseArm();
        } else if (gamepad2.dpad_right && !previousGamepad2.dpad_right) {
            intakeAndScoringSubsystem.lowerArm();
        } else if (gamepad2.cross && !previousGamepad2.cross) {
            intakeAndScoringSubsystem.moveToBasketLowScoringPosition();
        } else if (gamepad2.triangle && !previousGamepad2.triangle) {
            if(atHighBasket) {
                intakeAndScoringSubsystem.retractLinearSlide();
                atHighBasket = false;
            } else {
                intakeAndScoringSubsystem.moveToBasketHighScoringPosition();
                atHighBasket = true;
            }
        } else if (gamepad2.square && !previousGamepad2.square) {
            if (inSubmersibleClose) {
                intakeAndScoringSubsystem.retractLinearSlide();
                inSubmersibleClose = false;
            } else {
                intakeAndScoringSubsystem.moveToIntakeShortPosition();
                inSubmersibleClose = true;
            }
            inSubmersibleFar = false;
        } else if (gamepad2.circle && !previousGamepad2.circle) {
            if (inSubmersibleFar) {
                intakeAndScoringSubsystem.retractLinearSlide();
                inSubmersibleFar = false;
            } else {
                intakeAndScoringSubsystem.moveToIntakeLongPosition();
                inSubmersibleFar = true;
            }
            inSubmersibleClose = false;
        } else if (gamepad2.share && !previousGamepad2.share) {
            intakeAndScoringSubsystem.moveToNeutralPosition();
        } else if (gamepad2.options) {
            intakeAndScoringSubsystem.moveToStartPosition();
        }

        // Manual override controls for the arm and linear slide. The left joystick will raise and
        // lower the arm; the right joystick will extend and retract the linear slide.
       /* if (Math.abs(gamepad2.left_stick_y) > MIN_JOYSTICK_VALUE) {
            double adjustY = MANUAL_Y_ADJUSTMENT;
            if (gamepad2.left_stick_y > 0.0) {
                adjustY *= -1;
            }
            intakeAndScoringSubsystem.adjustY(adjustY);
            telemetry.addData("[IAS C] adj arm height", adjustY);
        }
        if (Math.abs(gamepad2.right_stick_y) > MIN_JOYSTICK_VALUE) {
            double adjustX = MANUAL_X_ADJUSTMENT;
            if (gamepad2.right_stick_y > 0.0) {
                adjustX *= -1;
            }
            intakeAndScoringSubsystem.adjustX(adjustX);
            telemetry.addData("[IAS C] adj slide length", adjustX);
        }*/

        // Open and close the claw; used for acquiring samples/specimens and scoring
        // or depositing them
        if (gamepad2.left_bumper && !previousGamepad2.left_bumper) {
            if (clawOpen) {
                intakeAndScoringSubsystem.closeClaw();
                clawOpen = false;
            } else {
                intakeAndScoringSubsystem.openClaw();
                clawOpen = true;
            }
        }

        // Update the scoring subsystem. This allows it to adjust the position of the managed
        // components as they move to a target position.
        intakeAndScoringSubsystem.execute();

        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);
    }
}
