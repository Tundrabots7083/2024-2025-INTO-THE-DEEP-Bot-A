package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.action.ActionEx;
import org.firstinspires.ftc.teamcode.ftc7083.action.SequentialAction;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.IntakeAndScoringSubsystem;

/**
 * Sample test of the intake and scoring subsystem that uses actions to acquire a sample.
 */
@Config
@TeleOp(name = "Intake and Scoring Subsystem Test", group = "tests")
public class IntakeAndScoringSubystemTest extends LinearOpMode {
    private static double TARGET_X = IntakeAndScoringSubsystem.INTAKE_SHORT_X;
    private static double TARGET_Y = IntakeAndScoringSubsystem.INTAKE_SHORT_Y;

    /**
     * Moves the scoring subsystem to the intake position, closes the claw to capture a sample,
     * and retracts the linear slide with the sample.
     *
     * @throws InterruptedException the OpMode was interrupted
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Equivalent to "init" in an OpMode
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Robot robot = Robot.init(hardwareMap, telemetry);

        // Create an action that acquire a sample using the intake and scoring subsystem
        ActionEx setIntakeAndScoringSubsystemPosition = new SequentialAction(
                robot.intakeAndScoringSubsystem.actionMoveTo(TARGET_X, TARGET_Y),
                robot.claw.actionCloseClawWithWait(),
                robot.intakeAndScoringSubsystem.actionRetractLinearSlide()
        );
        boolean finished = false;
        robot.claw.open();

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        // Equivalent to "loop" in an OpMode
        while (opModeIsActive() && !finished) {
            // Clear the bulk cache for each Lynx module hub. This must be performed once per loop
            // as the bulk read caches are being handled manually.
            for (LynxModule hub : robot.allHubs) {
                hub.clearBulkCache();
            }

            // Acquire the sample
            finished = !setIntakeAndScoringSubsystemPosition.run(new TelemetryPacket());

            telemetry.update();
        }
    }
}
