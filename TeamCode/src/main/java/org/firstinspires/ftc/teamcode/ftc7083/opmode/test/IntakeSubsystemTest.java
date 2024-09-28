package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.Robot;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.IntakeSubsystemController;

@TeleOp(name = "Intake Subsystem Test", group = "tests")
public class IntakeSubsystemTest extends OpMode {
    private IntakeSubsystemController intakeSubsystemController;
    private final Gamepad currentGamepad1 = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = Robot.init(hardwareMap, telemetry);
        intakeSubsystemController = new IntakeSubsystemController(robot.intakeSubsystem, telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        intakeSubsystemController.execute(currentGamepad1, currentGamepad2);

        telemetry.update();
    }
}
