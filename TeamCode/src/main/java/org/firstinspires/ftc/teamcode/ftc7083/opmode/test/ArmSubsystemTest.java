package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.shared.Position2d;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.LinearSlide;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.ArmSubsystemController;

@Config
@TeleOp(name = "Arm Subsystem Cumulative Test",group = "Test")
public class ArmSubsystemTest extends OpMode {

    public static double armHeight = 40.0;
    public static double xTargetPosition = 0.0;
    public static double zTargetPosition = 0.0;

    Wrist wrist;
    Arm arm;
    LinearSlide linearSlide;
    ArmSubsystemController armSubsystemController;
    private final Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        wrist = new Wrist(hardwareMap,telemetry);
        arm = new Arm(hardwareMap,telemetry);
        linearSlide = new LinearSlide(hardwareMap,telemetry);

        armSubsystemController = new ArmSubsystemController(armHeight, wrist, arm, linearSlide);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values are used within each subsystem controller.
        currentGamepad1.copy(gamepad1);

        Position2d targetPosition = new Position2d(xTargetPosition,zTargetPosition);

        armSubsystemController.moveToPosition(targetPosition,true);

        targetPosition.x = gamepad1.right_stick_x;
        targetPosition.z = -gamepad1.right_stick_y;

        telemetry.addData("x value: ",targetPosition.x);
        telemetry.addData("z value: ",targetPosition.z);

    }
}
