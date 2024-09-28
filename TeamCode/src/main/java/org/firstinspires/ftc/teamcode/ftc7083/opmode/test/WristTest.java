package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.ftc7083.subsystem.controller.SubsystemController;

/**
 * This OpMode tests the wrist using FTC Dashboard (http://192.168.43.1:8080/dash)
 * to change the Pitch and Yaw servo positions.
 */
@Config
@TeleOp(name = "Wrist Test TeleOp", group = "tests")
public class WristTest extends OpMode {

    public static double PITCH = 0.0;
    public static double YAW = 0.0;
    public double gamepadPitch;
    public double gamepadYaw;
    Wrist wrist;
    private final Gamepad currentGamepad1 = new Gamepad();

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
      
        wrist = new Wrist(hardwareMap,telemetry);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Always get a copy of the current gamepads. The gamepads are updated in near real time,
        // so this ensures that a consistent set of values are used within each subsystem controller.
        currentGamepad1.copy(gamepad1);

        this.gamepadPitch = -gamepad1.right_stick_y * 50;
        this.gamepadYaw = gamepad1.right_stick_x * 80;

        wrist.setPosition(gamepadPitch, gamepadYaw);

        telemetry.addData("Front/Right Servo: ",wrist.getFrontServoDegrees());
        telemetry.addData("Back/Left Servo: ",wrist.getBackServoDegrees());
        telemetry.update();
    }
}
