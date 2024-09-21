package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Claw;

/**
 * This OpMode tests the claw using FTC Dashboard (http://192.168.43.1:8080/dash)
 * to change the angle in degrees that the claw is opened to.
 */
@Config
@TeleOp(name = "Claw Test TeleOp", group = "Test")
public class ClawTest extends OpMode {

    public static double MAX_CLAW_DEGREES = 270;
    public static double CLAW_OPEN_DEGREES = 0.0;
    Claw claw;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        claw = new Claw(hardwareMap, "clawServo", MAX_CLAW_DEGREES);

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        claw.open(CLAW_OPEN_DEGREES);
        telemetry.addData("Open Degrees",claw.getDegrees());
        telemetry.update();
    }

}