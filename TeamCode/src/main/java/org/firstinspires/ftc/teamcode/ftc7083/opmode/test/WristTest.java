package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Wrist;

@Config
@TeleOp(name = "Wrist Test TeleOp", group = "Active")
public class WristTest extends OpMode {

    public static double PITCH = 0.0;
    public static double YAW = 0.0;
    Wrist wrist;

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
        wrist.setPitch(PITCH);
        wrist.setYaw(YAW);

        telemetry.addData("Pitch",wrist.getPitchPosition());
        telemetry.addData("Yaw",wrist.getYawPosition());
        telemetry.update();
    }

}
