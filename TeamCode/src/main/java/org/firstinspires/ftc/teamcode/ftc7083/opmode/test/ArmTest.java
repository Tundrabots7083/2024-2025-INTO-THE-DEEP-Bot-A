package org.firstinspires.ftc.teamcode.ftc7083.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ftc7083.subsystem.Arm;

@Config
@TeleOp(name = "Arm Testing", group = "Tests")
public class ArmTest extends OpMode {

    public static double ARM_SLIDE_LENGTH = 20.0;
    public static double ARM_SHOULDER_ANGLE = 45.0;
    private Arm arm;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(hardwareMap, telemetry);
        telemetry.addLine("Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        arm.setSholderAngle(ARM_SHOULDER_ANGLE);
        arm.setSlideLength(ARM_SLIDE_LENGTH);
        arm.execute();
    }
}
